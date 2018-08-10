/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#include <SineTester.hpp>
#include <rtt/Component.hpp> // needed for the macro at the end of this file

using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

SineTester::SineTester(const std::string &name): RTT::TaskContext(name) {

    in_robotstatus_var = rstrt::robot::JointState(7);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    in_frequencies_left_var = rstrt::kinematics::JointAngles(7);
    in_frequencies_left_port.setName("in_frequencies_left_port");
    in_frequencies_left_port.doc("Port to set frequencies for the left arm");
    ports()->addPort(in_frequencies_left_port);
    in_frequencies_left_flow = RTT::NoData;

    in_frequencies_right_var = rstrt::kinematics::JointAngles(7);
    in_frequencies_right_port.setName("in_frequencies_right_port");
    in_frequencies_right_port.doc("Port to set frequencies for the right arm");
    ports()->addPort(in_frequencies_right_port);
    in_frequencies_right_flow = RTT::NoData;

    out_desiredJointSpacePosition_var = rstrt::kinematics::JointAngles(7);

    out_desiredJointSpacePosition_port.setName("out_desiredJointSpacePosition_port");
    out_desiredJointSpacePosition_port.doc("Output port to send the next desired position of the trajectory to the robot");
    out_desiredJointSpacePosition_port.setDataSample(out_desiredJointSpacePosition_var);

    ports()->addPort(out_desiredJointSpacePosition_port);

    wait_time = 2.0;
    recoveryFactor = 10;

    isJointReady = Eigen::VectorXf::Zero(7);
    currentFrequencies = Eigen::VectorXf::Zero(7);
    lastFrequencies = Eigen::VectorXf::Zero(7);
    startTimes = Eigen::VectorXd::Zero(7);
    posMax = Eigen::VectorXf::Zero(7);

    posMax << 2.96705972839, 2.09439510239, 2.96705972839, 2.09439510239, 2.96705972839, 2.09439510239, 2.96705972839;

    addProperty("currentFrequencies", currentFrequencies);
    addProperty("recoveryFactor", recoveryFactor);

    RTT::log(RTT::Info) << "Done setting up" << RTT::endlog();
}

double SineTester::getSimulationTime() {
    return 1E-9
            * RTT::os::TimeService::ticks2nsecs(
                    RTT::os::TimeService::Instance()->getTicks());
}

bool SineTester::startHook() {
    startTimes.setOnes();
    startTimes *= this->getSimulationTime();
    return true;
}

bool SineTester::configureHook() {
    return true;
}

void SineTester::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void SineTester::cleanupHook() {
    // cleaning the component data
    wait_time = 0.0;
}

void SineTester::setFrequencies(float a, float b, float c, float d, float e, float f, float g) {
    currentFrequencies(0) = a;
    currentFrequencies(1) = b;
    currentFrequencies(2) = c;
    currentFrequencies(3) = d;
    currentFrequencies(4) = e;
    currentFrequencies(5) = f;
    currentFrequencies(6) = g;
}

void SineTester::getNewAngles() {
    
}

void SineTester::updateHook() {
    current_times = (Eigen::VectorXd::Ones(7) * this->getSimulationTime() - startTimes);
    time_diff = this->current_times - Eigen::VectorXd::Ones(7) * this->wait_time;

    //Update current position information
    if (in_robotstatus_port.connected()) {
    //    RTT::log(RTT::Info) << "Robotstatus port connected!" << RTT::endlog();
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        return;
    }

    if (in_frequencies_left_port.connected()) {
        in_frequencies_left_flow = in_frequencies_left_port.read(in_frequencies_left_var);
        in_frequencies_right_flow = in_frequencies_right_port.read(in_frequencies_right_var);
    }

    if (in_frequencies_right_flow == RTT::NewData) {
        RTT::log(RTT::Info) << "Got new frequencies for right arm!" << RTT::endlog();
        currentFrequencies = in_frequencies_right_var.angles;
    }

    currentAngles = in_robotstatus_var.angles;
    desiredAngles = currentAngles;

    for (unsigned int i=0; i < 7; i++) {

        if(time_diff(i) < 0.0) {
            time_diff(i) = 0; 
        }

        // Does the current joint need a reset?
        if (lastFrequencies(i) - currentFrequencies(i) != 0) {
            RTT::log(RTT::Fatal) << "Reset joint " << i << RTT::endlog();
            isJointReady(i) = 0;
        }

        // If joint is not ready try to reach initial position slowly
        if (isJointReady(i) == 0) {
            RTT::log(RTT::Fatal) << "joint " << i << " not ready: " << fabs(currentAngles(i)) << RTT::endlog();
            desiredAngles(i) = currentAngles(i) -   currentAngles(i)/recoveryFactor;
            
            // Is close enough to start position?
            if (fabs(currentAngles(i)) < 0.01) {
                RTT::log(RTT::Fatal) << "Resetting joint " << i << " done " << RTT::endlog();
                isJointReady(i) = 1;
                startTimes(i) = this->getSimulationTime();
                time_diff(i) = 0;
                RTT::log(RTT::Fatal) << "first joint pos: " << sin(currentFrequencies(i) * time_diff(i)) << RTT::endlog();
            }
        }
        
        if (isJointReady(i) == 1) {
            if (currentFrequencies(i) != 0) { // is frequency set? if not do not change the joint position.
                desiredAngles(i) = sin(currentFrequencies(i) * time_diff(i));
            }
        }
    }

    out_desiredJointSpacePosition_var.angles = desiredAngles; 
    out_desiredJointSpacePosition_port.write(out_desiredJointSpacePosition_var);
    
    lastFrequencies = currentFrequencies;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(SineTester)