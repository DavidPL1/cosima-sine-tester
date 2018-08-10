/* Author: David Leins
 * Date:   1 August 2018
 *
 */

#pragma once

#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/os/Timer.hpp>
#include <string>
#include <Eigen/Dense>

#include <rst-rt/geometry/Pose.hpp>
#include <rst-rt/kinematics/Twist.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/robot/JointState.hpp>

class SineTester : public RTT::TaskContext {

public:
    SineTester(std::string const & name);

    bool configureHook();
    bool startHook();
    void stopHook();
    void cleanupHook();
    void updateHook();

    void testFrequs();

    void setAngularFrequencies(float a, float b, float c, float d, float e);

    void setFrequencies(float a, float b, float c, float d, float e, float f, float g);
    void getNewAngles();
    
    double getSimulationTime();

protected:
    // Ports
    RTT::OutputPort<rstrt::kinematics::JointAngles> out_desiredJointSpacePosition_port;
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::InputPort<rstrt::kinematics::JointAngles> in_frequencies_right_port;
    RTT::InputPort<rstrt::kinematics::JointAngles> in_frequencies_left_port;
    
    // Data Flow
    RTT::FlowStatus out_desiredJointSpacePosition_flow;
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_frequencies_left_flow;
    RTT::FlowStatus in_frequencies_right_flow;

    // Variables
    rstrt::kinematics::JointAngles out_desiredJointSpacePosition_var;
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::kinematics::JointAngles in_frequencies_left_var;
    rstrt::kinematics::JointAngles in_frequencies_right_var;

    Eigen::VectorXf currentFrequencies; 
    Eigen::VectorXf lastFrequencies;

    Eigen::VectorXf currentAngles;
    Eigen::VectorXf desiredAngles;
    Eigen::VectorXf isJointReady;

    Eigen::VectorXf posMax;
    
    Eigen::VectorXd startTimes, time_diff, current_times;

    RTT::os::TimeService::ticks start_ticks;
    double wait_time;
    int recoveryFactor;
};