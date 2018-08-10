import rsb
from rst.kinematics.JointAngles_pb2 import JointAngles
from rsb.converter import ProtocolBufferConverter

converter = ProtocolBufferConverter(messageClass=JointAngles)
rsb.converter.registerGlobalConverter(converter)

rsb.setDefaultParticipantConfig(rsb.ParticipantConfig.fromDefaultSources())

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

class JointInfo():
    def __init__(self, name, initial, min_pos, max_pos):
        self.name = name
        self.initial = initial
        self.max = max_pos
        self.min = min_pos


class MainWindow(Gtk.Window):
    # any signal from the scales is signaled to the label the text of which is
    # changed
    def scale_moved(self, event, scale, name):
        print("Horizontal scale is " + name + " " + str(float(scale.get_value())));
        # if name == 'WaistLat':
        #     print "Setting " + name
        #     self.commandedR[0] = scale.get_value()
        # elif name == 'WaistSag':
        #     print "Setting " + name
        #     self.commandedR[1] = scale.get_value()
        # elif name == 'WaistYaw':
        #     print "Setting " + name
        #     self.commandedR[2] = scale.get_value()
        if name == 'rj0':
            print "Setting " + name
            self.commandedR[0] = scale.get_value()
        elif name == 'rj1':
            print "Setting " + name
            self.commandedR[1] = scale.get_value()
        elif name == 'rj2':
            print "Setting " + name
            self.commandedR[2] = scale.get_value()
        elif name == 'rj3':
            print "Setting " + name
            self.commandedR[3] = scale.get_value()
        elif name == 'rj4':
            print "Setting " + name
            self.commandedR[4] = scale.get_value()
        elif name == 'rj5':
            print "Setting " + name
            self.commandedR[5] = scale.get_value()
        elif name == 'rj6':
            print "Setting " + name
            self.commandedR[6] = scale.get_value()
        elif name == 'lj0':
            print "Setting " + name
            self.commandedL[0] = scale.get_value()
        elif name == 'lj1':
            print "Setting " + name
            self.commandedL[1] = scale.get_value()
        elif name == 'lj2':
            print "Setting " + name
            self.commandedL[2] = scale.get_value()
        elif name == 'lj3':
            print "Setting " + name
            self.commandedL[3] = scale.get_value()
        elif name == 'lj4':
            print "Setting " + name
            self.commandedL[4] = scale.get_value()
        elif name == 'lj5':
            print "Setting " + name
            self.commandedL[5] = scale.get_value()
        elif name == 'lj6':
            print "Setting " + name
            self.commandedL[6] = scale.get_value()



    def button_clicked(self,button):
        print("Button has been clicked!")
        # Send and event using a method that directly accepts data.
        messageR = JointAngles()
        messageR.angles.extend(self.commandedR)

        messageL = JointAngles()
        messageL.angles.extend(self.commandedL)

        self.informerR.publishData(messageR)
        self.informerL.publishData(messageL)

        print "Message sent: " + str(messageR)
        print "Message sent: " + str(messageL)

    def __init__(self):
        Gtk.Window.__init__(self, title="Robot GUI Quick Hack")
        self.set_border_width(3)

        self.informerR = rsb.createInformer("/right_arm/frequency", dataType=JointAngles)
        self.informerL = rsb.createInformer("/left_arm/frequency", dataType=JointAngles)

        # TODO get home / safe posture
        self.commandedR = [0] * 7;
        self.commandedL = [0] * 7;

        pane = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        pane.set_border_width(10)

        self.notebook = Gtk.Notebook()
        pane.add(self.notebook)

        button = Gtk.Button(label="Apply")
        button.connect("clicked", self.button_clicked)
        pane.add(button)

        self.add(pane)

        robot = {}

        robot['Right'] = {}
        robot['Right']['rj0'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj1'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj2'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj3'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj4'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj5'] = JointInfo(0, 0, -3, 3)
        robot['Right']['rj6'] = JointInfo(0, 0, -3, 3)

        robot['Left'] = {}
        robot['Left']['lj0'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj1'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj2'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj3'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj4'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj5'] = JointInfo(0, 0, -3, 3)
        robot['Left']['lj6'] = JointInfo(0, 0, -3, 3)

        for chain, joints in robot.iteritems():
            page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            page.set_border_width(10)
            for name, info in joints.iteritems():
                hbox = Gtk.Box(spacing=6)
                label = Gtk.Label(name)
                label.set_size_request(100, 30)
                hbox.add(label)

                # two adjustments (initial value, min value, max value,
                # step increment - press cursor keys to see!,
                # page increment - click around the handle to see!,
                # page size - not used here)
                ad1 = Gtk.Adjustment(info.initial, info.min, info.max, 0.1, 1, 0)

                # an horizontal scale
                h_scale = Gtk.Scale(
                    orientation=Gtk.Orientation.HORIZONTAL, adjustment=ad1)
                # of integers (no digits)
                h_scale.set_digits(1)
                # that can expand horizontally if there is space in the grid (see
                # below)
                h_scale.set_hexpand(True)
                # that is aligned at the top of the space allowed in the grid (see
                # below)
                h_scale.set_valign(Gtk.Align.START)
                h_scale.set_size_request(280, 30)
                # hbox.pack_start(h_scale, False, False, 0)
                hbox.add(h_scale)
                h_scale.connect("value-changed", self.scale_moved, h_scale, name)

                # adjustment = Gtk.Adjustment(0, 0, 100, 1, 10, 0)
                # spinbutton = Gtk.SpinButton()
                # spinbutton.set_adjustment(adjustment)
                # hbox.pack_start(spinbutton, False, False, 0)

                # check_numeric = Gtk.CheckButton("Numeric")
                # check_numeric.connect("toggled", self.on_numeric_toggled)
                # hbox.pack_start(check_numeric, False, False, 0)

                # check_ifvalid = Gtk.CheckButton("If Valid")
                # #check_ifvalid.connect("toggled", self.on_ifvalid_toggled)
                # hbox.pack_start(check_ifvalid, False, False, 0)

                page.add(hbox)

            self.notebook.append_page(page, Gtk.Label(chain))


        # self.h_scale.connect("value-changed", self.scale_moved)
        # self.page2.add(self.h_scale)
        # self.page2.set_border_width(10)
        # self.page2.add(Gtk.Label('A page with an image for a Title.'))
        # self.notebook.append_page(
        #     self.page2,
        #     Gtk.Image.new_from_icon_name(
        #         "help-about",
        #         Gtk.IconSize.MENU
        #     )
        #)

    def getinformerR(self):
        return self.informerR
    
    def getinformerL(self):
        return self.informerL


def main():
    win = MainWindow()
    win.connect("delete-event", Gtk.main_quit)
    win.show_all()
    Gtk.main()
    win.getinformerR().deactivate()
    win.getinformerL().deactivate()

if __name__ == '__main__':
    main()
