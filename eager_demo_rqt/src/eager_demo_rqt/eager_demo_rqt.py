import os
import rospy
import rospkg
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String
from interbotix_xs_sdk.srv import TorqueEnable, TorqueEnableRequest


class EagerDemoRqt(Plugin):

    def __init__(self, context):
        super(EagerDemoRqt, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('EagerDemoRqt')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('eager_demo_rqt'), 'resource', 'eager_demo.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('EagerDemoRqtUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.event_publisher = rospy.Publisher("eager_demo/event_in", String, queue_size=1, latch=True)
        self.torque_service = rospy.ServiceProxy('torque_enable', TorqueEnable)

        # Calibration Buttons
        self._widget.pose1Button.clicked[bool].connect(self.handle_pose1)
        self._widget.pose2Button.clicked[bool].connect(self.handle_pose2)
        self._widget.pose3Button.clicked[bool].connect(self.handle_pose3)
        self._widget.pose4Button.clicked[bool].connect(self.handle_pose4)
        self._widget.pose5Button.clicked[bool].connect(self.handle_pose5)
        self._widget.pose6Button.clicked[bool].connect(self.handle_pose6)
        self._widget.pose7Button.clicked[bool].connect(self.handle_pose7)
        self._widget.pose8Button.clicked[bool].connect(self.handle_pose8)
        self._widget.pose9Button.clicked[bool].connect(self.handle_pose9)
        self._widget.pose10Button.clicked[bool].connect(self.handle_pose10)
        self._widget.pose11Button.clicked[bool].connect(self.handle_pose11)
        self._widget.pose12Button.clicked[bool].connect(self.handle_pose12)

        # Manipulator Buttons
        self._widget.homeButton.clicked[bool].connect(self.handle_home)
        self._widget.uprightButton.clicked[bool].connect(self.handle_upright)
        self._widget.sleepButton.clicked[bool].connect(self.handle_sleep)

        # Gripper Buttons
        self._widget.openButton.clicked[bool].connect(self.handle_open)
        self._widget.closeButton.clicked[bool].connect(self.handle_close)

        # Torque Buttons
        self._widget.enableButton.clicked[bool].connect(self.handle_enable)
        self._widget.disableButton.clicked[bool].connect(self.handle_disable)

        # Demo Button
        self._widget.graspButton.clicked[bool].connect(self.handle_grasp)

    def shutdown_plugin(self):
        self.event_publisher.unregister()
        self.torque_service.close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def handle_pose1(self):
        msg = String('calibration_pose_1')
        self.event_publisher.publish(msg)

    def handle_pose2(self):
        msg = String('calibration_pose_2')
        self.event_publisher.publish(msg)

    def handle_pose3(self):
        msg = String('calibration_pose_3')
        self.event_publisher.publish(msg)

    def handle_pose4(self):
        msg = String('calibration_pose_4')
        self.event_publisher.publish(msg)

    def handle_pose5(self):
        msg = String('calibration_pose_5')
        self.event_publisher.publish(msg)

    def handle_pose6(self):
        msg = String('calibration_pose_6')
        self.event_publisher.publish(msg)

    def handle_pose7(self):
        msg = String('calibration_pose_7')
        self.event_publisher.publish(msg)

    def handle_pose8(self):
        msg = String('calibration_pose_8')
        self.event_publisher.publish(msg)

    def handle_pose9(self):
        msg = String('calibration_pose_9')
        self.event_publisher.publish(msg)

    def handle_pose10(self):
        msg = String('calibration_pose_10')
        self.event_publisher.publish(msg)

    def handle_pose11(self):
        msg = String('calibration_pose_11')
        self.event_publisher.publish(msg)

    def handle_pose12(self):
        msg = String('calibration_pose_12')
        self.event_publisher.publish(msg)

    def handle_home(self):
        msg = String('home')
        self.event_publisher.publish(msg)

    def handle_upright(self):
        msg = String('upright')
        self.event_publisher.publish(msg)

    def handle_sleep(self):
        msg = String('sleep')
        self.event_publisher.publish(msg)

    def handle_open(self):
        msg = String('open')
        self.event_publisher.publish(msg)

    def handle_close(self):
        msg = String('close')
        self.event_publisher.publish(msg)

    def handle_grasp(self):
        msg = String('grasp')
        self.event_publisher.publish(msg)

    def handle_enable(self):
        req = TorqueEnableRequest()
        req.cmd_type = 'group'
        req.name = 'arm'
        req.enable = True
        self.torque_service.call(req)

    def handle_disable(self):
        req = TorqueEnableRequest()
        req.cmd_type = 'group'
        req.name = 'arm'
        req.enable = False
        self.torque_service.call(req)
