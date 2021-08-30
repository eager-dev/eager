import os
import rospy
import rospkg
import roslaunch
import yaml
from rospy import ROSException
from eager_core.utils.file_utils import substitute_xml_args
from std_srvs.srv import Empty, EmptyRequest
from easy_handeye_msgs.srv import ComputeCalibration, ComputeCalibrationRequest
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from std_msgs.msg import String
from interbotix_xs_sdk.srv import TorqueEnable, TorqueEnableRequest


class EagerCalibrationRqt(Plugin):

    def __init__(self, context):
        super(EagerCalibrationRqt, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('EagerCalibrationRqt')
        # States
        self.calibrate_launch = None
        self.publish_launch = None

        # Get parameters
        self.namespace_prefix = rospy.get_param('~namespace_prefix', 'hand_eye_calibration')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'vx300s/base_link')
        self.image_is_rectified = rospy.get_param('~image_is_rectified', True)
        self.marker_size = rospy.get_param('~marker_size', 0.04)
        self.marker_id = rospy.get_param('~marker_id', 26)
        self.reference_frame = rospy.get_param('~reference_frame', 'camera_bottom_screw_frame')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        self.marker_frame = rospy.get_param('~marker_frame', 'camera_marker')
        self.start_rviz = rospy.get_param('~start_rviz', False)
        self.eye_on_hand = rospy.get_param('eye_on_hand', False)
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'vx300s/base_link')
        self.robot_effector_frame = rospy.get_param('~robot_effector_frame', 'vx300s/ee_arm_link')
        self.tracking_base_frame = rospy.get_param('~tracking_base_frame', self.reference_frame)
        self.tracking_marker_frame = rospy.get_param('~tracking_marker_frame', self.marker_frame)
        self.freehand_robot_movement = rospy.get_param('~freehand_robot_movement', True)
        self.robot_velocity_scaling = rospy.get_param('~robot_velocity_scaling', 0.2)
        self.robot_acceleration_scaling = rospy.get_param('robot_acceleration_scaling', 0.2)

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
        ui_file = os.path.join(rospkg.RosPack().get_path('eager_calibration_rqt'), 'resource', 'eager_calibration.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('EagerCalibrationRqtUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.event_publisher = rospy.Publisher("eager_calibration/event_in", String, queue_size=1, latch=True)
        self.torque_service = rospy.ServiceProxy('torque_enable', TorqueEnable)

        # Calibration Buttons
        self._widget.calibrateButton.clicked[bool].connect(self.handle_calibrate)
        self._widget.publishButton.clicked[bool].connect(self.handle_publish)
        self._widget.saveButton.clicked[bool].connect(self.handle_save)
        self._widget.pose1Button.clicked[bool].connect(self.handle_pose1)
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

        # Calibration Services
        self._get_calibration_service = rospy.ServiceProxy(
            '{}_eye_on_base/compute_calibration'.format(self.namespace_prefix), ComputeCalibration)
        self._save_calibration_service = rospy.ServiceProxy(
            '{}_eye_on_base/save_calibration'.format(self.namespace_prefix), Empty)

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

    def handle_calibrate(self):
        if self.calibrate_launch:
            rospy.logwarn('[{}] Already calibrating! Ignoring input.'.format(rospy.get_name()))
        else:
            if self.publish_launch:
                self.publish_launch.shutdown()
                self.publish_launch = None
            str_launch = '$(find eager_calibration)/launch/calibrate.launch'
            cli_args = [substitute_xml_args(str_launch),
                        "namespace_prefix:={}".format(self.namespace_prefix),
                        "base_link_frame:={}".format(self.base_link_frame),
                        "image_is_rectified:={}".format(self.image_is_rectified),
                        "marker_size:={}".format(self.marker_size),
                        "marker_id:={}".format(self.marker_id),
                        "reference_frame:={}".format(self.reference_frame),
                        "camera_frame:={}".format(self.camera_frame),
                        "marker_frame:={}".format(self.marker_frame),
                        "start_rviz:={}".format(self.start_rviz),
                        "eye_on_hand:={}".format(self.eye_on_hand),
                        "robot_base_frame:={}".format(self.robot_base_frame),
                        "robot_effector_frame:={}".format(self.robot_effector_frame),
                        "tracking_base_frame:={}".format(self.tracking_base_frame),
                        "tracking_marker_frame:={}".format(self.tracking_marker_frame),
                        "freehand_robot_movement:={}".format(self.freehand_robot_movement),
                        "robot_velocity_scaling:={}".format(self.robot_velocity_scaling),
                        "robot_acceleration_scaling:={}".format(self.robot_acceleration_scaling),
                        ]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.calibrate_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.calibrate_launch.start()

    def handle_publish(self):
        if self.publish_launch:
            rospy.logwarn('[{}] Already publishing! Ignoring input.'.format(rospy.get_name()))
        else:
            if self.calibrate_launch:
                self.calibrate_launch.shutdown()
                self.calibrate_launch = None
            str_launch = '$(find eager_calibration)/launch/publish.launch'
            cli_args = [substitute_xml_args(str_launch),
                        "namespace_prefix:={}".format(self.namespace_prefix),
                        ]
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self.publish_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self.publish_launch.start()

    def handle_save(self):
        if self.calibrate_launch is None:
            rospy.logwarn('[{}] Push calibrate first!.'.format(rospy.get_name()))
        else:
            try:
                self._get_calibration_service.wait_for_service(timeout=3)
            except ROSException:
                rospy.logwarn('[{}] Calculate calibration service not available!'.format(rospy.get_name()))
                return
            req = ComputeCalibrationRequest()
            try:
                response = self._get_calibration_service(req)
            except rospy.ServiceException:
                rospy.logwarn('[{}] Get calibration failed, did you make enough samples?'.format(rospy.get_name()))
                return
            try:
                self._save_calibration_service()
            except rospy.ServiceException as e:
                rospy.logwarn('[{}] Save calibration failed: {}'.format(rospy.get_name(), e))
            translation = response.calibration.transform.transform.translation
            rotation = response.calibration.transform.transform.rotation
            calibration = {}
            calibration['position'] = [translation.x, translation.y, translation.z]
            calibration['rotation'] = [rotation.x, rotation.y, rotation.z, rotation.w]
            save_path = substitute_xml_args('$(find eager_demo)/config/calibration.yaml')
            with open(save_path, 'w') as file:
                yaml.dump(calibration, file)

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
