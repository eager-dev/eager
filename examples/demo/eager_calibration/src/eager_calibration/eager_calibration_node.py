#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import numpy as np
import yaml
from eager_core.utils.file_utils import substitute_xml_args
from easy_handeye_msgs.srv import ComputeCalibration, ComputeCalibrationRequest, TakeSample
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from eager_calibration.srv import SetFloat32, SetFloat32Response
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
from eager_core.action_server import FollowJointTrajectoryActionServer


class EagerCalibration(object):

    def __init__(self):
        self.event = None

        rospy.init_node('eager_calibration', log_level=rospy.INFO)

        self.rate = rospy.Rate(5)

        robot_sim = rospy.get_param('~robot_sim')

        # MoveIt parameters
        self.current_position = None
        self.base_frame = rospy.get_param('~base_frame', 'vx300s/base_link')
        self.joint_names = rospy.get_param('~joint_names',
                                           ['waist', 'shoulder', 'elbow',
                                            'forearm_roll', 'wrist_angle',
                                            'wrist_rotate',
                                            ]
                                           )
        self.manipulator_group_name = rospy.get_param('~manipulator_group_name', 'interbotix_arm')
        self.end_effector_group_name = rospy.get_param('~end_effector_group_name', 'interbotix_gripper')

        self.ee_step = rospy.get_param('~ee_step', 0.01)
        self.jump_threshold = rospy.get_param('~jump_threshold', 0.0)
        self.collision_height = rospy.get_param('~collision_height', 0.2)
        self.base_length = rospy.get_param('~base_length', 0.4)
        self.workspace_length = rospy.get_param('~workspace_length', 2.4)
        self.velocity_scaling_factor = rospy.get_param('~velocity_scaling_factor', 0.75)
        self.acceleration_scaling_factor = rospy.get_param('~acceleration_scaling_factor', 0.75)

        # Calibration prefix
        self.namespace_prefix = rospy.get_param('~namespace_prefix', 'hand_eye_calibration')

        # Calibration poses
        calibration_pose_1 = rospy.get_param(
            '~calibration_pose_1',
            [-0.0076699042692780495, 0.15800002217292786, -0.42337870597839355,
            -0.0015339808305725455, 0.8390874862670898, -0.003067961661145091]
            )
        calibration_pose_2 = rospy.get_param(
            '~calibration_pose_2',
            [0.19634954631328583, 0.26077672839164734, -0.526155412197113,
            -0.8053399324417114, 1.084524393081665, 0.5890486240386963]
            )
        calibration_pose_3 = rospy.get_param(
            '~calibration_pose_3',
            [0.28378644585609436, 0.43104860186576843, -0.7179030179977417,
            -1.118272066116333, 1.366776943206787, 0.7133010625839233]
            )
        calibration_pose_4 = rospy.get_param(
            '~calibration_pose_4',
            [0.2822524607181549, 0.46019425988197327, -0.7071651816368103,
            -1.1167380809783936, 1.366776943206787, 0.5767768025398254]
            )
        calibration_pose_5 = rospy.get_param(
            '~calibration_pose_5',
            [0.3129321038722992, 0.849825382232666, -1.2026410102844238,
            -1.5569905042648315, 1.6889128684997559, 0.5706408619880676]
            )
        calibration_pose_6 = rospy.get_param(
            '~calibration_pose_6',
            [0.3129321038722992, 0.8636311888694763, -1.2072429656982422,
            -1.558524489402771, 1.6889128684997559, 0.9710098505020142]
            )
        calibration_pose_7 = rospy.get_param(
            '~calibration_pose_7',
            [0.31446605920791626, 0.5154175758361816, -0.6043884754180908,
            -1.4419419765472412, 1.3959225416183472, 1.2624661922454834]
            )
        calibration_pose_8 = rospy.get_param(
            '~calibration_pose_8',
            [-0.026077674701809883, 0.1733398288488388, -0.24543693661689758,
            -0.8559613227844238, 0.7056311964988708, 0.6043884754180908]
            )
        calibration_pose_9 = rospy.get_param(
            '~calibration_pose_9',
            [-0.03221359848976135, 0.1733398288488388, -0.0951068103313446,
            -1.1428157091140747, 0.5276893973350525, 0.8682331442832947]
            )
        calibration_pose_10 = rospy.get_param(
            '~calibration_pose_10',
            [0.06749515980482101, 0.2715145945549011, -0.17487381398677826,
            -1.4695535898208618, 0.9157865643501282, 1.1566215753555298]
            )
        calibration_pose_11 = rospy.get_param(
            '~calibration_pose_11',
            [0.06749515980482101, 0.28071850538253784, -0.1764077991247177,
            -1.4695535898208618, 0.9157865643501282, 1.402058482170105]
            )
        calibration_pose_12 = rospy.get_param(
            '~calibration_pose_12',
            [-0.07056311517953873, -0.0015339808305725455, -0.1764077991247177,
            -0.2346990704536438, 0.8022719621658325, -0.21322333812713623]
            )
        calibration_pose_13 = rospy.get_param(
            '~calibration_pose_12',
            [-0.16260196268558502, 0.5414952039718628, -0.951068103313446,
            0.12118448317050934, 1.1274758577346802, -0.653475821018219]
            )
        calibration_pose_14 = rospy.get_param(
            '~calibration_pose_12',
            [-0.23930101096630096, 0.36201947927474976, -0.49087387323379517,
            0.607456386089325, 0.6657477021217346, -1.0737866163253784]
            )
        calibration_pose_15 = rospy.get_param(
            '~calibration_pose_12',
            [-0.31446605920791626, 0.552233099937439, -0.8881748914718628,
            0.7623884677886963, 1.1397477388381958, -1.1075341701507568]
            )
        calibration_pose_16 = rospy.get_param(
            '~calibration_pose_12',
            [-0.2791845202445984, 0.5721748471260071, -0.9710098505020142,
            0.6013205051422119, 1.1842331886291504, -0.7470486760139465]
            )
        self.calibration_poses = np.asarray([calibration_pose_1,
                                             calibration_pose_2,
                                             calibration_pose_3,
                                             calibration_pose_4,
                                             calibration_pose_5,
                                             calibration_pose_6,
                                             calibration_pose_7,
                                             calibration_pose_8,
                                             calibration_pose_9,
                                             calibration_pose_10,
                                             calibration_pose_11,
                                             calibration_pose_12,
                                             calibration_pose_13,
                                             calibration_pose_14,
                                             calibration_pose_15,
                                             calibration_pose_16,
                                             ])

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # Initialize robot state
        self.robot_state = RobotState()
        self.robot_state.joint_state.name = self.joint_names

        self.manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        self.end_effector_group = moveit_commander.MoveGroupCommander(self.end_effector_group_name)

        if not robot_sim:
            self.manipulator_group.set_max_velocity_scaling_factor(self.velocity_scaling_factor)
            self.manipulator_group.set_max_acceleration_scaling_factor(self.acceleration_scaling_factor)

        # Five collision objects are added to the scene, such that the base is
        # not in collision, while the rest of the surface is a collision object.

        object_length = (self.workspace_length - self.base_length) / 2.0

        # Add a collision object to the scenes
        p0 = PoseStamped()
        p0.header.frame_id = self.base_frame
        p0.pose.position.z = - 0.051
        p0.pose.orientation.w = 1

        # Add a collision object to the scenes
        p1 = PoseStamped()
        p1.header.frame_id = self.base_frame
        p1.pose.position.x = (object_length + self.base_length) / 2.0
        p1.pose.position.z = self.collision_height / 2.0
        p1.pose.orientation.w = 1

        # Add a collision object to the scenes
        p2 = PoseStamped()
        p2.header.frame_id = self.base_frame
        p2.pose.position.x = - (object_length + self.base_length) / 2.0
        p2.pose.position.z = self.collision_height / 2.0
        p2.pose.orientation.w = 1

        # Add a collision object to the scenes
        p3 = PoseStamped()
        p3.header.frame_id = self.base_frame
        p3.pose.position.y = (object_length + self.base_length) / 2.0
        p3.pose.position.z = self.collision_height / 2.0
        p3.pose.orientation.w = 1

        # Add a collision object to the scenes
        p4 = PoseStamped()
        p4.header.frame_id = self.base_frame
        p4.pose.position.y = - (object_length + self.base_length) / 2.0
        p4.pose.position.z = self.collision_height / 2.0
        p4.pose.orientation.w = 1

        scene.add_box('base0', p0, size=(self.workspace_length, self.workspace_length, 0.1))
        scene.add_box('base1', p1, size=(object_length, self.base_length, self.collision_height))
        scene.add_box('base2', p2, size=(object_length, self.base_length, self.collision_height))
        scene.add_box('base3', p3, size=(self.workspace_length, object_length, self.collision_height))
        scene.add_box('base4', p4, size=(self.workspace_length, object_length, self.collision_height))

        self.action_server = FollowJointTrajectoryActionServer(
            self.joint_names, 'arm_controller/follow_joint_trajectory', duration=2.0)

        # Subscribers
        rospy.Subscriber('~event_in', String, self._event_in_callback)

        # Services
        self._state_validity_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)
        self._take_sample_service = rospy.ServiceProxy('{}_eye_on_base/take_sample'.format(self.namespace_prefix), TakeSample)
        self._get_calibration_service = rospy.ServiceProxy(
            '{}_eye_on_base/compute_calibration'.format(self.namespace_prefix), ComputeCalibration)
        self._save_calibration_service = rospy.ServiceProxy(
            '{}_eye_on_base/save_calibration'.format(self.namespace_prefix), Empty)

        rospy.Service('goal', SetFloat32, self._goal_callback)
        rospy.Service('check', SetFloat32, self._check_callback)

    def _event_in_callback(self, msg):
        if self.event is None:
            self.event = msg.data

    def _goal_callback(self, req):
        response = SetFloat32Response()
        self.action_server.reset()
        try:
            self._move_to_joint_goal(self.manipulator_group, list(req.data))
        except moveit_commander.exception.MoveItCommanderException:
            response.success = False
            return response
        response.success = True
        return response

    def _check_callback(self, req):
        self.robot_state.joint_state.position = req.data
        response = SetFloat32Response()
        gsvr = GetStateValidityRequest()
        gsvr.group_name = self.manipulator_group_name
        gsvr.robot_state = self.robot_state
        response.success = self._state_validity_service.call(gsvr).valid
        return response

    def _move_to_joint_goal(self, move_group, joint_goal):
        move_group.get_current_state()
        # Now, we call the planner to compute the plan and execute it.
        succes = move_group.go(joint_goal, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        return succes

    def _move_to_pose_goal(self, move_group, pose_goal):
        move_group.get_current_state()
        # Now, we call the planner to compute the plan and execute it.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        move_group.clear_pose_targets()
        return success

    def _move_along_cartesian_path(self, move_group, waypoints):
        plan, _ = move_group.compute_cartesian_path(
            waypoints,
            self.ee_step,
            self.jump_threshold,
            avoid_collisions=False,
        )
        retimed_plan = self.manipulator_group.retime_trajectory(self.manipulator_group.get_current_state(),
                                                                plan,
                                                                velocity_scaling_factor=0.1,
                                                                acceleration_scaling_factor=0.1,
                                                                )
        move_group.execute(retimed_plan, wait=True)

    def _command_home(self):
        target = self.manipulator_group.get_named_target_values('Home')
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_upright(self):
        target = self.manipulator_group.get_named_target_values('Upright')
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_sleep(self):
        self._command_home()
        current_state = self.manipulator_group.get_current_state()
        current_position = current_state.joint_state.position
        target = self.manipulator_group.get_named_target_values('Sleep')
        goal_pos = []
        for idx, joint_name in enumerate(self.joint_names):
            if abs(current_position[idx]) > 0.05:
                rospy.logwarn('[{}] Cannot go to sleep since robot is not in home position!'.format(rospy.get_name()))
                return
            goal_pos.append(target[joint_name])
        self.action_server.act(goal_pos)
        rospy.sleep(3)
        self.action_server.reset()

    def _command_open(self):
        target = self.end_effector_group.get_named_target_values('Open')
        self._move_to_joint_goal(self.end_effector_group, target)

    def _command_close(self):
        target = self.end_effector_group.get_named_target_values('Closed')
        self._move_to_joint_goal(self.end_effector_group, target)

    def _command_up(self):
        current_state = self.manipulator_group.get_current_state()
        current_position = current_state.joint_state.position
        target = self.manipulator_group.get_named_target_values('Sleep')
        goal_pos = []
        for idx, joint_name in enumerate(self.joint_names):
            if abs(target[joint_name] - current_position[idx]) > 0.25:
                rospy.logwarn('[{}] Cannot go up since robot is not in sleep position!'.format(rospy.get_name()))
                return
            goal_pos.append(target[joint_name] / 2)
        self.action_server.act(goal_pos)
        rospy.sleep(3)
        self.action_server.reset()

    def _command_calibrate(self):
        try:
            self._take_sample_service.wait_for_service(timeout=3)
        except rospy.ROSException:
            rospy.logwarn('[{}] Calculate calibration service not available!'.format(rospy.get_name()))
            return
        for pose in self.calibration_poses:
            self._move_to_joint_goal(self.manipulator_group, pose)
            rospy.sleep(1.0)
            try:
                self._take_sample_service()
            except rospy.ServiceException:
                rospy.logwarn('[{}] Take sample failed, is the marker visible for the camera?'.format(rospy.get_name()))
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
        calibration['orientation'] = [rotation.x, rotation.y, rotation.z, rotation.w]
        save_path = substitute_xml_args('$(find eager_demo)/config/calibration.yaml')
        with open(save_path, 'w') as file:
            yaml.dump(calibration, file)

    def command(self):
        while not rospy.is_shutdown():
            if self.event is not None:
                if self.event == 'calibrate':
                    self._command_calibrate()
                elif self.event == 'home':
                    self._command_home()
                elif self.event == 'upright':
                    self._command_upright()
                elif self.event == 'sleep':
                    self._command_sleep()
                elif self.event == 'open':
                    self._command_open()
                elif self.event == 'close':
                    self._command_close()
                elif self.event == 'up':
                    self._command_up()
                self.event = None
            self.rate.sleep()


def main():
    try:
        robot = EagerCalibration()
        robot.command()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
