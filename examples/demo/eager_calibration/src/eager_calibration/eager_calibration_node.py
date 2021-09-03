#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from eager_calibration.srv import SetFloat32, SetFloat32Response
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
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
        self.collision_height = rospy.get_param('~collision_height', 0.08)
        self.base_length = rospy.get_param('~base_length', 0.4)
        self.workspace_length = rospy.get_param('~workspace_length', 2.4)
        self.velocity_scaling_factor = rospy.get_param('~velocity_scaling_factor', 0.2)
        self.acceleration_scaling_factor = rospy.get_param('~acceleration_scaling_factor', 0.2)

        # Calibration poses
        self.calibration_pose_1 = rospy.get_param('~calibration_pose_1',
                                                  [0.26720266, 0.41769082, -0.61405873, -0.10920159, 1.02477717, 1.33158357]
                                                  )
        self.calibration_pose_2 = rospy.get_param('~calibration_pose_2',
                                                  [-0.1697083, 0.42902144, -0.17906268, -0.48181534, 0.1369806, -0.42154863]
                                                  )
        self.calibration_pose_3 = rospy.get_param('~calibration_pose_3',
                                                  [0.26720266, 0.41769082, -0.61405873, -0.10920159, 1.02477717, 1.33158357]
                                                  )
        self.calibration_pose_4 = rospy.get_param('~calibration_pose_4',
                                                  [-0.20652833, 0.88519322, -1.03238164, 1.19097259, 1.34757152, -1.96848806]
                                                  )
        self.calibration_pose_5 = rospy.get_param('~calibration_pose_5',
                                                  [-0.240835, 0.12732041, 0.05675729, 0.35895151, 0.02372742, -0.02372742]
                                                  )
        self.calibration_pose_6 = rospy.get_param('~calibration_pose_6',
                                                  [-0.31433059, 0.37894629, -0.40868386, -2.55617312, -0.95543624, 2.55777874]
                                                  )
        self.calibration_pose_7 = rospy.get_param('~calibration_pose_7',
                                                  [-0.19125552, 0.20515017, 0.11557607, 1.92872024, 0.08407946, -1.58544498]
                                                  )
        self.calibration_pose_8 = rospy.get_param('~calibration_pose_8',
                                                  [-0.44587893, 0.4809407, -0.12195346, 1.7612175, 1.143879, -1.5109534]
                                                  )
        self.calibration_pose_9 = rospy.get_param('~calibration_pose_9',
                                                  [0.07467588, 0.79585576, -1.2105316, 3.10480717, -1.20383058, -2.50254506]
                                                  )
        self.calibration_pose_10 = rospy.get_param('~calibration_pose_10',
                                                   [0.0909588, 0.77143045, -1.0667212, 2.82870459, -0.84640253, -2.95916986]
                                                   )
        self.calibration_pose_11 = rospy.get_param('~calibration_pose_11',
                                                   [-0.21074113, 0.80485942, -1.21160632, -3.1017593, -0.92140806, 2.94429842]
                                                   )
        self.calibration_pose_12 = rospy.get_param('~calibration_pose_12',
                                                   [0.27810045, 0.21982409, -0.27928612, 2.86218982, -0.8442566, -3.12706443]
                                                   )

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
        self.state_validity_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)

        rospy.Service('goal', SetFloat32, self._goal_callback)
        rospy.Service('check', SetFloat32, self._check_callback)

    def _event_in_callback(self, msg):
        if self.event is None:
            self.event = msg.data

    def _goal_callback(self, req):
        response = SetFloat32Response()
        rospy.logwarn('3 {}'.format(req.data))
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
        response.success = self.state_validity_service.call(gsvr).valid
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
        rospy.sleep(2)
        self.action_server.act([])

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
        rospy.sleep(2)
        self.action_server.act([])

    def command(self):
        while not rospy.is_shutdown():
            if self.event is not None:
                if self.event == 'calibration_pose_1':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_1)
                elif self.event == 'calibration_pose_2':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_2)
                elif self.event == 'calibration_pose_3':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_3)
                elif self.event == 'calibration_pose_4':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_4)
                elif self.event == 'calibration_pose_5':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_5)
                elif self.event == 'calibration_pose_6':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_6)
                elif self.event == 'calibration_pose_7':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_7)
                elif self.event == 'calibration_pose_8':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_8)
                elif self.event == 'calibration_pose_9':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_9)
                elif self.event == 'calibration_pose_10':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_10)
                elif self.event == 'calibration_pose_11':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_11)
                elif self.event == 'calibration_pose_12':
                    self._move_to_joint_goal(self.manipulator_group, self.calibration_pose_12)
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
