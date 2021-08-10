#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import sys
import moveit_commander
import copy
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from eager_demo.msg import EagerDemoErrorCodes
from eager_demo.srv import DetectObject, DetectObjectRequest


class EagerDemo(object):

    def __init__(self):
        self.event = None

        rospy.init_node('eager_demo', log_level=rospy.INFO)

        self.rate = rospy.Rate(2)

        sim = rospy.get_param('~sim')

        # In case of fake detection, we use a predefined pose
        self.fake_detection = rospy.get_param('~fake_detection', True)

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
        self.checks_per_rad = rospy.get_param('~checks_per_rad', 15)
        self.vel_limit = rospy.get_param('~vel_limit', 0.1)
        self.duration = rospy.get_param('~duration', 0.5)
        self.collision_height = rospy.get_param('~collision_height', 0.08)
        self.base_length = rospy.get_param('~base_length', 0.4)
        self.workspace_length = rospy.get_param('~workspace_length', 2.4)

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

        # Grasp parameters
        self.pre_grasp_height = rospy.get_param('~pre_grasp_height', 0.2)
        self.grasp_height = rospy.get_param('~grasp_height', 0.11)
        detect_topic = rospy.get_param('~pose_topic', 'gem_l515/detect')
        self.object_name = rospy.get_param('~object_name', 'can')

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        self.manipulator_group = moveit_commander.MoveGroupCommander(self.manipulator_group_name)
        self.end_effector_group = moveit_commander.MoveGroupCommander(self.end_effector_group_name)

        if not sim:
            self.manipulator_group.set_max_velocity_scaling_factor(0.2)
            self.manipulator_group.set_max_acceleration_scaling_factor(0.2)

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

        # Subscribers
        rospy.Subscriber('~event_in', String, self._event_in_callback)

        # Services
        if not self.fake_detection:
            self._get_object_pose_service = rospy.ServiceProxy(detect_topic, DetectObject)

        # Initialize tf
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)

    def _event_in_callback(self, msg):
        if self.event is None:
            self.event = msg.data

    def _move_to_joint_goal(self, move_group, joint_goal):
        # Now, we call the planner to compute the plan and execute it.
        succes = move_group.go(joint_goal, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        return succes

    def _move_to_pose_goal(self, move_group, pose_goal):
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
                                                                velocity_scaling_factor=0.2,
                                                                acceleration_scaling_factor=0.2,
                                                                )
        move_group.execute(retimed_plan, wait=True)

    def _transform_pose(self, pose_stamped, to_frame):
        original_frame = pose_stamped.header.frame_id
        try:
            transform = self.tfBuffer.lookup_transform(to_frame, original_frame, time=rospy.Time.now())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('[{}] Cannot transform pose, failed to lookup transform from {} to {}!'.format(
                rospy.get_name(), original_frame, to_frame)
                )
            return EagerDemoErrorCodes.TRANSFORM_POSE_FAILED
        rospy.logwarn(transform)
        return tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

    def _get_grasp_poses(self, object_pose):
        pre_grasp_pose = Pose()
        pre_grasp_pose.orientation.y = 0.7071068
        pre_grasp_pose.orientation.w = 0.7071068
        pre_grasp_pose.position.x = object_pose.pose.position.x
        pre_grasp_pose.position.y = object_pose.pose.position.y
        pre_grasp_pose.position.z = self.pre_grasp_height

        grasp_pose = copy.deepcopy(pre_grasp_pose)
        grasp_pose.position.z = self.grasp_height
        return pre_grasp_pose, grasp_pose

    def _command_home(self):
        target = self.manipulator_group.get_named_target_values('Home')
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_upright(self):
        target = self.manipulator_group.get_named_target_values('Upright')
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_sleep(self):
        target = self.manipulator_group.get_named_target_values('Sleep')
        self._move_to_joint_goal(self.manipulator_group, target)

    def _command_open(self):
        target = self.end_effector_group.get_named_target_values('Open')
        self._move_to_joint_goal(self.end_effector_group, target)

    def _command_close(self):
        target = self.end_effector_group.get_named_target_values('Closed')
        self._move_to_joint_goal(self.end_effector_group, target)

    def _grasp(self):
        # Request pose from object detector
        req = DetectObjectRequest(self.object_name)
        if not self.fake_detection:
            pose = self._get_object_pose_service(req)
        else:
            pose = PoseStamped()
            pose.header.frame_id = self.base_frame
            pose.pose.position.x = 0.4
            pose.pose.position.y = 0.105
            pose.pose.position.z = 0.4
            pose.pose.orientation.w = 1.0
            pose = self._transform_pose(pose, 'camera_color_optical_frame')
        # Transform object pose to robot base frame
        transformed_pose = self._transform_pose(pose, self.base_frame)

        # Get pre grasp pose
        pre_grasp_pose, grasp_pose = self._get_grasp_poses(transformed_pose)
        # Move to pregrasp pose
        self._move_to_pose_goal(self.manipulator_group, pre_grasp_pose)
        # Open gripper
        self._command_open()
        # Move down
        waypoints = [pre_grasp_pose, grasp_pose]
        self._move_along_cartesian_path(self.manipulator_group, waypoints)
        # TODO: Close gripper

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
                elif self.event == 'grasp':
                    self._grasp()
                self.event = None
            self.rate.sleep()


def main():
    try:
        robot = EagerDemo()
        robot.command()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
