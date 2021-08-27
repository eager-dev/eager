#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import numpy as np


class MoveForCalibration(object):

    def __init__(self):
        rospy.init_node('robot_commander', log_level=rospy.INFO)

        self.current_position = None
        object_frame = rospy.get_param('~object_frame', 'vx300s/base_link')
        self.joint_names = rospy.get_param('~joint_names',
                                           ['waist', 'shoulder', 'elbow',
                                            'forearm_roll', 'wrist_angle',
                                            'wrist_rotate',
                                            ]
                                           )
        self.group_name = rospy.get_param('~group_name', 'manipulator')
        self.checks_per_rad = rospy.get_param('~checks_per_rad', 15)
        self.vel_limit = rospy.get_param('~vel_limit', 0.1)
        self.duration = rospy.get_param('~duration', 0.5)
        self.collision_height = rospy.get_param('~collision_height', 0.1)
        self.base_length = rospy.get_param('~base_length', 0.4)
        self.workspace_length = rospy.get_param('~workspace_length', 2.4)

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        group_name = "interbotix_arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        # Five collision objects are added to the scene, such that the base is
        # not in collision, while the rest of the surface is a collision object.

        object_length = (self.workspace_length - self.base_length) / 2.0

        # Add a collision object to the scenes
        p0 = PoseStamped()
        p0.header.frame_id = object_frame
        p0.pose.position.z = - 0.06
        p0.pose.orientation.w = 1

        # Add a collision object to the scenes
        p1 = PoseStamped()
        p1.header.frame_id = object_frame
        p1.pose.position.x = (object_length + self.base_length) / 2.0
        p1.pose.position.z = self.collision_height / 2.0
        p1.pose.orientation.w = 1

        # Add a collision object to the scenes
        p2 = PoseStamped()
        p2.header.frame_id = object_frame
        p2.pose.position.x = - (object_length + self.base_length) / 2.0
        p2.pose.position.z = self.collision_height / 2.0
        p2.pose.orientation.w = 1

        # Add a collision object to the scenes
        p3 = PoseStamped()
        p3.header.frame_id = object_frame
        p3.pose.position.y = (object_length + self.base_length) / 2.0
        p3.pose.position.z = self.collision_height / 2.0
        p3.pose.orientation.w = 1

        # Add a collision object to the scenes
        p4 = PoseStamped()
        p4.header.frame_id = object_frame
        p4.pose.position.y = - (object_length + self.base_length) / 2.0
        p4.pose.position.z = self.collision_height / 2.0
        p4.pose.orientation.w = 1

        scene.add_box('base0', p0, size=(self.workspace_length, self.workspace_length, 0.1))
        scene.add_box('base1', p1, size=(object_length, self.base_length, self.collision_height))
        scene.add_box('base2', p2, size=(object_length, self.base_length, self.collision_height))
        scene.add_box('base3', p3, size=(self.workspace_length, object_length, self.collision_height))
        scene.add_box('base4', p4, size=(self.workspace_length, object_length, self.collision_height))

    def move(self, goal_position):
        joint_goal = self.group.get_current_joint_values()
        for i in range(len(goal_position)):
            joint_goal[i] = goal_position[i]

        # Now, we call the planner to compute the plan and execute it.
        self.group.go(goal_position, wait=True)
        self.group.clear_pose_targets()
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()


def main():
    try:
        robot = MoveForCalibration()
        pos = []
        pos.append([-1.2559711821165571, 0.30650458558302507, -0.2062261347323151, 0.04967706396524697, -0.03963437219001233,
                    0.6576527951880893, -0.4737149797290874, 1.5221709929372862, -0.38103466246464013]
                   )
        pos.append([-0.8711696493462195, -0.24561126695850266, -0.19924704372500468, 0.051729508049715045, -0.03758800608308003,
                    0.4430421792435615, 0.36081783592253913, 1.333136058528761, 0.33539437413290685]
                   )
        pos.append([-0.0393428152438382, -0.9367467764503745, -0.20421401472081868, 0.05437872853741439, -0.03493643514107211,
                    0.22726614472911333, 0.2619977857559608, 1.3039440342528792, 0.17318999678930425]
                   )
        pos.append([-1.2410713635765376, -0.7424715210558146, -0.20055376749094744, 0.05373928455073736, -0.03557441674481723,
                    0.5597800869274643, -0.08333464165909366, 1.6416546759502415, -0.12923054388572375]
                   )
        pos.append([-0.5168615941138297, 1.1530144610327548, -0.2198836569183431, 0.04840376492945446, -0.04090969946346537,
                    0.49188439868946077, -0.2886277015500953, 1.2034497923819254, -0.964784017186151]
                   )
        for position in pos:
            position = np.asarray(position)[[6, 5, 0, 1, 7, 8]]
            rospy.logwarn("===== Press `Enter` to move to next pose... ======")
            input()
            robot.move(position)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
