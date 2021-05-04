#!/usr/bin/env python3
import rospy
from action_server import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Ur5ActionServer(ActionServer):
    def __init__(self, name, queue_size=10, rate=30):
        action_topic = 'follow_joint_trajectory'
        msg_type = JointTrajectory
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        super(Ur5ActionServer, self).__init__("real", name, "ur5e", action_topic, msg_type, queue_size=queue_size, rate=rate)
        
    def joint_states_callback(self, data):
        self.joint_state = data
        
        
    def _convert_action(self, action_raw):
        # Code copied from robo-gym
        action = JointTrajectory()
        action.header = Header()
        action.joint_names = self.joint_names
        action.points=[JointTrajectoryPoint()]
        action.points[0].positions = action_raw
        duration = []
        for i in range(len(action.joint_names)):
            pos = self.joint_state.position[0:6]
            cmd = action_raw[i]
            max_vel = self.ur_joint_vel_limits[i]
            duration.append(max(abs(cmd-pos)/max_vel,self.min_traj_duration))

        action.points[0].time_from_start = rospy.Duration.from_sec(max(duration))
        return action