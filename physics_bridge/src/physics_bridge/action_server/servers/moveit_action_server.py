#!/usr/bin/env python3
import rospy, rosparam
from action_server.action_server import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np


class MoveitActionServer(ActionServer):
    def __init__(self, topic, name, actuator, actuator_params):
        joint_states_topic = actuator_params["joint_states_topic"]
        moveit_params = rosparam.load_file(actuator_params["moveit_config"])[0][0]
        controller_params = moveit_params[actuator_params["controller"]]
        controller_type = controller_params["type"]
        
        
        self.joint_names = controller_params["joints"]
        self.vel_limits = actuator_params["vel_limits"]
        if controller_type.strip().lower().endswith("trajectorycontroller"):
            action_type = JointTrajectory
            self.min_traj_duration = actuator_params["min_traj_duration"]
        else:
            rospy.logerror("Currently only controllers of type \"TrajectoryController\" are supported.")
        rospy.Subscriber(joint_states_topic, JointState, self.joint_states_callback)
        self.joint_state = rospy.wait_for_message(joint_states_topic, JointState)
        super(MoveitActionServer, self).__init__(name, actuator, actuator_params, action_type)
        
    def joint_states_callback(self, data):
        self.joint_state = data
        
    def convert_action_to_joint_trajectory(self, action_raw):
       # Code copied from robo-gym
       action = JointTrajectory()
       action.header = Header()
       action.joint_names = self.joint_names
       action.points=[JointTrajectoryPoint()]
       action.points[0].positions = action_raw
       duration = []
       pos = self.joint_state.position[0:len(self.joint_names)]
       for i in range(len(action.joint_names)):
           cmd = action_raw[i]
           max_vel = self.vel_limits[i]
           duration.append(max(abs(cmd-pos[i])/max_vel, self.min_traj_duration))
   
       action.points[0].time_from_start = rospy.Duration.from_sec(max(duration))
       return action
    
    def _convert_action(self, action_raw):
        action = self.convert_action_to_joint_trajectory(np.asarray(action_raw))
        return action
    
