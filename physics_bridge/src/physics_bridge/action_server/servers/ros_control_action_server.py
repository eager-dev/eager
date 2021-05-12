#!/usr/bin/env python3
import rospy, rosparam
from action_server.action_server import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np


class JointTrajectoryActionServer(ActionServer):
    def __init__(self, name, actuator, actuator_params):
        control_params = rosparam.load_file(actuator_params["control_config"])[0][0]
        controller_params = control_params[actuator_params["controller"]]
        controller_type = controller_params["type"]
        action_server = actuator_params["action_server"]
        if "joints" in controller_params:
            self.joint_names = controller_params["joints"]
        if controller_type.strip().lower().endswith("trajectorycontroller"):
            action_type = JointTrajectory
        else:
            rospy.logerror("Currently only controllers of type \"TrajectoryController\" are supported.")
        super(RosControlActionServer, self).__init__(actuator_topic, action_type)
        
        
    def convert_action_to_joint_trajectory(self, action_raw):
       # Code copied from robo-gym
       action = JointTrajectory()
       action.header = Header()
       action.joint_names = self.joint_names
       action.points=[JointTrajectoryPoint()]
       action.points[0].positions = action_raw
       action.points[0].time_from_start = rospy.Duration(0.005)
       return action
    
    def _convert_action(self, action_raw):
        action = self.convert_action_to_joint_trajectory(np.asarray(action_raw))
        return action