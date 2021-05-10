#!/usr/bin/env python3
import rospy, rosparam
from action_server import ActionServer
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from utils.action_conversion import create_action_converter


class MoveitActionServer(ActionServer):
    def __init__(self, topic, name, actuator):
        joint_states_topic = actuator["joint_states_topic"]
        moveit_params = rosparam.load_file(actuator["moveit_config"])
        controller_params = moveit_params[actuator["controller"]]
        controller_type = controller_params["type"]
        
        self.joint_names = controller_params["joints"]
        self.vel_limits = actuator["vel_limits"]
        if controller_type.strip().lower().endswith("trajectorycontroller"):
            action_type = JointTrajectory
            self.min_traj_duration = actuator["min_traj_duration"]
        else:
            rospy.logerror("Currently only controllers of type \"TrajectoryController\" are supported.")
        self.action_converter = create_action_converter(action_type)
        super(MoveitActionServer, self).__init__(topic, name, actuator, action_type)
        rospy.Subscriber(joint_states_topic, JointState, self.joint_states_callback)
        
    def joint_states_callback(self, data):
        self.joint_state = data
        
        
    def _convert_action(self, action_raw):
        action = self.action_converter(action_raw, 
                                       self.joint_state, 
                                       self.joint_names, 
                                       self.vel_limits, 
                                       self.min_traj_duration)
        return action