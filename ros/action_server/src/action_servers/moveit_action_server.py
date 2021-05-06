#!/usr/bin/env python3
import rospy, rosparam
from action_server import ActionServer
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from utils.action_conversion import create_action_converter


class MoveitActionServer(ActionServer):
    def __init__(self, topic, name, actuator):
        joint_states_topic = actuator["joint_states_topic"]
        params = rosparam.load_file(actuator["config"])
        controller_type = controller_params[actuator["controller"]]
        controller_params = params["controller_type"]
        if controller_type.lower().endswith("trajectorycontroller"):
            action_type = JointTrajectory
            min_traj_duration = actuator["min_traj_duration"]
        self.action_converter = create_action_converter(action_type)
        super(MoveitActionServer, self).__init__(topic, name, actuator, action_type)
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
        
    def joint_states_callback(self, data):
        self.joint_state = data
        
        
    def _convert_action(self, action_raw):
        action = self.action_converter(action_raw)
        return action