import sys
import rospy
import moveit_commander
from ros_gym_core.srv import BoxSpace, BoxSpaceResponse
from ros_gym_core.action_processor import ActionProcessor
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import CubicSpline
import numpy as np


class SafeActions(ActionProcessor):
    def __init__(self):
    		actuator_topic = rospy.get_param('~actuator_topic', 'joints')
    		raw_actuator
        self.joint_names = rospy.get_param('~joint_names',
                                           ['shoulder_pan_joint',
                                            'shoulder_lift_joint',
                                            'elbow_joint',
                                            'wrist_1_joint',
                                            'wrist_2_joint',
                                            'wrist_3_joint'])
        self.group_name = rospy.get_param('~group_name', 'manipulator')
        self.checks_per_rad = rospy.get_param('~checks_per_rad', 25)
        self.vel_limit = rospy.get_param('~vel_limit', 3.0)
        self.step_time = rospy.get_param('~step_time', 0.1)
        self.duration = rospy.get_param('~duration', 0.5)

    		
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface()

        self.get_observation_service = rospy.ServiceProxy(
            robot_topic + sensor_topic, BoxSpace)
        self.get_observation_service.wait_for_service()

        observation = self.get_observation_service()

        # prepare msg to interface with moveit
        self.rs = RobotState()
        self.rs.joint_state.name = self.joint_names
        self.rs.joint_state.position = np.asarray(observation.value)

        self.state_validity_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)
        self.state_validity_service.wait_for_service()
				
				# Add a collision object to the scene
        
        # This is ugly, but seems to be necessary, see: https://answers.ros.org/question/209030/moveit-planningsceneinterface-addbox-not-showing-in-rviz/
        rospy.sleep(5)

        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.pose.position.z = -0.05
        p.pose.orientation.w = 1
        scene.add_cylinder('table', p, 0.1, 1.5)
        
        super(ActionProcessor, self).__init__(actuator_topic, raw_actuator_topic)

		def _process_action(self, action):
				action = self._get_action_srv()
        safe_action = self._getSafeAction(np.asarray(action))
        return BoxSpaceResponse(safe_action)

    def _getSafeAction(self, goal_position):
        '''
        Given a goal_position, check if this satisfies the velocity limit 
        and whether the path is collision free
        return a collision free action
        '''
        observation = self.get_observation_service()
        current_position = np.asarray(observation.value)
        self.rs.joint_state.position = current_position

        gsvr = GetStateValidityRequest()
        gsvr.group_name = self.group_name
        gsvr.robot_state = self.rs

        # We interpolate using a cubic spline between the current joint state and the goal state
        # For now, we are not using velocity information, so it is actually linear interpolation
        # We could use the current velocity as a boundary condition for the spline
        t = [0, self.duration]
        x = [current_position, goal_position]
        cs = CubicSpline(t, x)

        # We also check joint limits on velocity
        dif = goal_position - current_position
        too_fast = np.abs(dif/self.duration) > self.vel_limit
        if np.any(too_fast):
            goal_position[too_fast] = current_position[too_fast] + \
                np.sign(dif[too_fast]) * self.duration * self.vel_limit
            x = [current_position, goal_position]
            cs = CubicSpline(t, x)

        # We calculate where the robot is at the next time step
        # We use this to calculate the number of checks we will perform
        next_pos = cs(1.2*self.step_time)
        max_angle_dif = np.max(np.abs(next_pos - current_position))

        n_checks = int(np.ceil(self.checks_per_rad * max_angle_dif))
        way_points = cs(np.linspace(0, 1.2*self.duration, n_checks))
        for i in range(n_checks):
            gsvr.robot_state.joint_state.position = way_points[i, :]
            if not self.state_validity_service.call(gsvr).valid:
                return way_points[max(i-1, 0)]
        return goal_position
