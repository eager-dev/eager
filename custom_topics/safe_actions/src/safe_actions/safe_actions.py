"""
    Checking if action is safe, partially copy paste from https://answers.ros.org/question/203633/collision-detection-in-python/
"""
import sys
import rospy
import moveit_commander
from ros_gym_core.srv import BoxSpace, BoxSpaceResponse
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState 
import numpy as np


class SafeActions():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.group_name = rospy.get_param('~group_name', 'manipulator')
        self.checks_per_rad = rospy.get_param('~checks_per_rad', 25) 
        self.vel_limit = rospy.get_param('~vel_limit', 3.14)
        self.step_time = rospy.get_param('~step_time', 0.1)
        self.duration = rospy.get_param('~duration', 0.5)
        
        rospy.logdebug("[safe_actions] Connecting to planning scene interface")
        scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("[safe_actions] Connected to planning scene interface")
        
        # prepare msg to interface with moveit
        self.rs = RobotState()
        
        # Subscribe to joint joint states
        rospy.Subscriber("joint_states", JointState, self.jointStatesCB, queue_size=1)
        
        rospy.logdebug("[safe_actions] Waiting for joint state message")
        
        joint_state = rospy.wait_for_message("joint_states", JointState)
        
        self.rs.joint_state.name = joint_state.name
        self.rs.joint_state.position = joint_state.position
        
        rospy.logdebug("[safe_actions] Waiting for state_validity service")
        
        # prepare service for collision check
        self.sv_srv = rospy.ServiceProxy('check_state_validity', GetStateValidity)
        # wait for service to become available
        self.sv_srv.wait_for_service()
        
        self._get_action_srv = rospy.ServiceProxy('/ros_env/objects/ur5e1/joints', BoxSpace)
        self._get_action_srv.wait_for_service()
        
        rospy.logdebug("[safe_actions] Adding collision object")
        
        # This is ugly, but seems to be necessary, see: https://answers.ros.org/question/209030/moveit-planningsceneinterface-addbox-not-showing-in-rviz/
        rospy.sleep(3)
        
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.pose.orientation.w = 1 
        scene.add_cylinder('table', p, 0.1, 2.3)
        
        self._set_act_srv = rospy.Service('/ros_env/objects/ur5e1/joints/preprocessed', BoxSpace, self._action_service)

    def _action_service(self, req):
        action = self._get_action_srv()
        safe_action = self.getSafeAction(np.asarray(action.value))
        rospy.logdebug("Processed action: {}".format(action))
        return BoxSpaceResponse(safe_action)
    
    def jointStatesCB(self, msg):
        '''
        update robot state
        '''
        self.rs.joint_state.position = np.asarray(msg.position)
        self.joint_states_received = True

    def getSafeAction(self, goal):
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.group_name = self.group_name
        gsvr.robot_state = self.rs
        
        current = self.rs.joint_state.position
        
        # First we limit positions that violate max joint velocity
        angle_dif = goal - current
        too_fast = np.abs(angle_dif)/self.duration > self.vel_limit
        goal[too_fast] = current[too_fast] + self.duration*self.vel_limit*np.sign(angle_dif[too_fast])
        
        # Next we check where the joints are at the next time step
        next_pos = self.step_time * (goal - current)/self.duration
        step_angle_dif = next_pos - current
        max_angle_dif = np.max(np.abs(step_angle_dif))
        
        n_checks = int(np.ceil(self.checks_per_rad * max_angle_dif))
        
        for i in range(n_checks):
            last_position = gsvr.robot_state.joint_state.position
            gsvr.robot_state.joint_state.position = current + step_angle_dif*float(i+1)/float(n_checks)
            if not self.sv_srv.call(gsvr).valid:
                return last_position
        return goal

    
    
