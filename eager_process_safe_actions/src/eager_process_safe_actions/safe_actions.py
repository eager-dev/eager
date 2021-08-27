import sys
import rospy
import roslaunch
import moveit_commander
from eager_core.action_processor import ActionProcessor
from eager_core.utils.file_utils import substitute_xml_args
from eager_core.utils.file_utils import load_yaml
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import CubicSpline
import numpy as np


class SafeActions(ActionProcessor):
    '''
    Custom action processing node for manipulators that are not allowed to move below a certain height.
    Can be used for collision avoidance, for example for manipulators that are mounted on a table or other sort of base
    surface. Checks velocity limits, prevents self-collision and collision with the base surface.

    !!!
    Be careful! Implementation assumes constant velocity based on the difference between the start and goal position
    and the duration parameter. If this assumption does not hold, velocity limit checking and collision avoidance will
    not work!
    !!!

    '''

    def __init__(self):
        # Get params
        self.vel_limit = rospy.get_param('~vel_limit')
        self.duration = rospy.get_param('~duration')
        self.checks_per_rad = rospy.get_param('~checks_per_rad')
        collision_height = rospy.get_param('~collision_height')
        robot_type = rospy.get_param('~robot_type')

        # Get params from config file
        params = load_yaml('eager_process_safe_actions', '{}'.format(robot_type))
        self.joint_names = params['joint_names']
        self.group_name = params['group_name']
        moveit_package = params['moveit_package']
        urdf_path = params['urdf_path']
        base_frame = params['base_frame']
        base_length = params['base_length']
        workspace_length = params['workspace_length']

        # Create robot_description on parameter server
        str_launch_sim = '$(find eager_process_safe_actions)/launch/upload_robot_description.launch'
        cli_args = [substitute_xml_args(str_launch_sim),
                    'ns:={}'.format(rospy.get_namespace()),
                    'urdf_path:={}'.format(urdf_path),
                    ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        urdf_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        urdf_launch.start()

        # Launch MoveIt
        str_launch_sim = '$(find {})/launch/move_group.launch'.format(moveit_package)
        cli_args = [substitute_xml_args(str_launch_sim),
                    'allow_trajectory_execution:=false',
                    'publish_monitored_planning_scene:=false',
                    'fake_execution:=false',
                    ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        moveit_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        moveit_launch.start()

        # Initialize Moveit Commander and Scene
        moveit_commander.roscpp_initialize(sys.argv)
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # Five collision objects are added to the scene, such that the base is
        # not in collision, while the rest of the surface is a collision object.

        object_length = (workspace_length - base_length) / 2.0

        # Add a collision object to the scenes
        p0 = PoseStamped()
        p0.header.frame_id = base_frame
        p0.pose.position.z = - 0.06
        p0.pose.orientation.w = 1

        # Add a collision object to the scenes
        p1 = PoseStamped()
        p1.header.frame_id = base_frame
        p1.pose.position.x = (object_length + base_length) / 2.0
        p1.pose.position.z = collision_height / 2.0
        p1.pose.orientation.w = 1

        # Add a collision object to the scenes
        p2 = PoseStamped()
        p2.header.frame_id = base_frame
        p2.pose.position.x = - (object_length + base_length) / 2.0
        p2.pose.position.z = collision_height / 2.0
        p2.pose.orientation.w = 1

        # Add a collision object to the scenes
        p3 = PoseStamped()
        p3.header.frame_id = base_frame
        p3.pose.position.y = (object_length + base_length) / 2.0
        p3.pose.position.z = collision_height / 2.0
        p3.pose.orientation.w = 1

        # Add a collision object to the scenes
        p4 = PoseStamped()
        p4.header.frame_id = base_frame
        p4.pose.position.y = - (object_length + base_length) / 2.0
        p4.pose.position.z = collision_height / 2.0
        p4.pose.orientation.w = 1

        scene.add_box('base0', p0, size=(workspace_length, workspace_length, 0.1))
        scene.add_box('base1', p1, size=(object_length, base_length, collision_height))
        scene.add_box('base2', p2, size=(object_length, base_length, collision_height))
        scene.add_box('base3', p3, size=(workspace_length, object_length, collision_height))
        scene.add_box('base4', p4, size=(workspace_length, object_length, collision_height))

        # Initialize state validity check service
        self.state_validity_service = rospy.ServiceProxy('check_state_validity', GetStateValidity)
        self.state_validity_service.wait_for_service()

        self.previous_position = None
        super(SafeActions, self).__init__()

    def _get_space(self):
        space = {}
        space['low'] = [-3.14] * len(self.joint_names)
        space['high'] = [3.14] * len(self.joint_names)
        space['type'] = 'boxf32'
        return space

    def _close(self):
        pass

    def _reset(self):
        pass

    def _process_action(self, action, observation):
        if len(observation) > 1:
            rospy.logwarn("[{}] Expected observation from only one robot".format(rospy.get_name()))
        for robot in observation:
            if len(observation[robot]) > 1:
                rospy.logwarn("[{}] Expected observation from only one sensor".format(rospy.get_name()))
            for sensor in observation[robot]:
                current_position = observation[robot][sensor]
        safe_action = self._getSafeAction(np.asarray(action), np.asarray(current_position))
        return safe_action

    def _getSafeAction(self, goal_position, current_position):
        '''
        Given a goal_position, check if this satisfies the velocity limit
        and whether the path is collision free
        return a collision free action
        '''
        rs = RobotState()
        rs.joint_state.name = self.joint_names
        rs.joint_state.position = current_position

        gsvr = GetStateValidityRequest()
        gsvr.group_name = self.group_name
        gsvr.robot_state = rs

        # We interpolate using a cubic spline between the current joint state and the goal state
        # For now, we are not using velocity information, so it is actually linear interpolation
        # We could use the current velocity as a boundary condition for the spline
        t = [0, self.duration]
        x = [current_position, goal_position]
        cs = CubicSpline(t, x)

        # We also check joint limits on velocity
        dif = goal_position - current_position
        too_fast = np.abs(dif / self.duration) > self.vel_limit
        if np.any(too_fast):
            goal_position[too_fast] = current_position[too_fast] + \
                np.sign(dif[too_fast]) * self.duration * self.vel_limit
            x = [current_position, goal_position]
            cs = CubicSpline(t, x)

        max_angle_dif = np.max(np.abs(goal_position - current_position))

        n_checks = int(np.ceil(self.checks_per_rad * max_angle_dif))
        way_points = cs(np.linspace(0, self.duration, n_checks))

        for i in range(n_checks):
            gsvr.robot_state.joint_state.position = way_points[i, :]
            if not self.state_validity_service.call(gsvr).valid:
                if i == 0:
                    rospy.logwarn("Current state in collision!")
                    return self.previous_position
                self.previous_position = current_position
                return way_points[i-1, :]
        self.previous_position = current_position
        return goal_position
