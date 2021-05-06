import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header


def create_action_converter(action_type):
    def convert_action_to_joint_trajectory(self, action_raw, joint_state, 
                                           joint_names, joint_limits, 
                                           min_traj_duration):
        # Code copied from robo-gym
        action = JointTrajectory()
        action.header = Header()
        action.joint_names = joint_names
        action.points=[JointTrajectoryPoint()]
        action.points[0].positions = action_raw
        duration = []
        for i in range(len(action.joint_names)):
            pos = joint_state.position[0:len(joint_names)]
            cmd = action_raw[i]
            max_vel = joint_limits[i]
            duration.append(max(abs(cmd-pos)/max_vel, min_traj_duration))
    
        action.points[0].time_from_start = rospy.Duration.from_sec(max(duration))
        return action
    if action_type.lower() == "jointtrajectorypoint":
        return convert_action_to_joint_trajectory
