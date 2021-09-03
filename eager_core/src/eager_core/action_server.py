import abc
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})


class ActionServer(ABC):

    def __init__(self, action_server, action_type):
        self.client = actionlib.SimpleActionClient(action_server, action_type)
        rospy.logdebug("Waiting for server {} ...".format(action_server))
        self.client.wait_for_server()
        rospy.logdebug("[{}] Connected to server".format(action_server))

    @abc.abstractmethod
    def act(self, action):
        self.client.send_goal(action)


class FollowJointTrajectoryActionServer(ActionServer):
    def __init__(self, joint_names, server_name, duration=0.5):
        self.joint_names = joint_names
        self.duration = duration
        super(FollowJointTrajectoryActionServer, self).__init__(server_name, FollowJointTrajectoryAction)

    def act(self, action_raw):
        action = FollowJointTrajectoryGoal()
        action.trajectory = JointTrajectory()
        action.trajectory.header = Header()
        action.trajectory.joint_names = self.joint_names
        action.trajectory.points = [JointTrajectoryPoint()]
        action.trajectory.points[0].positions = action_raw
        action.trajectory.points[0].time_from_start = rospy.Duration(self.duration)
        self.client.send_goal(action)
