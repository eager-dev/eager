#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *

class ActionServer():
    def __init__(self):
        rospy.init_node('action_server')
        
        
        name = rospy.get_namespace()
        self._action_name = name
        
        rate = rospy.get_param('~rate', default=30)
        self.rate = rospy.Rate(rate)
        
        action_info = rospy.get_param(
            '~action_info', 
            default={
                'topic': 'pos_traj_controller/command',
                'queue_size': 10,
                'msg_package': 'trajectory_msgs.msg',
                'msg': 'JointTrajectoryPoint' 
                }
            )
        
        # Import pub msg types
        msg_package_name = action_info['msg_package']
        msg_name = action_info['msg']
        
        msg_package = __import__(msg_package_name)
        msg = getattr(msg_package, msg_name) 
        
        
        # Create publishers
        publisher_topic = action_info['topic']
        queue_size = action_info['queue_size']
        self.pub = rospy.Publisher(publisher_topic, msg, queue_size=queue_size)
        
        self.action = None
        
    def set_action_cb(self, goal):
        success = True
        self.action = goal
        
        while not rospy.is_shutdown():
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self.pub.
                
    def publish_actions(self): 
        while not rospy.is_shutdown():
            if self.action is not None:
                self.pub.publish(self.action)
            else:
                pass 
            self.rate.sleep()
            
if __name__ == '__main__':
    try:
        actionServer = ActionServer()
        actionServer.publish_actions()
    except rospy.ROSInterruptException:
        pass
            
    
            
            
            
            
            
            
        
        
        