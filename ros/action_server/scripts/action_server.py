#!/usr/bin/env python3
import rospy

class ActionServer():
    def __init__(self):
        rospy.init_node('action_server')
        
        self.name = rospy.get_namespace()
        
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
        
        subscriber_topic = '/physics_bridge/' + self.name
        
        rospy.Subscriber(subscriber_topic, msg, self.callback, queue_size=1)
        
        self.action = None
        
    def callback(self, action_msg):
        self.action = action_msg
        
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
            
    
            
            
            
            
            
            
        
        
        