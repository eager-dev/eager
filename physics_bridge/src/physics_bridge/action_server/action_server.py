#!/usr/bin/env python3
import abc
import rospy
import actionlib

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionServer(ABC):
    
    def __init__(self, action_server, action_type):
        self.client = actionlib.SimpleActionClient(action_server, action_type)
        rospy.logdebug("Waiting for server {} ...".format(action_server))
        self.client.wait_for_server()
        rospy.logdebug("[{}] Connected to server".format(action_server))
    
    def act(self, action):
        self.client.send_goal(action)