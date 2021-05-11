#!/usr/bin/env python3
import abc
import rospy
from multiprocessing import Queue

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionServer(ABC):
    
    def __init__(self, name, actuator, actuator_params, action_type):
        self.action = None
        self.queue = Queue(maxsize=1)
        self.msg_type = action_type
        self.pub = rospy.Publisher(actuator_params["actuator_topic"], self.msg_type, queue_size=actuator_params["queue_size"])
    
    def publish_action(self, event):
        # TODO: Fix that latest action is used
        # Code copied from robo-gym
        if self.queue.full():
            action = self.queue.get() 
            rospy.logdebug("Publishing action {}".format(action))
            self.pub.publish(action)
            self.stop_flag = False 
        else:
            if not self.stop_flag:
                rospy.logdebug("Stopping Actuator")
                self.pub.publish(self.msg_type())
                self.stop_flag = True 
            else:
                pass 
    
    def set_action(self, action_raw):
        action = self._convert_action(action_raw)
        # Add to the Queue the next command to execute
        self.queue.put(action)
                
    @abc.abstractmethod
    def _convert_action(self, action_raw):
        return None # Converted message