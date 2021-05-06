#!/usr/bin/env python3
import abc
import rospy
from physics_bridge.srv import SetFloat, SetFloatResponse
from std_srvs.srv import SetBool, SetBoolResponse
from Queue import Queue

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionServer(ABC):
    
    def __init__(self, topic, name, actuator, action_type):
        rospy.init_node(name + "_action_server")
        
        self.enabled = False
        self.action = None

        self.queue = Queue(maxsize=1)
        self.msg_type = action_type
        self.rate = rospy.Rate(actuator["rate"])
        self.pub = rospy.Publisher(topic, self.msg_type, queue_size=actuator["queue_size"])
        
        self.__enable_service = rospy.Service(name + '/enable', SetBool, self.__enable_handler)
        self.__set_action_service = rospy.Service(name + '/value', SetFloat, self.__set_action_handler)
    
    def publish_action(self):
        # TODO: Fix that latest action is used
        # Code copied from robo-gym
        while not rospy.is_shutdown() and self.enabled:
            if self.queue.full():
                self.pub.publish(self.queue.get())
                self.stop_flag = False 
            else:
                if not self.stop_flag:
                    self.pub.publish(self.msg_type())
                    self.stop_flag = True 
                else:
                    pass 
            self.rate.sleep()
    
    @abc.abstractmethod
    def _convert_action(self, action_raw):
        return None # Converted message
    
    def __enable_handler(self, req):
        self.enabled = req.data
        response = SetBoolResponse()
        response.success = True
        return response
        
    def __set_action_handler(self, req):
        action_raw = req.data
        action = self._convert_action(action_raw)
        try:
            # Add to the Queue the next command to execute
            self.queue.put(action)
        except:
            pass
        response = SetFloatResponse()
        response.success = True
        return response
    
