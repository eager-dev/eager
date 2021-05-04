#!/usr/bin/env python3
import abc
import rospy, rospkg, rosparam
from physics_bridge.srv import SetFloat, SetFloatResponse
from std_srvs.srv import SetBool, SetBoolResponse
from Queue import Queue

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionServer(ABC):
    
    def __init__(self, name, object_type, action_topic, msg_type, queue_size=10, rate=30):
        self.enabled = False
        self.action = None
        
        pp = rospkg.RosPack().get_path("physics_bridge")
        filename = pp + "/config/robots/" + object_type + ".yaml"
        self.params = rosparam.load_file(filename)[0][0]
        
        self.queue = Queue(maxsize=1)
        
        self.msg_type = msg_type
        self.rate = rospy.Rate(rate)
        self.pub = rospy.Publisher(action_topic, msg_type, queue_size=queue_size)
        
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
    
