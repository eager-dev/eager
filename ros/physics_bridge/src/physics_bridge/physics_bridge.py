import abc
import rospy
from ros_env.srv import Register, StepEnv, ResetEnv, CloseEnv

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class PhysicsBridge(ABC):
    
    def __init__(self, bridge_type, name= 'physics_bridge'):
        self._bridge_type = bridge_type
        self.__register_service = rospy.Service(name + '/register', Register, self.__register_handler)
        self.__step_service = rospy.Service(name + '/step', StepEnv, self.__step_handler)
        self.__reset_service = rospy.Service(name + '/reset', ResetEnv, self.__reset_handler)
        self.__close_service = rospy.Service(name + '/close', CloseEnv, self.__close_handler)


    @abc.abstractmethod
    def _register_object(self):
        pass

    @abc.abstractmethod
    def _step(self):
        pass

    @abc.abstractmethod
    def _reset(self):
        pass

    @abc.abstractmethod
    def _close(self):
        pass

    def __register_handler(self, req):
        print(req.sensors)
        print(req.actuators)
        return () # Success

    def __step_handler(self, req):
        if self._step():
            return False # Success
        else:
            return None # Error

    def __reset_handler(self, req):
        if self._reset():
            return () # Success
        else:
            return None # Error

    def __close_handler(self, req):
        if self._close():
            return () # Success
        else:
            return None # Error