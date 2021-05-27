import abc
import rospy
import ast
from ros_gym_core.srv import Register, StepEnv, ResetEnv, CloseEnv
from ros_gym_core.utils.file_utils import load_yaml

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class PhysicsBridge(ABC):
    
    def __init__(self, bridge_type):
        self._bridge_type = bridge_type
        self.__register_service = rospy.Service('register', Register, self.__register_handler)
        self.__step_service = rospy.Service('step', StepEnv, self.__step_handler)
        self.__reset_service = rospy.Service('reset', ResetEnv, self.__reset_handler)
        self.__close_service = rospy.Service('close', CloseEnv, self.__close_handler)

    @abc.abstractmethod
    def _register_object(self, topic, name, package, object_type, args, config):
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
        for object in req.objects:
            object_type = object.type.split('/')
            args = ast.literal_eval(object.args)
            params = load_yaml(object_type[0], object_type[1])
            self._register_object("objects/" + object.name, object.name, object_type[0], object_type[1], args, params[self._bridge_type])

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
