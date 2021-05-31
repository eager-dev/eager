import abc
import rospy
import ast
from eager_core.srv import Register, StepEnv, ResetEnv, CloseEnv
from eager_core.utils.file_utils import load_yaml
from eager_core.utils.message_utils import get_message_from_def, get_response_from_def

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
            br_params = params[self._bridge_type]
            if 'sensors' in br_params:
                for sensor in br_params['sensors']:
                    sens_def = params['sensors'][sensor]
                    br_params['sensors'][sensor]['messages'] = (get_message_from_def(sens_def), get_response_from_def(sens_def))
            if 'actuators' in br_params:
                for actuator in br_params['actuators']:
                    act_def = params['actuators'][actuator]
                    br_params['actuators'][actuator]['messages'] = (get_message_from_def(act_def), get_response_from_def(act_def))
            if 'states' in br_params:
                for state in br_params['states']:
                    state_def = params['states'][state]
                    br_params['states'][state]['messages'] = (get_message_from_def(state_def), get_response_from_def(state_def))
                    
            self._register_object("objects/" + object.name, object.name, object_type[0], object_type[1], args, br_params)

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
