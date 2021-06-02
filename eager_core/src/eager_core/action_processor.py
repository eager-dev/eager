import abc
import rospy
from eager_core.srv import RegisterActionProcessor, ResetEnv, CloseEnv
from eager_core.utils.file_utils import load_yaml
from eager_core.utils.message_utils import get_message_from_def, get_response_from_def

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionProcessor(ABC):
    
    def __init__(self):
        rospy.logdebug("[{}] Initializing action processor".format(rospy.get_name()))
        self._get_observation_services = {}        
        self.__register_service = rospy.Service('register_action_processor', RegisterActionProcessor, self.__register_handler)
        self.__reset_service = rospy.Service('reset_action_processor', ResetEnv, self.__reset_handler)
        self.__close_service = rospy.Service('close_action_processor', CloseEnv, self.__close_handler)
           
    @abc.abstractmethod
    def _process_action(self, action, observation):
        return action

    @abc.abstractmethod
    def _reset(self):
        pass

    @abc.abstractmethod
    def _close(self):
        pass
        
    def __register_handler(self, req):
        rospy.logdebug("[{}] Handling register request".format(rospy.get_name()))
        ns = rospy.get_namespace()
        env = ns[:ns.find('/',1)+1]
        actuator = req.actuator
        
        raw_action_object = {'type' : req.raw_action_type}
        raw_action_msg = get_message_from_def(raw_action_object)
        
        action_object = {'type' : req.action_type}
        action_msg = get_message_from_def(action_object)
        self.response = get_response_from_def(action_object)
        
        for idx, observation_object in enumerate(req.observation_objects):
            object_name = observation_object.name
            object_type = observation_object.type.split('/')
            object_params = load_yaml(object_type[0], object_type[1])
            self._get_observation_services[object_name] = {}
            for sensor in object_params['sensors']:
                sens_def = object_params['sensors'][sensor]
                msg_type = get_message_from_def(sens_def)
                self._get_observation_services[object_name][sensor] = rospy.ServiceProxy(env + 'objects/' + object_name + '/' + sensor, msg_type)
        self._get_action_service = rospy.ServiceProxy(actuator + '/raw', raw_action_msg)
        self.__process_action_service = rospy.Service(actuator, action_msg, self.__process_action_handler)
        return () # Success
        
    def __process_action_handler(self, req):
        rospy.logdebug("[{}] Handling action request".format(rospy.get_name()))
        observation = {}
        action = self._get_action_service()
        action = action.value
        for robot in self._get_observation_services:
            observation[robot] = {}
            for sensor in self._get_observation_services[robot]:
                get_observation_service = self._get_observation_services[robot][sensor]
                obs = get_observation_service()
                observation[robot][sensor] = obs.value
        action_processed = self._process_action(action, observation)
        return self.response(action_processed)

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
