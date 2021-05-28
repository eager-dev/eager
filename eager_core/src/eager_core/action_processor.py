import abc
import rospy
from eager_core.srv import RegisterActuatorProcessor, ResetEnv, CloseEnv, BoxSpace, BoxSpaceResponse

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()}) 

class ActionProcessor(ABC):
    
    def __init__(self, name):
        self._get_observation_services = {}
        
        self.__register_service = rospy.Service('register_{}'.format(name), RegisterActuatorProcessor, self.__register_handler)
        self.__reset_service = rospy.Service('reset_{}'.format(name), ResetEnv, self.__reset_handler)
        self.__close_service = rospy.Service('close_{}'.format(name), CloseEnv, self.__close_handler)
        
        
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
        ns = rospy.get_namespace()
        env = ns[:ns.find('/',1)+1]
        actuator_processor = req.actuator_processor
        raw_actuator_topic =  actuator_processor.raw_actuator_topic
        actuator = actuator_processor.actuator
        for robot in actuator_processor.observations:
            self._get_observation_services[robot] = {}
            for sensor in  actuator_processor.observations[robot].sensors:
                self._get_observation_services[robot][sensor] = rospy.ServiceProxy(env + robot + sensor, BoxSpace)
        self._get_action_service = rospy.ServiceProxy(raw_actuator_topic, BoxSpace)
        self.__process_action_service = rospy.Service(actuator, BoxSpace, self.__process_action_handler)
        return () # Success
        
    def __process_action_handler(self, req):
        observation = {}
        action = self._get_action_service()
        for robot in self.observation_services:
            observation[robot] = {}
            for sensor in self.observation_services[robot]:
                get_observation_service = [robot][sensor] = self.observation_services[robot][sensor]
                observation[robot][sensor] = get_observation_service()
        action_processed = self._process_action(action.value, observation)
        return BoxSpaceResponse(action_processed)

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
