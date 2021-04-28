import gym, gym.spaces
from collections import OrderedDict
from typing import List, Callable, Type, Tuple
import rospy
import numpy as np

from ros_env.srv import BoxSpace

class BaseRosObject():

    def __init__(self, name: str) -> None:
        self.name = name

    def _infer_space(self, base_topic: str = '') -> gym.Space:
        pass

    def _get_message_class(cls, space_class: Type[gym.Space]) -> Type:
        if space_class is gym.spaces.Box:
            return BoxSpace
        else:
            raise NotImplementedError

    def init_node(self, base_topic: str = '') -> None:
        raise NotImplementedError
    
    def get_topic(self, base_topic: str = '') -> str:
        if base_topic == '':
            return self.name
        return base_topic + '/' + self.name


class Sensor(BaseRosObject):
    def __init__(self, name: str, space: gym.Space = None) -> None:
        super().__init__(name)
        self.observation_space = space

    def init_node(self, base_topic: str = '') -> None:
        if self.observation_space is None:
            self.observation_space = self._infer_space(base_topic)
        
        self._obs_service = rospy.ServiceProxy(self.get_topic(base_topic), self._get_message_class(type(self.observation_space)))


    def get_obs(self) -> object: #Type depends on space
        response = self._obs_service()
        return np.array(response.value)

    def add_preprocess(self, processed_space: gym.Space = None, launch_path='/path/to/custom/sensor_preprocess/ros_launchfile', node_type='service', stateless=True):
        self.observation_space = processed_space
        self.launch_path = launch_path
        self.node_type = node_type
        self.stateless = stateless

class Actuator(BaseRosObject):

    def __init__(self, name: str, space: gym.Space = None) -> None:
        super().__init__(name)
        self.action_space = space

    def init_node(self, base_topic: str = ''):
        if self.action_space is None:
            self.action_space = self._infer_space(base_topic)

        self._buffer = self.action_space.sample()
        
        self._act_service = rospy.Service(self.get_topic(base_topic), self._get_message_class(type(self.action_space)), self._action_service)
    
    def set_action(self, action: object) -> None:
        self._buffer = action

    def add_preprocess(self, processed_space: gym.Space = None, launch_path='/path/to/custom/actuator_preprocess/ros_launchfile', node_type='service', stateless=True):
        self.action_space = processed_space
        self.launch_path = launch_path
        self.node_type = node_type
        self.stateless = stateless
    
    def reset(self) -> None:
        pass

    def _action_service(self, request: object) -> object:
        return self._buffer

class Robot(BaseRosObject):
    def __init__(self, name: str, sensors: List[Sensor], actuators: List[Actuator], reset: Callable[['Robot'],  None] = None) -> None:
        super().__init__(name)

        #TODO: Transform to dicts
        self.sensors = sensors
        self.actuators = actuators
        self.reset_func = reset

    def init_node(self, base_topic: str = '') -> None:

        for sensor in self.sensors:
            sensor.init_node(self.get_topic(base_topic))
        
        for actuator in self.actuators:
            actuator.init_node(self.get_topic(base_topic))

    def set_action(self, action: 'OrderedDict[str, object]') -> None: # Error return?

        for actuator in self.actuators:
            actuator.set_action(action[actuator.name])
    
    def get_obs(self) -> 'OrderedDict[str, object]':

        obs = OrderedDict() # We might get away with using a list in some cases, might be quicker
        
        for sensors in self.sensors:
            obs[sensors.name] = sensors.get_obs()
        
        return obs
    
    @property
    def action_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for actuator in self.actuators:
            spaces[actuator.name] = actuator.action_space

        return gym.spaces.Dict(spaces=spaces)

    @property
    def observation_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for sensor in self.sensors:
            spaces[sensor.name] = sensor.observation_space

        return gym.spaces.Dict(spaces=spaces)

    def reset(self) -> None: # Error return?
        for actuator in self.actuators:
            actuator.reset()
        
        if self.reset_func is not None:
            self.reset_func(self)
    
    def get_topics(self, base_topic: str = '') -> Tuple[List[str], List[str]]:
        bt = self.get_topic(base_topic)

        sens_topics = []
        for sensor in self.sensors:
            sens_topics.append(sensor.get_topic(bt))
        
        act_topics = []
        for actuator in self.actuators:
            act_topics.append(actuator.get_topic(bt))
        return sens_topics, act_topics