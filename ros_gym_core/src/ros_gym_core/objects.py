import gym, gym.spaces
from ros_gym_core.utils.file_utils import load_yaml
from ros_gym_core.utils.gym_utils import *
from collections import OrderedDict
from typing import Dict, List, Callable, Union
import rospy
import numpy as np

class BaseRosObject():

    def __init__(self, type: str, name: str, **kwargs) -> None:
        self.type = type
        self.name = name
        self.args = str(kwargs)

    def _infer_space(self, base_topic: str = '') -> gym.Space:
        pass

    def init_node(self, base_topic: str = '') -> None:
        raise NotImplementedError
    
    def get_topic(self, base_topic: str = '') -> str:
        if base_topic == '':
            return self.name
        return base_topic + '/' + self.name


class Sensor(BaseRosObject):
    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.observation_space = space

    def init_node(self, base_topic: str = '') -> None:
        if self.observation_space is None:
            self.observation_space = self._infer_space(base_topic)
        
        self._obs_service = rospy.ServiceProxy(self.get_topic(base_topic), get_message_from_space(type(self.observation_space)))

    def get_obs(self) -> object: #Type depends on space
        response = self._obs_service()
        return np.array(response.value)

    def add_preprocess(self, processed_space: gym.Space = None, launch_path='/path/to/custom/sensor_preprocess/ros_launchfile', node_type='service', stateless=True):
        self.observation_space = processed_space
        self.launch_path = launch_path
        self.node_type = node_type
        self.stateless = stateless


class State(BaseRosObject):
    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.state_space = space

    def init_node(self, base_topic: str = '') -> None:
        if self.state_space is None:
            self.state_space = self._infer_space(base_topic)

        self._state_service = rospy.ServiceProxy(self.get_topic(base_topic),
                                                 get_message_from_space(type(self.state_space)))

    def get_state(self) -> object:  # Type depends on space
        response = self._state_service()
        return np.array(response.value)


class Actuator(BaseRosObject):

    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.action_space = space

    def init_node(self, base_topic: str = ''):
        if self.action_space is None:
            self.action_space = self._infer_space(base_topic)

        self._buffer = self.action_space.sample()
        
        self._act_service = rospy.Service(self.get_topic(base_topic), get_message_from_space(type(self.action_space)), self._action_service)
    
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
        msg_class = get_response_from_space(type(self.action_space))
        return msg_class(self._buffer)


class Robot(BaseRosObject):
    def __init__(self, type: str, name: str, 
                 sensors: Union[List[Sensor], Dict[str, Sensor]],
                 actuators: Union[List[Actuator], Dict[str, Actuator]], 
                 states: Union[List[State], Dict[str, State]],
                 reset: Callable[['Robot'], None] = None,
                 position: List[float] = [0, 0, 0],
                 orientation: List[float] = [0, 0, 0, 1],
                 fixed_base: bool = True,
                 self_collision: bool = True,
                 **kwargs
                 ) -> None:
        super().__init__(type, name, position=position, orientation=orientation, fixed_base=fixed_base, self_collision=self_collision, **kwargs)

        if isinstance(sensors, List):
            sensors = OrderedDict(zip([sensor.name for sensor in sensors], sensors))
        self.sensors = sensors if isinstance(sensors, OrderedDict) else OrderedDict(sensors)

        if isinstance(actuators, List):
            actuators = OrderedDict(zip([actuator.name for actuator in actuators], actuators))
        self.actuators = actuators if isinstance(actuators, OrderedDict) else OrderedDict(actuators)

        if isinstance(states, List):
            states = OrderedDict(zip([state.name for state in states], states))
        self.states = states if isinstance(states, OrderedDict) else OrderedDict(states)

        self.reset_func = reset
    
    @classmethod
    def create(cls, name: str, package_name: str, robot_type: str,
               position: List[float] = [0, 0, 0],
               orientation: List[float] = [0, 0, 0, 1],
               fixed_base: bool = True,
               self_collision: bool = True,
               **kwargs
               ) -> 'Robot':

        params = load_yaml(package_name, robot_type)

        sensors = []
        for sens_name, sensor in params['sensors'].items():
            sensor_space = get_space_from_def(sensor)
            sensors.append(Sensor(None, sens_name, sensor_space))

        actuators = []
        for act_name, actuator in params['actuators'].items():
            act_space = get_space_from_def(actuator)
            actuators.append(Actuator(None, act_name, act_space))

        states = []
        for state_name, state in params['states'].items():
            state_space = get_space_from_def(state)
            states.append(State(None, state_name, state_space))

        return cls(package_name + '/' + robot_type, name, sensors, actuators, states,
            position=position, orientation=orientation, fixed_base=fixed_base, self_collision=self_collision, **kwargs)

    def init_node(self, base_topic: str = '') -> None:

        for sensor in self.sensors.values():
            sensor.init_node(self.get_topic(base_topic))
        
        for actuator in self.actuators.values():
            actuator.init_node(self.get_topic(base_topic))

        for state in self.states.values():
            state.init_node(self.get_topic(base_topic))

    def set_action(self, action: 'OrderedDict[str, object]') -> None: # Error return?

        for act_name, actuator in self.actuators.items():
            actuator.set_action(action[act_name])
    
    def get_obs(self, sensors: List[str] = None) -> 'OrderedDict[str, object]':
        if sensors is not None:
            obs = OrderedDict([(sensor, self.sensors[sensor].get_obs()) for sensor in sensors])
        else:
            obs = OrderedDict([(sens_name, sensor.get_obs()) for sens_name, sensor in self.sensors.items()])
        return obs

    def get_state(self, states: List[str] = None) -> 'OrderedDict[str, object]':
        if states is not None:
            obs = OrderedDict([(state, self.states[state].get_state()) for state in states])
        else:
            obs = OrderedDict([(state_name, state.get_state()) for state_name, state in self.states.items()])
        return obs
    
    @property
    def action_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for act_name, actuator in self.actuators.items():
            spaces[act_name] = actuator.action_space

        return gym.spaces.Dict(spaces=spaces)

    @property
    def observation_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for sens_name, sensor in self.sensors.items():
            spaces[sens_name] = sensor.observation_space

        return gym.spaces.Dict(spaces=spaces)

    @property
    def state_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for state_name, state in self.states.items():
            spaces[state_name] = state.state_space

        return gym.spaces.Dict(spaces=spaces)

    def reset(self) -> None: # Error return?
        for actuator in self.actuators.values():
            actuator.reset()
        
        if self.reset_func is not None:
            self.reset_func(self)