import gym, gym.spaces
from collections import OrderedDict
from typing import List, Tuple, Callable

class Sensor():
    def __init__(self, name: str, preprocess_service: str = None) -> None:
        self.name = name
        self.preprocess_service = preprocess_service

    def get_obs(self) -> object: #Type depends on space
        # Request obs from bridge
        pass

    def observation_space(self) -> gym.Space:
        if self.preprocess_service is not None:
            # Get space from preprocess
            pass
        pass

class Actuator(): # Do actuators need seperate resets?

    def __init__(self, name: str, preprocess_service: str = None) -> None:
        self.name = name
        self.preprocess_service = preprocess_service
    
    def set_action(self, action: object) -> None:
        # Set action to action buffer
        pass

    def action_space(self) -> gym.Space:
        if self.preprocess_service is not None:
            # Get space from preprocess
            pass
        pass

    def _action_service(self, message: object) -> object:
        # get latest action from buffer
        pass

# Composition over inheritance is better here here I think
class Robot():
    def __init__(self, name: str, sensors: List[Sensor], actuators: List[Actuator], reset: Callable[['Robot'],  None] = None) -> None:
        self.name = name

        # These can also be dicts instead, with names as keys
        self.sensors = sensors
        self.actuators = actuators
        self.reset_func = reset

    def set_action(self, action: object) -> None: # Error return?

        # I don't think we're guaranteed to also get a dict back... enforce?

        #Split actions?
        for actuator in self.actuators:
            actuator.set_action(action[1:])
    
    def get_obs(self) -> 'OrderedDict[str, object]':

        obs = OrderedDict() # We might get away with using a list in some cases, might be quicker
        
        #Split actions?
        for sensors in self.sensors:
            obs[sensors.name] = sensors.get_obs()
        
        return obs
    
    def action_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for actuator in self.actuators:
            spaces[actuator.name] = actuator.action_space()

        return gym.spaces.Dict(spaces=spaces)

    def observation_space(self) -> gym.spaces.Dict:
        spaces = OrderedDict()
        for sensor in self.sensors:
            spaces[sensor.name] = sensor.observation_space()

        return gym.spaces.Dict(spaces=spaces)

    def reset(self) -> None: # Error return?
        if self.reset_func is not None:
            self.reset_func(self)