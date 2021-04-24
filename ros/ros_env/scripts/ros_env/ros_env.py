import gym
from .objects import *
from collections import OrderedDict
from typing import List, Tuple, Callable

class BaseRosEnv(gym.Env):
    pass

class RosEnv(BaseRosEnv):

    def __init__(self, robots: List[Robot] = [], sensors: List[Sensor] = [], custom_topics: List['Observer'] = []) -> None:
        super().__init__()
        self.robots = robots
        self.sensors = sensors
        self.custom_topics = custom_topics

        # Send init to pysics bridge
        # merge names so: robot1.name/sensor1.name/value is the value channel for sensor1 of robot1

        # Merge and set spaces here
    
    def step(self, action: object) -> Tuple[object, float, bool, dict]:

        # Same issue as in Robot class
        # I don't think we're guaranteed to also get a dict back... enforce?

        for robot in self.robots:
            robot.set_action(action[:1]) # How to split?

        # Send step

        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self.is_done(obs)
        return obs, reward, done, {}
    
    def reset(self) -> object:

        for robot in self.robots:
            robot.reset()

        #Send global reset

        return self._get_obs()

    def render(self, mode: str = 'human') -> None:
        # Send render command
        pass

    def close(self) -> None:
        # Send close
        pass

    def seed(self, seed=None) -> None:
        # How to implement?
        pass
    
    def _get_obs(self) -> 'OrderedDict[str, object]':

        obs = OrderedDict()

        for custom_topic in self.custom_topics:
            obs[custom_topic.name] = custom_topic.get_obs()

        for sensor in self.sensors:
            obs[sensor.name] = sensor.get_obs()

        for robot in self.robots:
            obs[robot.name] = robot.get_obs()

        return obs
    
    def _get_states(self) -> 'OrderedDict[str, object]':
        # How / what is it??
        pass

    def _get_reward(self, obs: 'OrderedDict[str, object]') -> float:
        # if needed:
        states = self._get_states()