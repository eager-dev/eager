import gym, gym.spaces
from collections import OrderedDict
from ros_gym_core.objects import Robot, Sensor, Actuator
from ros_gym_core.ros_env import BaseRosEnv
from typing import List, Tuple, Callable

class Ur5eEnv(BaseRosEnv):
    def __init__(self, ur5e: Robot, sensors: List[Sensor] = [], observers: List['Observer'] = [], **kwargs) -> None:
        # todo: Interface changes a lot, use **kwargs.
        #  Make arguments of baseclass explicit when interface is more-or-less fixed.
        super().__init__(**kwargs)
        self.ur5e = ur5e
        self.sensors = sensors
        self.observers = observers

        self._init_nodes([self.ur5e], self.sensors, self.observers)

        obs_spaces = OrderedDict()
        act_spaces = OrderedDict()
        obs_spaces[self.ur5e.name] = self.ur5e.observation_space
        act_spaces[self.ur5e.name] = self.ur5e.action_space
        self.observation_space = gym.spaces.Dict(spaces=obs_spaces)
        self.action_space = gym.spaces.Dict(spaces=act_spaces)

        # self.observation_space, self.action_space = self._merge_spaces(self.robots, self.sensors, self.observers)

    def step(self, action: 'OrderedDict[str, object]') -> Tuple[object, float, bool, dict]:

        self.ur5e.set_action(action[self.ur5e.name])

        self._step()

        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self._is_done(obs)
        return obs, reward, done, {}

    def reset(self) -> object:

        self.ur5e.reset()

        self._reset()

        return self._get_obs()

    def render(self, mode: str = 'human') -> None:
        # Send render command
        pass

    def close(self) -> None:
        self._close()

    def seed(self, seed=None) -> None:
        # How to implement?
        pass

    def _get_obs(self) -> 'OrderedDict[str, object]':

        obs = OrderedDict()

        obs[self.ur5e.name] = self.ur5e.get_obs()

        return obs

    def _get_states(self) -> 'OrderedDict[str, object]':
        # How / what is it??
        pass

    def _get_reward(self, obs: 'OrderedDict[str, object]') -> float:
        # if needed:
        # states = self._get_states()
        return 0.0

    def _is_done(self, obs: 'OrderedDict[str, object]') -> bool:
        return False