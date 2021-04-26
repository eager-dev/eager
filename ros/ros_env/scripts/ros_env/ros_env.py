import gym, gym.spaces
from .objects import *
from collections import OrderedDict
from typing import List, Tuple, Callable
from ros_env.srv import StepEnv, ResetEnv, CloseEnv

class BaseRosEnv(gym.Env):

    def _init_nodes(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []):

        for robot in robots:
            robot.init_node('')
        
        for sensor in sensors:
            sensor.init_node('')
        
        for observer in observers:
            observer.init_node()

    def _merge_spaces(cls, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []):
        
        obs_spaces = OrderedDict()
        act_spaces = OrderedDict()

        for robot in robots:
            obs_spaces[robot.name] = robot.observation_space
            act_spaces[robot.name] = robot.action_space
        
        for sensor in sensors:
            obs_spaces[sensor.name] = sensor.observation_space
        
        for observer in observers:
            obs_spaces[observer.name] = observer.observation_space
        
        return gym.spaces.Dict(spaces=obs_spaces), gym.spaces.Dict(spaces=act_spaces)

class RosEnv(BaseRosEnv):

    def __init__(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []) -> None:
        super().__init__()
        self.robots = robots
        self.sensors = sensors
        self.observers = observers

        self._init_nodes(self.robots, self.sensors, self.observers)

        self.observation_space, self.action_space = self._merge_spaces(self.robots, self.sensors, self.observers)

        self._step_service = rospy.ServiceProxy('physics_bridge/step', StepEnv)
        self._reset_service = rospy.ServiceProxy('physics_bridge/reset', ResetEnv)
        self._close_service = rospy.ServiceProxy('physics_bridge/close', CloseEnv)
    
    def step(self, action: 'OrderedDict[str, object]') -> Tuple[object, float, bool, dict]:

        for robot in self.robots:
            robot.set_action(action[robot.name]) # How to split?

        self._step_service()

        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self.is_done(obs)
        return obs, reward, done, {}
    
    def reset(self) -> object:

        for robot in self.robots:
            robot.reset()

        self._reset_service()

        return self._get_obs()

    def render(self, mode: str = 'human') -> None:
        # Send render command
        pass

    def close(self) -> None:
        self._close_service()

    def seed(self, seed=None) -> None:
        # How to implement?
        pass
    
    def _get_obs(self) -> 'OrderedDict[str, object]':

        obs = OrderedDict()

        for observer in self.observers:
            obs[observer.name] = observer.get_obs()

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