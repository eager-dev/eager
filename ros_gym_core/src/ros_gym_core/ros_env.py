import gym, gym.spaces
import rospy, roslaunch, rosparam
from .objects import *
from collections import OrderedDict
from typing import List, Tuple, Callable
from ros_gym_core.srv import StepEnv, ResetEnv, CloseEnv, Register
from ros_gym_core.msg import Object

class BaseRosEnv(gym.Env):
    def __init__(self, name: str = 'ros_env', engine_params: dict = {}) -> None:
        super().__init__()

        self._initialize_physics_bridge(engine_params=engine_params, name=name)

        self.name = name

        self._step = rospy.ServiceProxy(name + '/step', StepEnv)
        self._reset = rospy.ServiceProxy(name + '/reset', ResetEnv)
        self._close = rospy.ServiceProxy(name + '/close', CloseEnv)

    def _init_nodes(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []) -> None:

        self._register_objects(robots, sensors, observers)
        self._init_listeners(robots, sensors, observers)

    def _register_objects(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []) -> None:
        objects = []

        for el in (robots, sensors, observers):
            for object in el:
                objects.append(Object(object.type, object.name))

        register_service = rospy.ServiceProxy(self.name + '/register', Register)
        register_service.wait_for_service(20)
        register_service(objects)

    def _init_listeners(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []) -> None:
        bt = self.name + '/objects'

        for el in (robots, sensors, observers):
            for object in el:
                object.init_node(bt)

    def _merge_spaces(cls, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = []) -> Tuple[gym.spaces.Dict, gym.spaces.Dict]:
        
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

    def _initialize_physics_bridge(self, engine_params: dict = {}, name: str = 'ros_env'):
        # Delete all parameters parameter server (from a previous run) within namespace 'name'
        # todo: dangerous! could delete parameters if 'name' used by other ros nodes unrelated to this new env
        try:
            rosparam.delete_param('/%s' % name)
            rospy.loginfo('Pre-existing parameters under namespace "/%s" deleted.' % name)
        except:
            pass

        # Upload dictionary with engine parameters to ROS parameter server
        rosparam.upload_params('%s/physics_bridge' % name, engine_params)

        # Launch the physics bridge under the namespace 'name'
        cli_args = [engine_params['launch_file'],
                    'name:=' + name]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()
        ...


class RosEnv(BaseRosEnv):

    def __init__(self, robots: List[Robot] = [], sensors: List[Sensor] = [], observers: List['Observer'] = [], **kwargs) -> None:
        # todo: Interface changes a lot, use **kwargs.
        #  Make arguments of subclass explicit when interface is more-or-less fixed.
        super().__init__(**kwargs)
        self.robots = robots
        self.sensors = sensors
        self.observers = observers

        self._init_nodes(self.robots, self.sensors, self.observers)

        self.observation_space, self.action_space = self._merge_spaces(self.robots, self.sensors, self.observers)
    
    def step(self, action: 'OrderedDict[str, object]') -> Tuple[object, float, bool, dict]:

        for robot in self.robots:
            robot.set_action(action[robot.name])

        self._step()

        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self._is_done(obs)
        return obs, reward, done, {}
    
    def reset(self) -> object:

        for robot in self.robots:
            robot.reset()

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

        for robot in self.robots:
            obs[robot.name] = robot.get_obs()

        for sensor in self.sensors:
            obs[sensor.name] = sensor.get_obs()

        for observer in self.observers:
            obs[observer.name] = observer.get_obs()

        return obs
    
    def _get_states(self) -> 'OrderedDict[str, object]':
        # How / what is it??
        pass

    def _get_reward(self, obs: 'OrderedDict[str, object]') -> float:
        # if needed:
        #states = self._get_states()
        return 0.0
    
    def _is_done(self, obs: 'OrderedDict[str, object]') -> bool:
        return False