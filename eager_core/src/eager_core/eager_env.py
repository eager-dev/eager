import gym
import gym.spaces
import rospy
import roslaunch
import rosparam
from gym.utils import seeding
from collections import OrderedDict
from typing import List, Tuple, Callable, Optional
from eager_core.objects import Object, Sensor
from eager_core.srv import StepEnv, ResetEnv, ResetEnvRequest, Register
from eager_core.utils.file_utils import substitute_xml_args, is_namespace_empty, launch_node
from eager_core.msg import Seed, Object as ObjectMsg
from eager_core.engine_params import EngineParams
from eager_core.msg import ObjectStates, State



class BaseEagerEnv(gym.Env):
    """
    Base Gym Env to interact with

    :param engine: The physics engine to run this environment on
    :param name: The namespace to run this environment in (must by unique if using multiple environments simultaniously)
    """

    def __init__(self, engine: EngineParams, name: str = 'ros_env') -> None:
        super().__init__()

        self._initialize_physics_bridge(engine, name)

        self.name = name

        self._step = rospy.ServiceProxy(name + '/step', StepEnv)
        self._reset = rospy.ServiceProxy(name + '/reset', ResetEnv)
        self.__seed_publisher = rospy.Publisher(name + '/seed', Seed, queue_size=1)

    def _init_nodes(self, objects: List[Object] = [], observers: List['Observer'] = []) -> None:
        """
        Initializes and registers a list of objects and observers in the physics engine.

        Must be called before retreiving the observation, action and state spaces.

        :param objects: A list of objects to initialize
        :param observers: A list of observers to initialize
        """
        # Verify that object name is unique (i.e. no topics/services already defined within namespace)
        obj_names = [obj.name for obj in objects]
        for obj in objects:
            ns = '/%s/objects/%s' % (self.name, obj.name)
            is_unique = is_namespace_empty(ns) and 1 == len([name for name in obj_names if obj.name is name])
            if not is_unique:
                str_err = 'Cannot register object "%s", because object name not unique (namespace non-empty).' % obj.name
                rospy.logerr(str_err)
                raise Exception(str_err)

        # Verify that observer name is unique (i.e. no topics/services already defined within namespace)
        obs_names = [obs.name for obs in observers]
        for obs in observers:
            ns = '/%s/observers/%s' % (self.name, obs.name)
            is_unique = is_namespace_empty(ns) and 1 == len([name for name in obs_names if obs.name is name])
            if not is_unique:
                str_err = 'Cannot register observer "%s", because observer name not unique (namespace non-empty).' % obs.name
                rospy.logerr(str_err)
                raise Exception(str_err)

        self._register_objects(objects, observers)
        self._init_listeners(objects, observers)

    def _register_objects(self, objects: List[Object] = [], observers: List['Observer'] = []) -> None:
        """
        Registers objects to the physics engine.

        Typically not called directly but through _init_nodes.

        :param objects: A list of objects to register
        :param observers: A list of observers to register

        """
        objects_reg = []

        for el in (objects, observers):
            for object in el:
                objects_reg.append(ObjectMsg(object.type, object.name, object.args))

        register_service = rospy.ServiceProxy(self.name + '/register', Register)
        register_service.wait_for_service(30)
        register_service(objects_reg)

    def _init_listeners(self, objects: List[Object] = [], observers: List['Observer'] = []) -> None:
        """
        Initializes listening to observation topics in objects and observers.

        Called after registration.
        Typically not called directly but through _init_nodes.

        :param objects: A list of objects to register
        :param observers: A list of observers to register

        """
        bt = self.name + '/objects'

        for el in (objects, observers):
            for object in el:
                object.init_node(bt)

    def _merge_spaces(cls, objects: List[Object] = [], observers: List['Observer'] = []
                      ) -> Tuple[gym.spaces.Dict, gym.spaces.Dict, gym.spaces.Dict]:
        """
        Merges the observation, action and state space of lists of objects and observers.

        Can only be called after nodes are initialized.

        :param objects: A list of objects to merge
        :param observers: A list of observers to merge
        :return: Observation space, action space, state space

        """
        # Make sure all objects & observers are initialized
        for el in (objects, observers):
            for e in el:
                if not e.is_initialized:
                    str_err = '"%s" not yet initialized. Can only merge spaces after "%s" is initialized.' % (e.name, e.name)
                    rospy.logerr(str_err)
                    raise Exception(str_err)

        obs_spaces = OrderedDict()
        act_spaces = OrderedDict()
        state_spaces = OrderedDict()

        for object in objects:
            if object.observation_space:
                obs_spaces[object.name] = object.observation_space
            if object.action_space:
                act_spaces[object.name] = object.action_space
            if object.state_space:
                state_spaces[object.name] = object.state_space

        for observer in observers:
            obs_spaces[observer.name] = observer.observation_space

        return gym.spaces.Dict(spaces=obs_spaces), gym.spaces.Dict(spaces=act_spaces), gym.spaces.Dict(spaces=state_spaces)

    def _initialize_physics_bridge(self, engine: EngineParams, name: str) -> None:
        """
        Starts the physics bridge.

        Called by the constructor, do not call directly.

        :param engine: The chosen physics engine
        :param name: The namespace to start the engine in
        """
        rosparam.upload_params('%s/physics_bridge' % name, engine.__dict__)

        # Launch the physics bridge under the namespace 'name'
        launch_file = substitute_xml_args(engine.launch_file)
        self._launch = launch_node(launch_file, args=['name:=' + name])
        self._launch.start()

    def close(self, objects: List[Object] = [], observers: List['Observer'] = []) -> None:
        """
        Closes and cleans up the environment.

        Will shutdown the physics engine and disable all passed node listeners.

        :param objects: A list of objects to close
        :param observers: A list of observers to close
        """
        self._launch.shutdown()

        for el in (objects, observers):
            for object in el:
                object.close()

        try:
            rosparam.delete_param('/%s' % self.name)
            rospy.loginfo('Pre-existing parameters under namespace "/%s" deleted.' % self.name)
        except BaseException:
            pass

    def seed(self, seed: int = None) -> List[int]:
        """
        Seeds the physics bridge and any processors where possible.

        :param seed: A seed to use in the physics bridge and processors
        :return: Currently only returns the given seed
        """
        # todo: does this actually seed e.g. numpy on the EagerEnv side?
        seed = seeding.create_seed(seed)
        self.__seed_publisher.publish(seed)
        return [seed]


class EagerEnv(BaseEagerEnv):
    """
    A simple fully functional gym environment for multiple physics engines.

    Will merge the observation, action and state space of all given objects.

    :param engine: The physics engine to run this environment on
    :param objects: The objects to include in this environment
    :param observers: The observers to include in this environment
    :param name: The namespace to run this environment in (must by unique if using multiple environments simultaniously)
    :param render_sensor: A reference to the :func:`eager_core.objects.Sensor` object of the observation to be used
        in the :func:`render` function
    :param max_steps: The amount of steps per rollout, overridden if ``is_done_fn`` is set
    :param reward_fn: The reward function of this environment. Takes the environment and observations and returns a reward
    :param is_done_fn: Function to indicate this environment is done and should reset. Takes the environment and observations
        and returns a reward
    :param reset_fn: Function that sets the states of objects in the world when resetting
    """

    def __init__(self, engine: EngineParams, objects: List[Object] = [], observers: List['Observer'] = [],
                 name: str = 'ros_env', render_sensor: Sensor = None, max_steps: int = None,
                 reward_fn: Callable[['EagerEnv', 'OrderedDict[str, object]'], float] = None,
                 is_done_fn: Callable[['EagerEnv', 'OrderedDict[str, object]'], bool] = None,
                 reset_fn: Callable[['EagerEnv'], None] = None) -> None:

        super().__init__(engine, name)
        self.objects = objects
        self.observers = observers
        self.render_sensor = render_sensor
        self.render_init = False

        # Overwrite the _get_reward() function if provided
        if reward_fn:
            self._get_reward = reward_fn

        # Overwrite the _is_done() function if provided
        if is_done_fn:
            self._is_done = is_done_fn

        if reset_fn:
            self._reset_fn = reset_fn
        else:
            self._reset_fn = self._random_reset

        self.STEPS_PER_ROLLOUT = max_steps
        self.steps = 0

        self._init_nodes(self.objects, self.observers)

        self.observation_space, self.action_space, self.state_space = self._merge_spaces(self.objects, self.observers)

    def step(self, action: 'OrderedDict[str, object]') -> Tuple[object, float, bool, dict]:
        """
        Steps the environment

        :param action: A dictionary of actions to perform
        :return: observations, reward, done, states
        """
        self.steps += 1

        for object in self.objects:
            if object.action_space:
                object.set_action(action[object.name])

        self._step()

        state = self._get_states()
        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self._is_done(obs)
        return obs, reward, done, {'state': state}

    def reset(self) -> 'OrderedDict[str, object]':
        """
        Resets the environment by first calling the objects reset function and then resetting the environment

        :return: observations
        """
        self.steps = 0
        states_to_reset = self.state_space.sample()
        done = False
        while not done:
            reset_dict = self._reset_fn(self)
            for obj_name, object in reset_dict.items():
                for state_name, state in list(object.items()):
                    if (obj_name not in states_to_reset) or (state_name not in states_to_reset[obj_name]):
                        reset_dict[obj_name].pop(state_name)
            for object in self.objects:
                if object.name in reset_dict:
                    object.reset(reset_dict[object.name])
            request = ResetEnvRequest()
            for obj_name, state_dict in reset_dict.items():
                object_states = ObjectStates(name=obj_name)
                for state in state_dict.keys():
                    object_state = State(name=state)
                    object_states.states.append(object_state)
                request.objects.append(object_states)
            response = self._reset(request)

            done = True
            states_to_reset = dict()
            for object in response.objects:
                states_to_reset[object.name] = {}
                for state in object.states:
                    done = False
                    states_to_reset[object.name][state.name] = None
        return self._get_obs()

    def render(self, mode: str = 'human') -> Optional[object]:
        """
        Produces a render image using the provided render_sensor in EagerEnv's constructor.

        :param mode: The rendering mode, currently not used
        :return: The observation given in ``render_sensor`` in the constructor, None if not set
        """
        if self.render_sensor:
            if mode == 'human' and not self.render_init:
                # todo: launch render node with address, render rate and environment name as args
                launch_file = '$(find eager_core)/launch/render.launch'
                topic_name = self.render_sensor.topic_name
                self.render_node = launch_node(launch_file, args=['name:=' + self.name,
                                                                  'topic_name:=' + topic_name,
                                                                  'rate:=' + str(20)])
                self.render_node.start()
                self.render_init = True
            return self.render_sensor.get_obs()
        else:
            return None

    def _get_obs(self) -> 'OrderedDict[str, object]':
        """
        Gets the observations of all objects and observers in this environment

        :return: observations
        """
        obs = OrderedDict()

        for object in self.objects:
            if object.observation_space:
                obs[object.name] = object.get_obs()

        for observer in self.observers:
            obs[observer.name] = observer.get_obs()

        return obs

    def _get_states(self) -> 'OrderedDict[str, object]':
        """
        Gets the states of all objects and observers in this environment

        :return: states
        """
        state = OrderedDict()

        for object in self.objects:
            if object.state_space:
                state[object.name] = object.get_state()

        return state

    def _get_reward(self, obs: 'OrderedDict[str, object]') -> float:
        """
        The reward function of this environment.

        Is zero by default, must be set in the costructor with ``reward_fn`` or overridden

        :param obs: The observations of the current time step
        :return: reward
        """
        return 0.0

    def _is_done(self, obs: 'OrderedDict[str, object]') -> bool:
        """
        Signals if the environment should reset after this step

        By default returns true after ``max_steps`` if set and can be set in costructor
        by setting ``is_done_fn`` or can be overridden

        :param obs: The observations of the current time step
        :return: done
        """
        if self.STEPS_PER_ROLLOUT and self.STEPS_PER_ROLLOUT > 0:
            return self.steps >= self.STEPS_PER_ROLLOUT
        else:
            return False

    def _random_reset(self, env):
        reset_dict = env.state_space.sample()
        return reset_dict

    def close(self) -> None:
        """
        Closes and cleans up the environment.

        Will shutdown the physics engine and disable all passed node listeners.
        """
        if self.render_init:
            self.render_node.shutdown()
        super().close(objects=self.objects, observers=self.observers)
