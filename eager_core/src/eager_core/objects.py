import gym
import gym.spaces
from eager_core.utils.file_utils import load_yaml, substitute_xml_args
from eager_core.utils.gym_utils import (get_space_from_space_msg, get_message_from_space,
                                        get_def_from_space, get_response_from_space,
                                        get_space_from_def)
from eager_core.srv import RegisterActionProcessor, RegisterActionProcessorRequest
from std_srvs.srv import Empty
from eager_core.msg import Object as Object_msg
from collections import OrderedDict
from typing import Dict, List, Callable, Union
import rospy
import roslaunch
import rosservice
import numpy as np


class BaseRosObject():

    def __init__(self, type: str, name: str, **kwargs) -> None:
        self.type = type
        self.name = name
        self.args = str(kwargs)
        self._is_initialized = False
        self.topic_name = None

    def _infer_space(self, base_topic: str = '') -> gym.Space:
        pass

    def init_node(self, base_topic: str = '') -> None:
        raise NotImplementedError

    def get_topic(self, base_topic: str = '') -> str:
        if base_topic == '':
            self.topic_name = self.name
            return self.name
        self.topic_name = base_topic + '/' + self.name
        return base_topic + '/' + self.name

    def assert_not_yet_initialized(self, assert_type):
        if self._is_initialized:
            if assert_type == 'preprocess':
                str_err = '"%s" already initialized. Cannot add preprocessing after initialization.' % self.name
            elif assert_type == 'init':
                str_err = '"%s" already initialized. Hint: perhaps in another environment.' % self.name
            else:
                str_err = '"%s" already initialized. Cannot perform "%s" after initialization.' % (self.name, assert_type)
            rospy.logerr(str_err)
            raise Exception(str_err)
        else:
            return

    @property
    def is_initialized(self):
        return self._is_initialized


class Sensor(BaseRosObject):
    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.observation_space = space

    def init_node(self, base_topic: str = '') -> None:
        self.assert_not_yet_initialized('init')
        self._is_initialized = True

        if self.observation_space is None:
            self.observation_space = self._infer_space(base_topic)

        self._get_sensor_service = rospy.ServiceProxy(
            self.get_topic(
                base_topic + '/sensors'),
            get_message_from_space(
                self.observation_space))

    def get_obs(self) -> object:  # Type depends on space
        response = self._get_sensor_service()
        if self.observation_space.dtype == np.dtype(np.uint8):
            buf = np.frombuffer(response.value, np.uint8).reshape(self.observation_space.shape)
            return buf
        return np.array(response.value, dtype=self.observation_space.dtype).reshape(self.observation_space.shape)

    def add_preprocess(self, processed_space: gym.Space = None, launch_path='/path/to/custom/sensor_preprocess/ros_launchfile',
                       node_type='service', stateless=True) -> None:
        self.assert_not_yet_initialized('preprocess')

        self.observation_space = processed_space
        self.launch_path = launch_path
        self.node_type = node_type
        self.stateless = stateless

    def close(self):
        self._get_sensor_service.close()


class State(BaseRosObject):
    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.state_space = space
        self._has_warned = False

    def init_node(self, base_topic: str = '') -> None:
        self.assert_not_yet_initialized('init')
        self._is_initialized = True

        if self.state_space is None:
            self.state_space = self._infer_space(base_topic)

        self._buffer = None

        self._get_state_service = rospy.ServiceProxy(self.get_topic(base_topic + '/states'),
                                                     get_message_from_space(self.state_space))

        self._send_state_service = rospy.Service(
            self.get_topic(
                base_topic + '/resets'),
            get_message_from_space(
                self.state_space),
            self._send_state)

    def get_state(self) -> object:  # Type depends on space
        if self._get_state_service.resolved_name not in rosservice.get_service_list():
            if not self._has_warned:
                rospy.logwarn('State "{}" cannot be retrieved in this environment.'.format(self.name))
                self._has_warned = True
            return None
        response = self._get_state_service()
        if self.state_space.dtype == np.dtype(np.uint8):
            buf = np.frombuffer(response.value, np.uint8).reshape(self.state_space.shape)
            return buf
        return np.array(response.value, dtype=self.state_space.dtype).reshape(self.state_space.shape)

    def set_state(self, state: object) -> None:
        self._buffer = state

    def _send_state(self, request: object) -> object:
        msg_class = get_response_from_space(self.state_space)
        return msg_class(self._buffer)

    def close(self):
        self._get_state_service.close()
        self._send_state_service.shutdown()


class Actuator(BaseRosObject):

    def __init__(self, type: str, name: str, space: gym.Space = None) -> None:
        super().__init__(type, name)
        self.action_space = space
        self.preprocess_launch_args = None
        self.processor_action_space = None

    def init_node(self, base_topic: str = ''):
        self.assert_not_yet_initialized('init')
        self._is_initialized = True

        if self.action_space is None:
            self.action_space = self._infer_space(base_topic)

        if self.preprocess_launch_args is not None:
            # Launch processor
            cli_args = self.preprocess_launch_args
            cli_args.append('ns:={}'.format(self.get_topic(base_topic + '/actuators')))
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            self._preprocess_launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            self._preprocess_launch.start()

            # Register processor
            self._register_action_processor_service = rospy.ServiceProxy(self.get_topic(
                base_topic + '/actuators') + "/register_processor", RegisterActionProcessor)
            self._register_action_processor_service.wait_for_service()
            response = self._register_action_processor_service(self.preprocess_req)

            # Close service
            self._register_action_processor_service = rospy.ServiceProxy(self.get_topic(
                base_topic + '/actuators') + "/close_processor", Empty)

            if self.processor_action_space is not None:
                self.action_space = self.processor_action_space
            else:
                self.action_space = get_space_from_space_msg(response.action_space)

            # Initialize buffer
            self._buffer = self.action_space.sample()

            # Create act service
            self._act_service = rospy.Service(
                self.get_topic(
                    base_topic + '/actuators') + "/raw",
                get_message_from_space(
                    self.action_space),
                self._send_action)
        else:
            self._buffer = self.action_space.sample()

            self._act_service = rospy.Service(
                self.get_topic(
                    base_topic + '/actuators'),
                get_message_from_space(
                    self.action_space),
                self._send_action)

    def set_action(self, action: object) -> None:
        self._buffer = action

    def add_preprocess(self, processor, observations_from_objects: list = [],
                       action_space: gym.Space = None) -> None:
        self.assert_not_yet_initialized('preprocess')
        launch_file = processor.launch_file
        launch_args = processor.__dict__
        cli_args = [substitute_xml_args(launch_file)]
        for key, value in launch_args.items():
            cli_args.append('{}:={}'.format(key, value))
        self.preprocess_launch_args = cli_args

        observation_objects = []
        for object in observations_from_objects:
            object_msg = Object_msg()
            object_msg.type = object.type
            object_msg.name = object.name
            observation_objects.append(object_msg)

        req = RegisterActionProcessorRequest()
        if action_space is not None:
            req.raw_action_type = get_def_from_space(action_space)['type']
        req.action_type = get_def_from_space(self.action_space)['type']
        req.observation_objects = observation_objects
        self.preprocess_req = req

        if action_space is not None:
            self.processor_action_space = action_space

    def reset(self) -> None:
        pass

    def _send_action(self, request: object) -> object:
        msg_class = get_response_from_space(self.action_space)
        return msg_class(self._buffer)

    def close(self):
        self._act_service.shutdown()
        if self.preprocess_launch_args:
            self._register_action_processor_service()
            self._preprocess_launch.shutdown()


class Object(BaseRosObject):
    """
    Object for use in the EAGER environment.

    Objects can have a observation, action and state space.
    The constructor of Objects is usually not called directly.
    To create a new object use :func:`eager_core.objects.Object.create`.

    :param type: The object type
    :param name: The name of this object, must be unique
    :param sensors: A list or dict of sensors in this object
    :param actuators: A list or dict of actuators in this object
    :param states: A list or dict of sensors in this object
    :param position: The initial position of the object in the world
    :param orientation: The initial orientation of the object in the world (quaternion xyzw)
    :param fixed_base: Fix the base position and orientation (for manipulators)
    :param self_collision: Collide with own collision boxes (for manipulators)
    """

    def __init__(self, type: str, name: str,
                 sensors: Union[List[Sensor], Dict[str, Sensor]],
                 actuators: Union[List[Actuator], Dict[str, Actuator]],
                 states: Union[List[State], Dict[str, State]],
                 reset: Callable[['Object'], None] = None,
                 position: List[float] = [0, 0, 0],
                 orientation: List[float] = [0, 0, 0, 1],
                 fixed_base: bool = True,
                 self_collision: bool = True,
                 **kwargs
                 ) -> None:
        super().__init__(type, name, position=position, orientation=orientation,
                         fixed_base=fixed_base, self_collision=self_collision, **kwargs)

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
    def create(cls, name: str, package_name: str, object_type: str,
               position: List[float] = [0, 0, 0],
               orientation: List[float] = [0, 0, 0, 1],
               fixed_base: bool = True,
               self_collision: bool = True,
               **kwargs
               ) -> 'Object':
        """
        Creates an Object for use in the EAGER environment.

        Objects can have a observation, action and state space.
        The function will read the config of ``object_type`` in ``package_name``
        and create this type with handle ``name``.

        :param name: The name of this object, must be unique
        :param package_name: The name of the package to load this object from
        :param object_type: The object type in the package
        :param position: The initial position of the object in the world
        :param orientation: The initial orientation of the object in the world (quaternion xyzw)
        :param fixed_base: Fix the base position and orientation (for manipulators)
        :param self_collision: Collide with own collision boxes (for manipulators)
        :return: An uninitialized Object with name ``name``
        """

        params = load_yaml(package_name, object_type)

        sensors = []
        if 'sensors' in params:
            for sens_name, sensor in params['sensors'].items():
                sensor_space = get_space_from_def(sensor)
                sensors.append(Sensor(None, sens_name, sensor_space))

        actuators = []
        if 'actuators' in params:
            for act_name, actuator in params['actuators'].items():
                act_space = get_space_from_def(actuator)
                actuators.append(Actuator(None, act_name, act_space))

        states = []
        if 'states' in params:
            for state_name, state in params['states'].items():
                state_space = get_space_from_def(state)
                states.append(State(None, state_name, state_space))

        return cls(package_name + '/' + object_type, name, sensors, actuators, states,
                   position=position, orientation=orientation, fixed_base=fixed_base, self_collision=self_collision, **kwargs)

    def init_node(self, base_topic: str = '') -> None:
        self.assert_not_yet_initialized('init')
        self._is_initialized = True

        for sensor in self.sensors.values():
            sensor.init_node(self.get_topic(base_topic))

        for actuator in self.actuators.values():
            actuator.init_node(self.get_topic(base_topic))

        for state in self.states.values():
            state.init_node(self.get_topic(base_topic))

    def set_action(self, actions: 'OrderedDict[str, object]') -> None:  # Error return?

        for act_name, actuator in self.actuators.items():
            actuator.set_action(actions[act_name])

    def get_obs(self, sensors: List[str] = None) -> 'OrderedDict[str, object]':
        if sensors is not None:
            obs = OrderedDict([(sensor, self.sensors[sensor].get_obs()) for sensor in sensors])
        else:
            obs = OrderedDict([(sens_name, sensor.get_obs()) for sens_name, sensor in self.sensors.items()])
        return obs

    def get_state(self, states: List[str] = None) -> 'OrderedDict[str, object]':
        state_dict = dict()
        if states is not None:
            for state in states:
                state_val = self.states[state].get_state()
                if state_val is not None:
                    state_dict[state] = state_val
        else:
            for state_name, state in self.states.items():
                state_val = self.states[state_name].get_state()
                if state_val is not None:
                    state_dict[state_name] = state_val
        return state_dict

    @property
    def action_space(self) -> gym.spaces.Dict:
        if not self.actuators:
            return None
        spaces = OrderedDict()
        for act_name, actuator in self.actuators.items():
            spaces[act_name] = actuator.action_space

        return gym.spaces.Dict(spaces=spaces)

    @property
    def observation_space(self) -> gym.spaces.Dict:
        if not self.sensors:
            return None
        spaces = OrderedDict()
        for sens_name, sensor in self.sensors.items():
            spaces[sens_name] = sensor.observation_space

        return gym.spaces.Dict(spaces=spaces)

    @property
    def state_space(self) -> gym.spaces.Dict:
        if not self.states:
            return None
        spaces = OrderedDict()
        for state_name, state in self.states.items():
            spaces[state_name] = state.state_space

        return gym.spaces.Dict(spaces=spaces)

    def reset(self, states: 'OrderedDict[str, object]') -> None:  # Error return?
        for actuator in self.actuators.values():
            actuator.reset()

        for state_name, state in states.items():
            self.states[state_name].set_state(state)

        if self.reset_func is not None:
            self.reset_func(self)

    def close(self):
        for sensor in self.sensors.values():
            sensor.close()
        for actuator in self.actuators.values():
            actuator.close()
        for state in self.states.values():
            state.close()
