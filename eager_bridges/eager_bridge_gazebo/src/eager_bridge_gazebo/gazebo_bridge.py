import rospy
import roslaunch
import functools
import sensor_msgs.msg
import rosservice
from operator import add
from inspect import getmembers, isclass, isroutine
import eager_core.action_server
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_gazebo.orientation_utils import quaternion_multiply, euler_from_quaternion
from gazebo_msgs.srv import (GetPhysicsProperties, GetPhysicsPropertiesRequest, SetPhysicsProperties,
                             SetPhysicsPropertiesRequest, SetModelState, SetModelStateRequest, GetModelState,
                             GetModelStateRequest)
from std_srvs.srv import Empty
from controller_manager_msgs.srv import ListControllers, ListControllersRequest, SwitchController, SwitchControllerRequest
from eager_bridge_gazebo.srv import SetInt, SetIntRequest, SetJointState, SetJointStateRequest
from geometry_msgs.msg import Point, Quaternion
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def
from eager_core.srv import ResetEnvResponse
from eager_core.msg import ObjectStates, State
import time


class GazeboBridge(PhysicsBridge):

    def __init__(self):
        self.stepped = False
        self.ready = False

        self._start_simulator()

        step_time = rospy.get_param('physics_bridge/time_step', 0.1)

        physics_parameters_service = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        physics_parameters_service.wait_for_service()

        physics_parameters = physics_parameters_service(GetPhysicsPropertiesRequest())

        new_physics_parameters = SetPhysicsPropertiesRequest()
        new_physics_parameters.time_step = rospy.get_param('physics_bridge/solver_time_step', 0.001)
        new_physics_parameters.max_update_rate = rospy.get_param('physics_bridge/solver_max_update_rate', 0.0)
        new_physics_parameters.gravity = physics_parameters.gravity
        new_physics_parameters.ode_config = physics_parameters.ode_config

        set_physics_properties_service = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics_properties_service.wait_for_service()

        set_physics_properties_service(new_physics_parameters)

        self._get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._get_model_state.wait_for_service()
        self._set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self._set_model_state.wait_for_service()

        self._set_joint_state = rospy.ServiceProxy('/gazebo/set_joint_state', SetJointState)
        self._set_joint_state.wait_for_service()

        self._pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._pause_physics.wait_for_service()

        self._unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self._unpause_physics.wait_for_service()

        self._step_world = rospy.ServiceProxy('/gazebo/step_world', SetInt)
        self._step_world.wait_for_service()

        self.step_request = SetIntRequest(int(round(step_time / physics_parameters.time_step)))

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()
        self._action_servers = dict()

        self._state_buffer = dict()
        self._state_subscribers = []
        self._state_services = []
        self._get_state_services = []

        self._reset_services = dict()
        self.controller_list = []

        self._remap_publishers = dict()

        self.switch_controller_request = SwitchControllerRequest()
        self.switch_controller_request.start_asap = True
        self.switch_controller_request.strictness = 1

        super(GazeboBridge, self).__init__("gazebo")

    def _start_simulator(self):
        str_launch_sim = '$(find eager_bridge_gazebo)/launch/gazebo_sim.launch'
        cli_args = [substitute_xml_args(str_launch_sim),
                    'gui:=%s' % rospy.get_param('physics_bridge/gui', False),
                    'world:=%s' % rospy.get_param('physics_bridge/world')]
        seed = rospy.get_param('physics_bridge/seed', None)
        if seed is not None:
            cli_args.append('extra_gazebo_args:=--seed %d' % seed)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        self._launch.start()

    def _register_object(self, topic, name, package, object_type, args, config):
        if self.stepped:
            rospy.logwarn("Training has started, cannot add {}".format(name))
            return False

        self._add_robot(topic, name, package, args, config)

        if 'sensors' in config:
            self._init_sensors(topic, name, config['sensors'])

        if 'actuators' in config:
            self._init_actuators(topic, name, config['actuators'])

        if 'states' in config:
            self._init_states(topic, name, config['states'])
            self._init_resets(topic, name, config['states'])
        return True

    def _add_robot(self, topic, name, package, args, config):
        if self.stepped:
            rospy.logwarn('[{}] Cannot add robot {}, because training has started!'.format(rospy.get_name(), name))
        else:
            self.ready = False
            if 'default_translation' in config:
                pos = "-x {:.2f} -y {:.2f} -z {:.2f}".format(*map(add, config['default_translation'], args['position']))
            else:
                pos = "-x {:.2f} -y {:.2f} -z {:.2f}".format(*args['position'])
            if 'default_orientation' in config:
                ori = "-R {:.2f} -P {:.2f} -Y {:.2f}".format(
                    *euler_from_quaternion(*quaternion_multiply(config['default_orientation'], args['orientation'])))
            else:
                ori = "-R {:.2f} -P {:.2f} -Y {:.2f}".format(*euler_from_quaternion(*args['orientation']))
            str_launch_object = '$(find %s)/launch/gazebo.launch' % package
            cli_args = [substitute_xml_args(str_launch_object),
                        'ns:=%s' % topic,
                        'name:=%s' % name,
                        'configuration:=%s' % (pos + " " + ori)]
            if package == "eager_solid_other":
                cli_args.append('model_name:=%s' % config['model_name'])
            if 'fixed_base' in args:
                if package == "eager_solid_other":
                    rospy.logwarn("Cannot process fixed base argument for solids.".format())
                else:
                    cli_args.append('fixed_base:=%s' % args['fixed_base'])
            if 'self_collision' in args:
                if package == "eager_solid_other":
                    rospy.logwarn("Cannot process self_collision argument for solids.".format())
                else:
                    cli_args.append('self_collision:=%s' % args['self_collision'])
            roslaunch_args = cli_args[1:]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            launch.start()
            self.ready = True

        # Wait for model to be present
        time.sleep(2)

    def _init_sensors(self, topic, name, sensors):
        self._sensor_buffer[name] = {}
        self._remap_publishers[name] = {}
        for sensor in sensors:
            rospy.logdebug("Initializing sensor {}".format(sensor))
            sensor_params = sensors[sensor]
            msg_topic = sensor_params['topic']
            if not msg_topic.startswith('/'):
                msg_topic = topic + "/" + sensor_params["topic"]
            msg_name = sensor_params["msg_name"]
            space = sensor_params['space']
            valid_msgs = [i[0] for i in getmembers(sensor_msgs.msg) if isclass(i[1])]
            if msg_name in valid_msgs:
                msg_type = getattr(sensor_msgs.msg, msg_name)
            else:
                rospy.logerror("Sensor message {} does not exist. Valid messages are: {}".format(msg_name, valid_msgs))
            attribute_name = sensor_params["type"]
            valid_attributes = [i[0] for i in getmembers(msg_type) if not isroutine(i[1])]
            if attribute_name in valid_attributes:
                attribute = attribute_name
            else:
                rospy.logerror("Sensor message {} does not have an attribute named {}. Valid attributes are: {}".format(
                    msg_name, attribute_name, valid_attributes))
            entries = None
            if 'entries' in sensor_params:
                entries = sensor_params['entries']
            self._sensor_buffer[name][sensor] = [get_value_from_def(space)] * get_length_from_def(space)
            if entries:
                self._sensor_subscribers.append(rospy.Subscriber(
                    msg_topic,
                    msg_type,
                    functools.partial(
                        self._sensor_entries_callback, name=name, sensor=sensor, attribute=attribute, entries=entries))
                    )
            else:
                self._remap_publishers[name][sensor] = None
                if 'remap' in sensor_params:
                    if sensor_params['remap']:
                        self._remap_publishers[name][sensor] = rospy.Publisher(
                            topic + "/sensors/" + sensor, msg_type, queue_size=1)
                self._sensor_subscribers.append(rospy.Subscriber(
                    msg_topic,
                    msg_type,
                    functools.partial(
                        self._sensor_callback,
                        name=name,
                        sensor=sensor,
                        attribute=attribute)))
            self._sensor_services.append(rospy.Service(topic + "/sensors/" + sensor, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._sensor_buffer,
                                                                         name=name, obs_name=sensor,
                                                                         message_type=get_response_from_def(space)
                                                                         )))

    def _init_actuators(self, topic, name, actuators):
        self._actuator_services[name] = {}
        self._action_servers[name] = {}
        for actuator in actuators:
            rospy.logdebug("Initializing actuator {}".format(actuator))
            names = actuators[actuator]["names"]
            space = actuators[actuator]['space']
            server_name = topic + "/" + actuators[actuator]["server_name"]
            action_server_name = actuators[actuator]["action_server"]
            valid_servers = [i[0] for i in getmembers(eager_core.action_server) if isclass(i[1])]
            if action_server_name in valid_servers:
                action_server = getattr(eager_core.action_server, action_server_name)
                self._action_servers[name][actuator] = action_server(names, server_name)
            else:
                rospy.logerror("Action server {} not implemented. Valid action servers are: {}".format(
                    action_server_name, valid_servers))
            get_action_srv = rospy.ServiceProxy(topic + "/actuators/" + actuator, get_message_from_def(space))
            set_action_srv = self._action_servers[name][actuator].act
            reset_action_srv = self._action_servers[name][actuator].reset
            self._actuator_services[name][actuator] = (get_action_srv, set_action_srv, reset_action_srv)

    def _init_states(self, topic, name, states):
        self._state_buffer[name] = {}
        for state_name in states:
            rospy.logdebug("Initializing state {}".format(state_name))
            state = states[state_name]
            space = state['space']
            self._state_buffer[name][state_name] = [get_value_from_def(space)] * get_length_from_def(space)
            if state['type'] == 'joint_pos' or state['type'] == 'joint_vel':
                if 'topic' in state:
                    msg_topic = state['topic']
                    if not msg_topic.startswith('/'):
                        msg_topic = topic + '/' + msg_topic
                else:
                    msg_topic = topic + '/joint_states'
                entries = None
                if 'entries' in state:
                    entries = state['entries']
                msg_type = sensor_msgs.msg.JointState
                if state['type'] == 'joint_pos':
                    attribute = 'position'
                elif state['type'] == 'joint_vel':
                    attribute = 'velocity'
                if entries:
                    self._state_subscribers.append(rospy.Subscriber(
                        msg_topic,
                        msg_type,
                        functools.partial(
                            self._state_entries_callback, name=name, state=state_name, attribute=attribute, entries=entries)))
                else:
                    self._state_subscribers.append(rospy.Subscriber(
                        msg_topic,
                        msg_type,
                        functools.partial(
                            self._state_callback, name=name, state=state_name, attribute=attribute)))
            self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state_name,
                                                                         message_type=get_response_from_def(space)
                                                                         )))

    def _sensor_callback(self, data, name, sensor, attribute):
        if self._remap_publishers[name][sensor]:
            self._remap_publishers[name][sensor].publish(data)
        data_list = getattr(data, attribute)
        self._sensor_buffer[name][sensor] = data_list

    def _sensor_entries_callback(self, data, name, sensor, attribute, entries):
        data_list = getattr(data, attribute)
        self._sensor_buffer[name][sensor] = list(map(data_list.__getitem__, entries))

    def _state_callback(self, data, name, state, attribute):
        data_list = getattr(data, attribute)
        self._state_buffer[name][state] = data_list

    def _state_entries_callback(self, data, name, state, attribute, entries):
        data_list = getattr(data, attribute)
        self._state_buffer[name][state] = list(map(data_list.__getitem__, entries))

    def _init_resets(self, topic, name, states):
        self._reset_services[name] = dict()
        for state_name in states:
            state = states[state_name]
            space = state['space']
            get_state_srv = None
            if 'joint' in state['type']:
                self._step_world(SetIntRequest(1))
                # Controllers should be switched off before setting the joint states
                controller_list_topics = [x for x in rosservice.get_service_list() if (
                    topic in x and x.endswith('/controller_manager/list_controllers')
                    )]
                for controller_list_topic in controller_list_topics:
                    controller_list_service = rospy.ServiceProxy(controller_list_topic, ListControllers)
                    controller_list_service.wait_for_service()
                    self._step_world(SetIntRequest(1))
                    controller_list = controller_list_service(ListControllersRequest())
                    controller_names = []
                    for controller in controller_list.controller:
                        controller_names.append(controller.name)
                    if len(controller_names) > 0:
                        controller_switch_topic = controller_list_topic[:-16] + 'switch_controller'
                        self.controller_list.append(
                            {'service': rospy.ServiceProxy(controller_switch_topic, SwitchController),
                             'names': controller_names,
                             }
                                                    )
                if state['type'] == 'joint_pos':
                    request = SetJointStateRequest()
                    request.model_name = name
                    request.joint_names = state['names']

                    def set_reset_srv(value, request):
                        request.joint_positions = value
                        return self._set_joint_state(request)
                    set_reset_srv = functools.partial(set_reset_srv, request=request)
                elif state['type'] == 'joint_vel':
                    request = SetJointStateRequest()
                    request.model_name = name
                    request.joint_names = state['names']

                    def set_reset_srv(value, request):
                        rospy.logwarn("Reset of joint velocities is not tested yet!")
                        request.joint_velocities = value
                        return self._set_joint_state(request)
                    set_reset_srv = functools.partial(set_reset_srv, request=request)
            elif 'link' in state['type']:
                raise ValueError('State type ("%s") cannot be reset in Gazebo.' % states[state]['type'])
            elif 'base' in state['type']:
                set_request = SetModelStateRequest()
                get_request = GetModelStateRequest()

                set_request.model_state.model_name = name
                get_request.model_name = name
                if state['type'] == 'base_pos':
                    def set_reset_srv(value, get_request, set_request):
                        response = self._get_model_state(get_request)
                        set_request.model_state.pose.position = Point(*value)
                        set_request.model_state.pose.orientation = response.pose.orientation
                        return self._set_model_state(set_request)
                    set_reset_srv = functools.partial(set_reset_srv, get_request=get_request, set_request=set_request)

                    def get_state_srv(request, name, state_name):
                        response = self._get_model_state(request)
                        position = [response.pose.position.x, response.pose.position.y, response.pose.position.z]
                        self._state_buffer[name][state_name] = position
                    get_state_srv = functools.partial(get_state_srv, request=get_request, name=name, state_name=state_name)
                elif state['type'] == 'base_orientation':
                    def set_reset_srv(value, get_request, set_request):
                        response = self._get_model_state(get_request)
                        set_request.model_state.pose.position = response.pose.position
                        set_request.model_state.pose.orientation = Quaternion(*value)
                        return self._set_model_state(set_request)
                    set_reset_srv = functools.partial(set_reset_srv, get_request=get_request, set_request=set_request)

                    def get_state_srv(request, name, state_name):
                        response = self._get_model_state(request)
                        orientation = [response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z,
                                       response.pose.orientation.w]
                        self._state_buffer[name][state_name] = orientation
                    get_state_srv = functools.partial(get_state_srv, request=get_request, name=name, state_name=state_name)
            else:
                raise ValueError('State type ("%s") must contain "{joint, base}".' % states[state]['type'])
            get_reset_srv = rospy.ServiceProxy(topic + "/resets/" + state_name, get_message_from_def(space))
            self._reset_services[name][state_name] = {'get': get_reset_srv, 'set': set_reset_srv}
            if get_state_srv is not None:
                self._get_state_services.append(get_state_srv)

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        if not self.stepped:
            self._step_world(SetIntRequest(1))
            self.stepped = True
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv, reset_action_srv) = self._actuator_services[robot][actuator]
                actions = get_action_srv()
                set_action_srv(actions.value)
        self._step_world(self.step_request)
        # Get states from services
        for get_state_service in self._get_state_services:
            get_state_service()
        return True

    def _reset(self, req):
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv, reset_action_srv) = self._actuator_services[robot][actuator]
                reset_action_srv()
        # First we switch off all controllers
        for controller in self.controller_list:
            service = controller['service']
            names = controller['names']
            self.switch_controller_request.stop_controllers = names
            self._unpause_physics()
            service(self.switch_controller_request)
            self._pause_physics()
        # Now we can perform the reset
        response = ResetEnvResponse()
        failed_resets = []
        for object in req.objects:
            object_name = object.name
            if object_name not in self._reset_services:
                break
            failed_states = ObjectStates()
            failed_states.name = object_name
            states = object.states
            for state in states:
                state_name = state.name
                if state_name not in self._reset_services[object_name]:
                    break
                reset_srvs = self._reset_services[object_name][state_name]
                state = reset_srvs['get']()
                if state.value:
                    if 'check' in reset_srvs:
                        validity_response = reset_srvs['check'](state.value)
                        if not validity_response.data:
                            failed_state = State()
                            failed_state.name = state_name
                            failed_states.states.append(failed_state)
                        else:
                            reset_srvs['set'](list(state.value))
                    else:
                        reset_srvs['set'](list(state.value))
            failed_resets.append(failed_states)
        response.objects = failed_resets
        # Now we need to turn back on the controllers
        for controller in self.controller_list:
            service = controller['service']
            names = controller['names']
            self.switch_controller_request.start_controllers = names
            self._unpause_physics()
            service(self.switch_controller_request)
            self._pause_physics()
        self.stepped = False
        return response

    def _close(self):
        self._launch.shutdown()
        return True

    def _seed(self, seed):
        pass
