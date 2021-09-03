import rospy
import functools
import sensor_msgs.msg
import importlib
from inspect import getmembers, isclass, isroutine
import eager_core.action_server
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def
from eager_core.srv import ResetEnvResponse
from eager_core.msg import ObjectStates, State


class RealBridge(PhysicsBridge):

    def __init__(self):
        action_rate = rospy.get_param('physics_bridge/action_rate', 30)
        self.rate = rospy.Rate(action_rate)

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

        super(RealBridge, self).__init__("real")

    def _register_object(self, topic, name, package, object_type, args, config):
        if 'sensors' in config:
            self._init_sensors(topic, name, config['sensors'])

        if 'actuators' in config:
            self._init_actuators(topic, name, config['actuators'])

        if 'states' in config:
            self._init_states(topic, name, config['states'])
            self._init_resets(topic, name, config['states'])
        return True

    def _init_sensors(self, topic, name, sensors):
        self._sensor_buffer[name] = {}
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
                self._sensor_subscribers.append(rospy.Subscriber(
                    msg_topic,
                    msg_type,
                    functools.partial(self._sensor_callback, name=name, sensor=sensor, attribute=attribute)))
            self._sensor_services.append(rospy.Service(topic + "/sensors/" + sensor, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._sensor_buffer,
                                                                         name=name, obs_name=sensor,
                                                                         message_type=get_response_from_def(space)
                                                                         )
                                                       )
                                         )

    def _init_actuators(self, topic, name, actuators):
        self._actuator_services[name] = {}
        self._action_servers[name] = {}
        for actuator in actuators:
            rospy.logdebug("Initializing actuator {}".format(actuator))
            names = actuators[actuator]["names"]
            space = actuators[actuator]['space']
            server_name = actuators[actuator]["server_name"]
            if not server_name.startswith('/'):
                server_name = topic + "/" + server_name
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
            self._actuator_services[name][actuator] = (get_action_srv, set_action_srv)

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
                            self._state_entries_callback, name=name, state=state_name, attribute=attribute, entries=entries))
                        )
                else:
                    self._state_subscribers.append(rospy.Subscriber(
                        msg_topic,
                        msg_type,
                        functools.partial(
                            self._state_callback, name=name, state=state_name, attribute=attribute))
                        )
            self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state_name,
                                                                         message_type=get_response_from_def(space)
                                                                         )
                                                       )
                                         )

    def _sensor_callback(self, data, name, sensor, attribute):
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
            get_state_srv = None
            check_state_srv = None
            state = states[state_name]
            space = state['space']
            reset_topic = state['reset_topic']
            if not reset_topic.startswith('/'):
                reset_topic = topic + '/' + reset_topic
            reset_srv_type = state['reset_type']
            reset_srv_package_srv = reset_srv_type.split(sep='/')
            reset_srv = getattr(importlib.import_module(reset_srv_package_srv[0]), reset_srv_package_srv[1])
            set_reset_srv = rospy.ServiceProxy(reset_topic, reset_srv)
            get_reset_srv = rospy.ServiceProxy(topic + "/resets/" + state_name, get_message_from_def(space))
            self._reset_services[name][state_name] = {'get': get_reset_srv, 'set': set_reset_srv}
            if 'check_topic' in state:
                check_topic = state['check_topic']
                if not check_topic.startswith('/'):
                    check_topic = topic + '/' + check_topic
                check_srv_type = state['check_type']
                check_srv_package_srv = check_srv_type.split(sep='/')
                check_srv = getattr(importlib.import_module(check_srv_package_srv[0]), check_srv_package_srv[1])
                check_state_srv = rospy.ServiceProxy(check_topic, check_srv)
                check_state_srv.wait_for_service()
                self._reset_services[name][state_name] = {'get': get_reset_srv, 'set': set_reset_srv, 'check': check_state_srv}
            if get_state_srv is not None:
                self._get_state_services.append(get_state_srv)

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv) = self._actuator_services[robot][actuator]
                actions = get_action_srv()
                set_action_srv(actions.value)
        # Get states from services
        for get_state_service in self._get_state_services:
            get_state_service()
        self.rate.sleep()
        return True

    def _reset(self, req):
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
                        rospy.logwarn('1: {}'.format(state.value))
                        if not validity_response.success:
                            rospy.logwarn(
                                '[{}] Reset state {} for object {} not valid!'.format(rospy.get_name(), state_name, object_name)
                                )
                            failed_state = State()
                            failed_state.name = state_name
                            failed_states.states.append(failed_state)
                        else:
                            rospy.logwarn('valid')
                            rospy.logwarn('2 {}'.format(state.value))
                            reset_response = reset_srvs['set'](list(state.value))
                            if not reset_response.success:
                                rospy.logwarn(
                                    '[{}] Reset state {} for object {} not failed!'.format(rospy.get_name(), state_name, object_name)
                                    )
                                failed_state = State()
                                failed_state.name = state_name
                                failed_states.states.append(failed_state)
                    else:
                        reset_srvs['set'](list(state.value))
            failed_resets.append(failed_states)
        response.objects = failed_resets
        return response

    def _close(self):
        return True

    def _seed(self, seed):
        pass
