import rospy
import functools
import sensor_msgs.msg
from inspect import getmembers, isclass, isroutine
import eager_core.action_server
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def


class RealBridge(PhysicsBridge):

    def __init__(self):
        action_rate = rospy.get_param('physics_bridge/action_rate', 30)
        self.rate = rospy.Rate(action_rate)

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

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
            entries = sensor_params['entries']
            self._sensor_buffer[name][sensor] = [get_value_from_def(space)] * get_length_from_def(space)
            self._sensor_subscribers.append(rospy.Subscriber(
                msg_topic,
                msg_type,
                functools.partial(self._sensor_callback, name=name, sensor=sensor, attribute=attribute, entries=entries)))
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
                    msg_topic = topic + '/' + state['topic']
                else:
                    msg_topic = topic + '/joint_states'
                if 'entries' in state:
                    entries = state['entries']
                msg_type = sensor_msgs.msg.JointState
                if state['type'] == 'joint_pos':
                    attribute = 'position'
                elif state['type'] == 'joint_vel':
                    attribute = 'velocity'
                self._state_subscribers.append(rospy.Subscriber(
                    msg_topic,
                    msg_type,
                    functools.partial(self._state_callback, name=name, state=state_name, attribute=attribute, entries=entries)))
            self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state_name,
                                                                         message_type=get_response_from_def(space)
                                                                         )
                                                       )
                                         )

    def _sensor_callback(self, data, name, sensor, attribute, entries):
        data_list = getattr(data, attribute)
        self._sensor_buffer[name][sensor] = list(map(data_list.__getitem__, entries))

    def _state_callback(self, data, name, state, attribute, entries):
        data_list = getattr(data, attribute)
        self._state_buffer[name][state] = list(map(data_list.__getitem__, entries))

    def _init_resets(self, topic, name, states):
        self._reset_services[name] = dict()
        for state_name in states:
            state = states[state_name]
            space = state['space']
            get_state_srv = None
            if 'joint' in state['type']:
                if state['type'] == 'joint_pos':
                    def set_reset_srv(value):
                        pass
                elif state['type'] == 'joint_vel':
                    rospy.logwarn("Reset of joint velocities is not possible in reality!")
            elif 'base' in state['type']:
                rospy.logwarn("Reset of base is not implemented!")
            else:
                raise ValueError('State type ("%s") must contain "{joint, base}".' % states[state]['type'])
            get_reset_srv = rospy.ServiceProxy(topic + "/resets/" + state_name, get_message_from_def(space))
            self._reset_services[name][state_name] = (get_reset_srv, set_reset_srv)
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

    def _reset(self):
        for robot in self._reset_services:
            robot_resets = self._reset_services[robot]
            for state in robot_resets:
                (get_state_srv, set_reset_srvs) = robot_resets[state]
                state = get_state_srv()
                if state.value:
                    set_reset_srvs(list(state.value))
        return True

    def _close(self):
        self._launch.shutdown()
        return True

    def _seed(self, seed):
        pass
