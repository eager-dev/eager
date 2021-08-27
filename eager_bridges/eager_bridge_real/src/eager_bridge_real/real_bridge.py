import rospy
import roslaunch
import functools
import sensor_msgs.msg
from inspect import getmembers, isclass, isroutine
import eager_core.action_server
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
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

        super(RealBridge, self).__init__("real")

    def _register_object(self, topic, name, package, object_type, args, config):
        self._add_robot(topic, name, package, args, config)

        if 'sensors' in config:
            self._init_sensors(topic, name, config['sensors'])

        if 'actuators' in config:
            self._init_actuators(topic, name, config['actuators'])

        if 'states' in config:
            self._init_states(topic, name, config['states'])
        return True

    def _add_robot(self, topic, name, package, args, config):
        str_launch_object = '$(find %s)/launch/real.launch' % package
        cli_args = [substitute_xml_args(str_launch_object),
                    'ns:=%s' % topic,
                    'name:=%s' % name,
                    ]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        # launch.start()

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
            self._sensor_buffer[name][sensor] = [get_value_from_def(space)] * get_length_from_def(space)
            self._sensor_subscribers.append(rospy.Subscriber(
                                            msg_topic,
                                            msg_type,
                                            functools.partial(self._sensor_callback, name=name, sensor=sensor,
                                                              attribute=attribute)))
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
        for actuator in actuators:
            rospy.logdebug("Initializing actuator {}".format(actuator))
            names = actuators[actuator]["names"]
            space = actuators[actuator]['space']
            server_name = topic + "/" + actuators[actuator]["server_name"]
            action_server_name = actuators[actuator]["action_server"]
            valid_servers = [i[0] for i in getmembers(eager_core.action_server) if isclass(i[1])]
            if action_server_name in valid_servers:
                action_server = getattr(eager_core.action_server, action_server_name)
            else:
                rospy.logerror("Action server {} not implemented. Valid action servers are: {}".format(
                    action_server_name, valid_servers))
            get_action_srv = rospy.ServiceProxy(topic + "/actuators/" + actuator, get_message_from_def(space))
            set_action_srv = action_server(names, server_name).act
            self._actuator_services[name][actuator] = (get_action_srv, set_action_srv)

    def _init_states(self, topic, name, states):
        self._state_buffer[name] = {}
        for state in states:
            rospy.logdebug("Initializing state {}".format(state))
            state_params = states[state]
            space = state_params['space']
            self._state_buffer[name][state] = [get_value_from_def(space)] * get_length_from_def(space)
            self._sensor_services.append(rospy.Service(topic + "/states/" + state, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state,
                                                                         message_type=get_response_from_def(space)
                                                                         )
                                                       )
                                         )

    def _sensor_callback(self, data, name, sensor, attribute):
        self._sensor_buffer[name][sensor] = getattr(data, attribute)

    def _state_callback(self, data, state):
        # todo: implement routine to update state buffer.
        # data_list = data.position
        # self._state_buffer[state] = data_list
        pass

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv) = self._actuator_services[robot][actuator]
                actions = get_action_srv()
                set_action_srv(actions.value)
        self.rate.sleep()
        return True

    def _reset(self):
        return True

    def _close(self):
        self._launch.shutdown()
        return True

    def _seed(self, seed):
        pass
