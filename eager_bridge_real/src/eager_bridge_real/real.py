from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def
import rospy
import roslaunch
import functools
import sensor_msgs.msg
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_real.action_server.servers.follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer


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
        str_launch_object = '$(find %s)/launch/real.launch' % package
        cli_args = [substitute_xml_args(str_launch_object),
                    'ns:=%s' % name]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        self._launch.start()
        
        self._init_sensors(topic, name, config['sensors'])
        
        self._init_actuators(topic, name, config['actuators'])

        self._init_states(topic, name, config['states'])
        return True
    
    def _init_sensors(self, topic, name, sensors):
        self._sensor_buffer[name] = {}
        for sensor in sensors:
            rospy.logdebug("Initializing sensor {}".format(sensor))
            self._sensor_buffer[name][sensor] = []
            sensor_params = sensors[sensor]
            msg_topic = name + "/" + sensor_params["topic"]
            msg_name = sensor_params["msg_name"]
            space = sensor_params['space']
            msg_type = getattr(sensor_msgs.msg, msg_name)
            rospy.logdebug("Waiting for message topic {}".format(msg_topic))
            rospy.logdebug("Sensor {} received message from topic {}".format(sensor, msg_topic))
            self._sensor_subscribers.append(rospy.Subscriber(
                msg_topic,
                msg_type, 
                functools.partial(self._sensor_callback, name=name, sensor=sensor)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, get_message_from_def(space), functools.partial(self._service, buffer=self._sensor_buffer, name=name, obs_name=sensor, message_type=get_response_from_def(space))))

    
    def _init_actuators(self, topic, name, actuators):
        actuator_services = dict()
        for actuator in actuators:
            rospy.logdebug("Initializing actuator {}".format(actuator))
            joint_names = actuators[actuator]["names"]
            space = actuators[actuator]['space']
            server_name = name + "/" + actuators[actuator]["server_name"]
            get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, get_message_from_def(space))
            set_action_srv = FollowJointTrajectoryActionServer(joint_names, server_name).act
            actuator_services[actuator] = (get_action_srv, set_action_srv)
        self._actuator_services[name] = actuator_services

    def _init_states(self, topic, name, states):
        robot_states = dict()
        for state in states:
            rospy.logdebug("Initializing state {}".format(state))
            state_params = states[state]
            space = state_params['space']
            robot_states[state] = [get_value_from_def(space)]*len(states[state])
            # msg_topic = name + "/" + state_params["topic"]
            # msg_name = state_params["msg_name"]
            # msg_type = getattr(state_msgs.msg, msg_name)
            # rospy.logdebug("Waiting for message topic {}".format(msg_topic))
            # rospy.wait_for_message(msg_topic, msg_type)
            # rospy.logdebug("Sensor {} received message from topic {}".format(state, msg_topic))
            # self._state_subscribers.append(rospy.Subscriber(
            #     msg_topic,
            #     msg_type,
            #     functools.partial(self._state_callback, state=state)))
            self._sensor_services.append(rospy.Service(topic + "/" + state, get_message_from_def(space), functools.partial(self._service, buffer=self._state_buffer, name=name, obs_name=state, message_type=get_response_from_def(space))))
        self._state_buffer[name] = robot_states
        
    def _sensor_callback(self, data, name, sensor):
        data_list = data.position
        self._sensor_buffer[name][sensor] = data_list

    def _state_callback(self, data, state):
        # todo: implement routine to update state buffer.
        # data_list = data.position
        # self._state_buffer[state] = data_list
        pass

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        rospy.logdebug("Stepping")
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv) = self._actuator_services[robot][actuator]
                actions = get_action_srv()
                rospy.logdebug("Actuator {} received action: {}".format(actuator, actions.value))
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
