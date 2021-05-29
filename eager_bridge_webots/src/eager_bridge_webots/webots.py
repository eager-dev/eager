import rospy, rosservice, roslaunch
import functools
import re
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_core.utils.message_utils import get_value_from_message
from webots_ros.msg import Float64Stamped
from webots_ros.srv import set_int, set_float

class WeBotsBridge(PhysicsBridge):

    def __init__(self):
        self._start_simulator()

        self._step_time = 80

        self._supervisor_name = self._get_supervisor()

        self._step_service = rospy.ServiceProxy(self._supervisor_name + "/robot/time_step", set_int)

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        super(WeBotsBridge, self).__init__("webots")

    def _start_simulator(self):
        str_launch_sim = '$(find eager_bridge_webots)/launch/webots_sim.launch'
        cli_args = [substitute_xml_args(str_launch_sim),
                    'mode:=%s' % rospy.get_param('physics_bridge/mode', 'fast'),
                    'no_gui:=%s' % rospy.get_param('physics_bridge/no_gui', 'false'),
                    'world:=%s' % rospy.get_param('physics_bridge/world')]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

    def _get_supervisor(cls):
        supervisor_checks = 0
        supervisors = [x for x in rosservice.get_service_list() if '/supervisor' in x]
        while not supervisors:
            if supervisor_checks > 20:
                rospy.logfatal("Could not find WeBots supervisor.")
                raise Exception
            supervisor_checks += 1
            rospy.sleep(1)
            supervisors = [x for x in rosservice.get_service_list() if '/supervisor' in x]

        return re.search("[^\/]+(?=\/supervisor)", supervisors[0]).group()


    def _register_object(self, topic, name, package, object_type, args, config):

        self._init_sensors(topic, name, config['sensors'])
        self._init_actuators(topic, name, config['actuators'])
        return True
    
    def _init_sensors(self, topic, name, sensors):
        robot_sensors = dict()
        for sensor_name in sensors:
            sensor = sensors[sensor_name]
            topic_list = sensor['names']
            messages = sensor['messages']
            robot_sensors[sensor_name] = [get_value_from_message(messages[0])]*len(topic_list)
            for idx, webots_sensor_name in enumerate(topic_list):
                enable_sensor = rospy.ServiceProxy(name + "/" + webots_sensor_name + "/enable", set_int)
                success = enable_sensor(self._step_time)
                if success:
                    self._sensor_subscribers.append(rospy.Subscriber(name + "/" + webots_sensor_name + "/value",
                        Float64Stamped, functools.partial(self._sensor_callback, name=name, sensor=sensor_name, pos=idx)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor_name, messages[0], functools.partial(self._sensor_service, name=name, sensor=sensor_name, message_type=messages[1])))
        self._sensor_buffer[name] = robot_sensors
    
    def _init_actuators(self, topic, name, actuators):
        robot_actuators = dict()
        for actuator_name in actuators:
            actuator = actuators[actuator_name]
            topic_list = actuator['names']
            messages = actuator['messages']
            set_action_srvs = []
            for webots_actuator_name in topic_list:
                set_action_srvs.append(rospy.ServiceProxy(name + "/" + webots_actuator_name + "/set_position", set_float))

            get_action_srv = rospy.ServiceProxy(topic + "/" + actuator_name, messages[0])
            robot_actuators[actuator_name] = (get_action_srv, set_action_srvs)
        self._actuator_services[name] = robot_actuators

    def _sensor_callback(self, data, name, sensor, pos):
        self._sensor_buffer[name][sensor][pos] = data.data

    def _sensor_service(self, req, name, sensor, message_type):
        return message_type(self._sensor_buffer[name][sensor])

    def _step(self):

        for robot in self._actuator_services:
            robot_actuators = self._actuator_services[robot]
            for actuator in robot_actuators:
                (get_action_srv, set_action_srvs) = robot_actuators[actuator]
                actions = get_action_srv()
                for idx, set_srv in enumerate(set_action_srvs):
                    success = set_srv(actions.value[idx])
                    if not success:
                        rospy.logwarn("Not all actions for %s could be set", actuator)
    
        self._step_service(self._step_time)
        # todo: Seems like after calling the step_service and returning the step service to RosEnv,
        #  we are not sure that the observation buffers are updated.
        return True

    def _reset(self):
        return True

    def _close(self):
        return True
