import rospy, rosservice, roslaunch
import functools
import re
from eager_bridge_webots.webots_parser import WebotsParser
from eager_bridge_webots.orientation_utils import quad_to_axis_angle, quaternion_multiply
from operator import add
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def, get_dtype_from_def
from webots_ros.msg import Float64Stamped
from webots_ros.srv import set_int, set_float, get_uint64, node_get_field, field_import_node_from_string
from sensor_msgs.msg import Image

class WeBotsBridge(PhysicsBridge):

    def __init__(self):
        self._start_simulator()

        self._step_time = rospy.get_param('physics_bridge/step_time')

        self._supervisor_name = self._get_supervisor()

        self._step_service = rospy.ServiceProxy(self._supervisor_name + "/robot/time_step", set_int)

        get_root = rospy.ServiceProxy(self._supervisor_name + '/supervisor/get_root', get_uint64)
        get_root.wait_for_service(20)
        root = get_root(True).value
        get_node_field = rospy.ServiceProxy(self._supervisor_name + '/supervisor/node/get_field', node_get_field)
        self._root_node_field = get_node_field(root, "children", False).field
        self._import_robot_service = rospy.ServiceProxy(self._supervisor_name + '/supervisor/field/import_node_from_string', field_import_node_from_string)

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        self._state_buffer = dict()
        self._state_subscribers = []
        self._state_services = []

        super(WeBotsBridge, self).__init__("webots")

    def _start_simulator(self):
        """
        wp = WebotsParser()
        print(substitute_xml_args('%s' % rospy.get_param('physics_bridge/world')))
        wp.load(substitute_xml_args('%s' % rospy.get_param('physics_bridge/world')))
        print(wp.content)
        for f in wp.content['root']:
            for field in f['fields']:
                if field['name'] == 'basicTimeStep':
                    #field['value'] = rospy.get_param('physics_bridge/basicTimeStep', 20)
                    break
            else:
                continue
            break

        """
        
        str_launch_sim = '$(find eager_bridge_webots)/launch/webots_sim.launch'
        cli_args = [substitute_xml_args(str_launch_sim),
                    'mode:=%s' % rospy.get_param('physics_bridge/mode', 'fast'),
                    'no_gui:=%s' % rospy.get_param('physics_bridge/no_gui', 'false'),
                    'world:=%s' % rospy.get_param('physics_bridge/world')]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        self._launch.start()

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
        self._add_robot(config['node_type_name'], name, args, config)
        if 'sensors' in config:
            self._init_sensors(topic, name, config['sensors'])
        if 'actuators' in config:
            self._init_actuators(topic, name, config['actuators'])
        if 'states' in config:
            self._init_states(topic, name, config['states'])
        return True
    
    def _init_sensors(self, topic, name, sensors):
        self._sensor_buffer[name] = dict()
        for sensor_name in sensors:
            sensor = sensors[sensor_name]
            topic_list = sensor['names']
            space = sensor['space']
            self._sensor_buffer[name][sensor_name] = [get_value_from_def(space)] * get_length_from_def(space)
            for idx, webots_sensor_name in enumerate(topic_list):
                enable_sensor = rospy.ServiceProxy(name + "/" + webots_sensor_name + "/enable", set_int)
                enable_sensor.wait_for_service(5)

                success = enable_sensor(self._step_time) # For the second robot WeBots gets stuck here.
                # To continue handing this service it needs a step in the synchronized robot.
                # Which we cannot do at the same time as we are stuck in this service call.
                # Seems unfixable without major multi-process hacky things.
                if success:
                    if 'type' in sensor:
                        sens_type = sensor['type']
                        if sens_type == 'camera':
                            assert len(topic_list) == 1 # Temp check
                            self._sensor_subscribers.append(rospy.Subscriber(name + "/" + webots_sensor_name + "/image",
                                Image, functools.partial(self._camera_callback, name=name, sensor=sensor_name)))
                    else:
                        self._sensor_subscribers.append(rospy.Subscriber(name + "/" + webots_sensor_name + "/value",
                            Float64Stamped, functools.partial(self._sensor_callback, name=name, sensor=sensor_name, pos=idx)))
            self._sensor_services.append(rospy.Service(topic + "/sensors/" + sensor_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._sensor_buffer,
                                                                         name=name,
                                                                         obs_name=sensor_name,
                                                                         message_type=get_response_from_def(space))))
    
    def _init_actuators(self, topic, name, actuators):
        self._actuator_services[name] = dict()
        for actuator_name in actuators:
            actuator = actuators[actuator_name]
            topic_list = actuator['names']
            space = actuator['space']
            set_action_srvs = []
            for webots_actuator_name in topic_list:
                set_action_srvs.append(rospy.ServiceProxy(name + "/" + webots_actuator_name + "/set_position", set_float))

            get_action_srv = rospy.ServiceProxy(topic + "/actuators/" + actuator_name, get_message_from_def(space))
            self._actuator_services[name][actuator_name] = (get_action_srv, set_action_srvs)

    def _init_states(self, topic, name, states):
        self._state_buffer[name] = dict()
        for state_name in states:
            state = states[state_name]
            topic_list = state['names']
            space = state['space']
            self._state_buffer[name][state_name] = [get_value_from_def(space)] * get_length_from_def(space)
            # for idx, webots_sensor_name in enumerate(topic_list):
            #     enable_sensor = rospy.ServiceProxy(name + "/" + webots_sensor_name + "/enable", set_int)
            #     success = enable_sensor(self._step_time)
            #     if success:
            #         self._sensor_subscribers.append(rospy.Subscriber(name + "/" + webots_sensor_name + "/value",
            #             Float64Stamped, functools.partial(self._sensor_callback, name=name, sensor=sensor, pos=idx)))
            self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state_name,
                                                                         message_type=get_response_from_def(space))))
    
    def _add_robot(self, node_type, name, args, config):
        pos = '{} {} {}'.format(*map(add, config['default_translation'], args['position']))
        ori = '{} {} {} {}'.format(*quad_to_axis_angle(quaternion_multiply(config['default_orientation'], args['orientation'])))
        self._import_robot_service(self._root_node_field, 0, '{} {{ translation {} rotation {} controller "ros" controllerArgs [ "--name={}" ]}}'.format(node_type, pos, ori, name))
    

    def _sensor_callback(self, data, name, sensor, pos):
        if data.data != data.data:
            rospy.logwarn('NaN sensor input from WeBots')
            return
        self._sensor_buffer[name][sensor][pos] = data.data
    
    def _camera_callback(self, data, name, sensor):
        #TODO: Standardize image encoding
        self._sensor_buffer[name][sensor] = data.data

    def _state_callback(self, data, name, state, pos):
        # todo: implement routine to update state buffer.
        # self._state_buffer[name][state][pos] = data.data
        pass

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

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
    
        #self._step_service(self._step_time)
        rospy.sleep(float(self._step_time)/1000)
        # todo: Seems like after calling the step_service and returning the step service to RosEnv,
        #  we are not sure that the observation buffers are updated.
        return True

    def _reset(self):
        return True

    def _close(self):
        self._launch.shutdown()
        return True
    
    def _seed(self, seed):
        rospy.logwarn("Webots must be seeded in the world file (.wbt). Did not set seed to %d.", seed)
