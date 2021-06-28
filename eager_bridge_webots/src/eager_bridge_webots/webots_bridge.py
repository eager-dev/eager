import rospy, rosservice, roslaunch
import functools
import re
import tempfile
import os
from eager_bridge_webots.webots_parser import WebotsParser
from eager_bridge_webots.orientation_utils import quad_to_axis_angle, quaternion_multiply
from operator import add
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def
from webots_ros.msg import Float64Stamped
import webots_ros.srv
from webots_ros.srv import set_int, set_float, get_uint64, node_get_field, field_import_node_from_string, supervisor_get_from_def
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Quaternion

class WeBotsBridge(PhysicsBridge):

    def __init__(self):
        self._start_simulator()

        self._step_time = rospy.get_param('physics_bridge/step_time')

        self._supervisor_name = self._get_supervisor()

        self._step_service = rospy.ServiceProxy(self._supervisor_name + "/robot/time_step", set_int)

        get_root = rospy.ServiceProxy(self._supervisor_name + '/supervisor/get_root', get_uint64)
        get_root.wait_for_service(20)
        root = get_root(True).value
        self._get_node_field = rospy.ServiceProxy(self._supervisor_name + '/supervisor/node/get_field', node_get_field)
        self._root_node_field = self._get_node_field(root, "children", False).field
        self._import_robot_service = rospy.ServiceProxy(self._supervisor_name + '/supervisor/field/import_node_from_string', field_import_node_from_string)
        self._get_node_from_def = rospy.ServiceProxy(self._supervisor_name + '/supervisor/get_from_def', supervisor_get_from_def)

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        self._state_buffer = dict()
        self._state_subscribers = []
        self._state_services = []

        self._reset_services = dict()

        super(WeBotsBridge, self).__init__("webots")

    def _start_simulator(self):

        world_file_path = substitute_xml_args('%s' % rospy.get_param('physics_bridge/world'))

        wp = WebotsParser()
        wp.load(world_file_path)
        set_time = False
        set_seed = not rospy.has_param('physics_bridge/seed')
        for f in wp.content['root']:
            if f['name'] == 'WorldInfo':
                for field in f['fields']:
                    if not set_time and field['name'] == 'basicTimeStep':
                        field['value'] = rospy.get_param('physics_bridge/basicTimeStep')
                        set_time = True
                    elif not set_seed and field['name'] == 'randomSeed':
                        field['value'] = rospy.get_param('physics_bridge/seed')
                        set_seed = True
                    if set_time and set_seed:
                        break
                if not set_time:
                    f['fields'].append({'type': 'SFInt32', 'name': 'basicTimeStep', 'value': rospy.get_param('physics_bridge/basicTimeStep')})
                if not set_seed:
                    f['fields'].append({'type': 'SFInt32', 'name': 'randomSeed', 'value': rospy.get_param('physics_bridge/seed')})
                break

        self.world_file = tempfile.NamedTemporaryFile(mode = 'w', suffix = '.wbt', dir = os.path.dirname(world_file_path))
        wp.save_in_file(self.world_file)
        self.world_file.flush()
        
        str_launch_sim = '$(find eager_bridge_webots)/launch/webots_sim.launch'
        cli_args = [substitute_xml_args(str_launch_sim),
                    'mode:=%s' % rospy.get_param('physics_bridge/mode', 'fast'),
                    'no_gui:=%s' % rospy.get_param('physics_bridge/no_gui', False),
                    'world:=%s' % self.world_file.name]
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
            self._init_resets(topic, name, config['states'])
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
            space = state['space']
            if state_name == 'position':
                field_getter = self._get_field(name, 'translation', 'vec3f')
                resp = get_response_from_def(space)
                self._state_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                    lambda _, field_getter=field_getter: resp(field_getter(index=0).value.__getstate__())))
            elif state_name == 'orientation':
                field_getter = self._get_field(name, 'rotation', 'rotation')
                resp = get_response_from_def(space)
                self._state_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                    lambda _, field_getter=field_getter: resp(field_getter(index=0).value.__getstate__())))
            else:
                self._state_buffer[name][state_name] = [get_value_from_def(space)] * get_length_from_def(space)
                self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                            functools.partial(self._service,
                                                                buffer=self._state_buffer,
                                                                name=name,
                                                                obs_name=state_name,
                                                                message_type=get_response_from_def(space))))

    def _init_resets(self, topic, name, states):
        self._reset_services[name] = dict()
        for state_name in states:
            state = states[state_name]
            space = state['space']
            if state_name == 'position':
                field_setter = self._set_field(name, 'translation', 'vec3f')
                set_reset_srv = lambda value, field_setter=field_setter: field_setter(index=0, value=Vector3(*value))
            elif state_name == 'orientation':
                field_setter = self._set_field(name, 'rotation', 'rotation')
                set_reset_srv = lambda value, field_setter=field_setter: field_setter(index=0, value=Quaternion(*value))
            else:
                continue
            get_reset_srv = rospy.ServiceProxy(topic + "/resets/" + state_name, get_message_from_def(space))
            self._reset_services[name][state_name] = (get_reset_srv, set_reset_srv)

    def _get_field(self, node_def, field_name, field_type):
        """
        Possible field types:
            bool, int32, float, vec2f, vec3f, rotation, color, string, node
        """
        node = self._get_node_from_def(node_def, 0).node
        field = self._get_node_field(node, field_name, False).field
        get_srv = rospy.ServiceProxy(self._supervisor_name + '/supervisor/field/get_' + field_type,
            getattr(webots_ros.srv, 'field_get_' + field_type))
        return functools.partial(get_srv.__call__, field=field)
    
    def _set_field(self, node_def, field_name, field_type):
        """
        Possible field types:
            bool, int32, float, vec2f, vec3f, rotation, color, string, node
        """
        node = self._get_node_from_def(node_def, 0).node
        field = self._get_node_field(node, field_name, False).field
        set_srv = rospy.ServiceProxy(self._supervisor_name + '/supervisor/field/set_' + field_type,
            getattr(webots_ros.srv, 'field_set_' + field_type))
        return functools.partial(set_srv.__call__, field=field)
    
    def _add_robot(self, node_type, name, args, config):
        pos = '{} {} {}'.format(*map(add, config['default_translation'], args['position']))
        ori = '{} {} {} {}'.format(*quad_to_axis_angle(quaternion_multiply(config['default_orientation'], args['orientation'])))
        self._import_robot_service(self._root_node_field, 0, 'DEF {} {} {{ translation {} rotation {} controller "ros" controllerArgs [ "--name={}" ]}}'.format(name, node_type, pos, ori, name))

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
        # Set all actions before stepping the world
        for robot in self._reset_services:
            robot_resets = self._reset_services[robot]
            for state in robot_resets:
                (get_state_srv, set_reset_srvs) = robot_resets[state]
                state = get_state_srv()
                print(state.value)
                set_reset_srvs(list(state.value))

        # update all observation & state buffers
        # Sensors are topics here, no quarantee for update, reset to 0?
        # There is no state buffer
        return True

    def _close(self):
        self._launch.shutdown()
        # TODO: WeBots should fix this cleanup...
        os.remove(os.path.dirname(self.world_file.name) + '/.' + os.path.splitext(os.path.basename(self.world_file.name))[0] + '.wbproj')
        self.world_file.close()
        return True
    
    def _seed(self, seed):
        rospy.logwarn("Webots must be seeded in the world file (.wbt). Did not set seed to %d.", seed)
