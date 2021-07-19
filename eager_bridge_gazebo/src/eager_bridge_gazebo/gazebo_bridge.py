from eager_core.utils.message_utils import get_value_from_def, get_message_from_def, get_response_from_def, get_length_from_def
import rospy
import roslaunch
import functools
import sensor_msgs.msg
from operator import add
from eager_core.physics_bridge import PhysicsBridge
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_gazebo.orientation_utils import quaternion_multiply, euler_from_quaternion
from eager_bridge_gazebo.action_server.servers.follow_joint_trajectory_action_server import \
    FollowJointTrajectoryActionServer
from gazebo_msgs.srv import (GetPhysicsProperties, GetPhysicsPropertiesRequest, SetPhysicsProperties,
                             SetPhysicsPropertiesRequest, SetModelState, SetModelStateRequest, SetModelConfiguration,
                             SetModelConfigurationRequest, GetModelState, GetModelStateRequest)
from eager_bridge_gazebo.srv import SetInt, SetIntRequest
from geometry_msgs.msg import Point, Quaternion


class GazeboBridge(PhysicsBridge):

    def __init__(self):
        self._start_simulator()
        self.stepped = False

        step_time = rospy.get_param('physics_bridge/step_time', 0.1)

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
        self._set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self._set_model_configuration.wait_for_service()

        self._step_world = rospy.ServiceProxy('/gazebo/step_world', SetInt)
        self._step_world.wait_for_service()

        self.step_request = SetIntRequest(int(round(step_time / physics_parameters.time_step)))

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        self._state_buffer = dict()
        self._state_subscribers = []
        self._state_services = []
        self._get_state_services = []

        self._reset_services = dict()

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
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        launch.start()

    def _init_sensors(self, topic, name, sensors):
        self._sensor_buffer[name] = {}
        for sensor in sensors:
            rospy.logdebug("Initializing sensor {}".format(sensor))
            sensor_params = sensors[sensor]
            msg_topic = topic + "/" + sensor_params["topic"]
            msg_name = sensor_params["msg_name"]
            space = sensor_params['space']
            msg_type = getattr(sensor_msgs.msg, msg_name)
            attribute = sensor_params["type"]
            self._sensor_buffer[name][sensor] = [get_value_from_def(space)] * get_length_from_def(space)
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
        for actuator in actuators:
            rospy.logdebug("Initializing actuator {}".format(actuator))
            joint_names = actuators[actuator]["names"]
            space = actuators[actuator]['space']
            server_name = topic + "/" + actuators[actuator]["server_name"]
            get_action_srv = rospy.ServiceProxy(topic + "/actuators/" + actuator, get_message_from_def(space))
            set_action_srv = FollowJointTrajectoryActionServer(joint_names, server_name).act
            self._actuator_services[name][actuator] = (get_action_srv, set_action_srv)

    def _init_states(self, topic, name, states):
        self._state_buffer[name] = {}
        for state_name in states:
            rospy.logdebug("Initializing state {}".format(state_name))
            state = states[state_name]
            space = state['space']
            self._state_buffer[name][state_name] = [get_value_from_def(space)] * get_length_from_def(space)
            if state['type'] == 'joint_pos' or state['type'] == 'joint_vel':
                msg_topic = topic + '/joint_states'
                msg_type = sensor_msgs.msg.JointState
                if state['type'] == 'joint_pos':
                    attribute = 'position'
                elif state['type'] == 'joint_vel':
                    attribute = 'velocity'
                self._state_subscribers.append(rospy.Subscriber(
                    msg_topic,
                    msg_type,
                    functools.partial(self._state_callback, name=name, state=state_name, attribute=attribute)))
            self._sensor_services.append(rospy.Service(topic + "/states/" + state_name, get_message_from_def(space),
                                                       functools.partial(self._service,
                                                                         buffer=self._state_buffer,
                                                                         name=name,
                                                                         obs_name=state_name,
                                                                         message_type=get_response_from_def(space)
                                                                         )
                                                       )
                                         )

    def _init_resets(self, topic, name, states):
        self._reset_services[name] = dict()
        for state_name in states:
            state = states[state_name]
            space = state['space']
            get_state_srv = None
            if 'joint' in state['type']:
                if state['type'] == 'joint_pos':
                    request = SetModelConfigurationRequest()
                    request.model_name = name
                    request.joint_names = state['names']

                    def set_reset_srv(value):
                        request.joint_positions = value
                        return self._set_model_configuration(request)
                elif state['type'] == 'joint_vel':
                    raise ValueError('State type ("%s") cannot be reset in gazebo.' % states[state]['type'])
            elif 'link' in state['type']:
                raise ValueError('State type ("%s") cannot be reset in Gazebo.' % states[state]['type'])
            elif 'base' in state['type']:
                set_request = SetModelStateRequest()
                get_request = GetModelStateRequest()

                set_request.model_state.model_name = name
                get_request.model_name = name
                if state['type'] == 'base_pos':
                    def set_reset_srv(value):
                        set_request.model_state.pose.position = Point(*value)
                        return self._set_model_state(set_request)

                    def get_state_srv():
                        response = self._get_model_state(get_request)
                        position = [response.pose.position.x, response.pose.position.y, response.pose.position.z]
                        self._state_buffer[name][state_name] = position
                elif state['type'] == 'base_orientation':
                    def set_reset_srv(value):
                        set_request.model_state.pose.orientation = Quaternion(*value)
                        return self._set_model_state(set_request)

                    def get_state_srv():
                        response = self._get_model_state(get_request)
                        orientation = [response.pose.orientation.x, response.pose.orientation.y, response.pose.orientation.z,
                                       response.pose.orientation.w]
                        self._state_buffer[name][state_name] = orientation
            else:
                raise ValueError('State type ("%s") must contain "{joint, base}".' % states[state]['type'])
            get_reset_srv = rospy.ServiceProxy(topic + "/resets/" + state_name, get_message_from_def(space))
            self._reset_services[name][state_name] = (get_reset_srv, set_reset_srv)
            if get_state_srv is not None:
                self._get_state_services.append(get_state_srv)

    def _sensor_callback(self, data, name, sensor, attribute):
        self._sensor_buffer[name][sensor] = getattr(data, attribute)

    def _state_callback(self, data, name, state, attribute):
        self._state_buffer[name][state] = getattr(data, attribute)

    def _service(self, req, buffer, name, obs_name, message_type):
        return message_type(buffer[name][obs_name])

    def _step(self):
        for robot in self._actuator_services:
            for actuator in self._actuator_services[robot]:
                (get_action_srv, set_action_srv) = self._actuator_services[robot][actuator]
                actions = get_action_srv()
                set_action_srv(actions.value)
        self._step_world(self.step_request)
        # Get states from services
        for get_state_service in self._get_state_services:
            get_state_service()
        return True

    def _reset(self):
        # Set all actions before stepping the world
        for robot in self._reset_services:
            robot_resets = self._reset_services[robot]
            for state in robot_resets:
                (get_state_srv, set_reset_srvs) = robot_resets[state]
                state = get_state_srv()
                if state.value:
                    set_reset_srvs(list(state.value))

        # update all observation & state buffers
        # Sensors are topics here, no quarantee for update, reset to 0?
        # There is no state buffer
        return True

        return True

    def _close(self):
        self._launch.shutdown()
        return True

    def _seed(self, seed):
        pass
