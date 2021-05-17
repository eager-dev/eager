import rospy
import functools
import sensor_msgs.msg
from ros_gym_core.physics_bridge import PhysicsBridge
from ros_gym_core.srv import BoxSpace, BoxSpaceResponse
from ros_gym_bridge_real.action_server.servers.follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer
from std_srvs.srv import Empty, EmptyRequest, Trigger, TriggerResponse
from gazebo_msgs.srv import GetPhysicsProperties, GetPhysicsPropertiesRequest, SetPhysicsProperties, SetPhysicsPropertiesRequest
from gazebo_step_world_plugin.srv import SetInt, SetIntRequest
class GazeboBridge(PhysicsBridge):

    def __init__(self, name = 'physics_bridge'):
        step_time = 0.1
        self.paused = False
        
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_physics_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        
        rospy.wait_for_service('/gazebo/get_physics_properties')
        physics_parameters_service = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        physics_parameters = physics_parameters_service(GetPhysicsPropertiesRequest())
        
        new_physics_parameters = SetPhysicsPropertiesRequest()
        new_physics_parameters.time_step = physics_parameters.time_step
        new_physics_parameters.max_update_rate = 0.0
        new_physics_parameters.gravity = physics_parameters.gravity
        new_physics_parameters.ode_config = physics_parameters.ode_config

        rospy.wait_for_service('/gazebo/set_physics_properties')
        set_physics_properties_service = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        set_physics_properties_service(new_physics_parameters)
        
        rospy.wait_for_service('/gazebo/step_world')
        self.step_world = rospy.ServiceProxy('/gazebo/step_world', SetInt)
        
        self.step_request = SetIntRequest(int(round(step_time/new_physics_parameters.time_step)))
        
        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []
        self._actuator_services = dict()

        super(GazeboBridge, self).__init__("gazebo")

    def _register_object(self, topic, name, params):

        self._init_sensors(topic, name, params['sensors'])
        
        self._init_actuators(topic, name, params['actuators'])

        return True
    
    def _init_sensors(self, topic, name, sensors):
        for sensor in sensors:
            rospy.logdebug("Initializing sensor {}".format(sensor))
            self._sensor_buffer[sensor] = []
            sensor_params = sensors[sensor]
            msg_topic = sensor_params["topic"]
            msg_name = sensor_params["msg_name"]
            msg_type = getattr(sensor_msgs.msg, msg_name)
            rospy.logdebug("Waiting for message topic {}".format(msg_topic))
            rospy.wait_for_message("/" + msg_topic, msg_type)
            rospy.logdebug("Sensor {} received message from topic {}".format(sensor, msg_topic))
            self._sensor_subscribers.append(rospy.Subscriber(
                "/" + msg_topic,
                msg_type, 
                functools.partial(self._sensor_callback, sensor=sensor)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, BoxSpace, functools.partial(self._sensor_service, sensor=sensor)))
    
    def _init_actuators(self, topic, name, actuators):
      for actuator in actuators:
          rospy.logdebug("Initializing actuator {}".format(actuator))
          joint_names = actuators[actuator]["joint_names"]
          server_name = "/" + actuators[actuator]["server_name"]
          get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, BoxSpace)
          set_action_srv = FollowJointTrajectoryActionServer(joint_names, server_name).act
          # preprocess_srv = rospy.Service(topic + "/" + actuator + "/enable_preprocess", Trigger, self._actuator_enable_preprocess_service(topic, actuator))
          self._actuator_services[actuator] = (get_action_srv, set_action_srv)
          
    # def _actuator_enable_preprocess_service(self, topic, actuator):
    #     (_, set_action_srv, preprocess_srv) = self._actuator_services[actuator]
    #     get_action_srv = rospy.ServiceProxy(topic + "/" + actuator + "/preprocessed", BoxSpace)
    #     self._actuator_services[actuator] = (get_action_srv, set_action_srv, preprocess_srv)
    #     response = TriggerResponse()
    #     response.success = True
    #     return response
        
    def _sensor_callback(self, data, sensor):
        data_list = data.position
        self._sensor_buffer[sensor] = data_list
    
    def _sensor_service(self, req, sensor):
        return BoxSpaceResponse(self._sensor_buffer[sensor])

    def _step(self):
        if not self.paused:
            self.pause_physics_service()
            self.paused = True
        rospy.logdebug("Stepping")
        for actuator in self._actuator_services:
            (get_action_srv, set_action_srv) = self._actuator_services[actuator]
            actions = get_action_srv()
            rospy.logdebug("Actuator {} received action: {}".format(actuator, actions.value))
            set_action_srv(actions.value)
        self.step_world(self.step_request)
        return True

    def _reset(self):
        return True

    def _close(self):
        return True
