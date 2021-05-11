import rospy
import functools
import sensor_msgs.msg
from physics_bridge import PhysicsBridge
from ros_env.srv import BoxSpace, BoxSpaceResponse
from action_server.servers.moveit_action_server import MoveitActionServer
from utils.sensor_conversion import create_sensor_converter

class RealBridge(PhysicsBridge):

    def __init__(self, name = 'physics_bridge'):
        self._step_time = 0.2
        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []
        self._sensor_converters = dict()
        self._action_servers = dict()
        self._actuator_services = dict()

        super(RealBridge, self).__init__("real", name)

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
            rospy.wait_for_message(msg_topic, msg_type)
            rospy.logdebug("Sensor {} received message from topic {}".format(sensor, msg_topic))
            self._sensor_converters[sensor] = create_sensor_converter(msg_type)
            self._sensor_subscribers.append(rospy.Subscriber(
                msg_topic,
                msg_type, 
                functools.partial(self._sensor_callback, sensor=sensor)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, BoxSpace, functools.partial(self._sensor_service, sensor=sensor)))
    
    def _init_actuators(self, topic, name, actuators):
      for actuator in actuators:
          rospy.logdebug("Initializing actuator {}".format(actuator))
          actuator_params = actuators[actuator]
          rate = actuator_params["rate"]
          if actuator_params["type"].strip().lower() == "moveit":
               self._action_servers[actuator] = MoveitActionServer(topic, name, actuator, actuator_params)
               rospy.Timer(rospy.Duration(1.0 / rate), self._action_servers[actuator].publish_action)
          else:
              rospy.logerr("Currently only actuators of type \"MoveIt\" are implemented")
          self._actuator_services[actuator] = rospy.ServiceProxy(topic + "/" + actuator, BoxSpace)

    def _sensor_callback(self, data, sensor):
        data_list = data.position
        self._sensor_buffer[sensor] = data_list
    
    def _sensor_service(self, req, sensor):
        return BoxSpaceResponse(self._sensor_buffer[sensor])

    def _step(self):
        rospy.logdebug("Stepping")
        for actuator in self._actuator_services:
            get_action_srv = self._actuator_services[actuator]
            actions = get_action_srv()
            rospy.logdebug("Actuator {} received action: {}".format(actuator, actions.value))
            self._action_servers[actuator].set_action(actions.value)
            rospy.sleep(self._step_time)
        return True

    def _reset(self):
        return True

    def _close(self):
        return True
