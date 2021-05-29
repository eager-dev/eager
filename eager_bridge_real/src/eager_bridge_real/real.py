import rospy
import functools
import sensor_msgs.msg
from eager_core.physics_bridge import PhysicsBridge
from action_server.servers.follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer

class RealBridge(PhysicsBridge):

    def __init__(self, name = 'physics_bridge'):
        action_rate = rospy.get_param('physics_bridge/rate', 0.1)
        self.rate = rospy.Rate(action_rate)
        
        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []
        self._actuator_services = dict()

        super(RealBridge, self).__init__("real")

    def _register_object(self, topic, name, package, object_type, args, config):
        self._init_sensors(topic, name, config['sensors'])
        
        self._init_actuators(topic, name, config['actuators'])

        return True
    
    def _init_sensors(self, topic, name, sensors):
        for sensor in sensors:
            rospy.logdebug("Initializing sensor {}".format(sensor))
            self._sensor_buffer[sensor] = []
            sensor_params = sensors[sensor]
            msg_topic = name + "/" + sensor_params["topic"]
            msg_name = sensor_params["msg_name"]
            messages = sensor_params['messages']
            msg_type = getattr(sensor_msgs.msg, msg_name)
            rospy.logdebug("Waiting for message topic {}".format(msg_topic))
            rospy.wait_for_message(msg_topic, msg_type)
            rospy.logdebug("Sensor {} received message from topic {}".format(sensor, msg_topic))
            self._sensor_subscribers.append(rospy.Subscriber(
                msg_topic,
                msg_type, 
                functools.partial(self._sensor_callback, sensor=sensor)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, messages[0], functools.partial(self._sensor_service, sensor=sensor, message_type=messages[1])))
    
    def _init_actuators(self, topic, name, actuators):
      for actuator in actuators:
          rospy.logdebug("Initializing actuator {}".format(actuator))
          joint_names = actuators[actuator]["joint_names"]
          messages = actuators[actuator]['messages']
          server_name = name + "/" + actuators[actuator]["server_name"]
          get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, messages[0])
          set_action_srv = FollowJointTrajectoryActionServer(joint_names, server_name).act
          self._actuator_services[actuator] = (get_action_srv, set_action_srv)
        
    def _sensor_callback(self, data, sensor):
        data_list = data.position
        self._sensor_buffer[sensor] = data_list
    
    def _sensor_service(self, req, sensor, message_type):
        return message_type(self._sensor_buffer[sensor])

    def _step(self):
        rospy.logdebug("Stepping")
        for actuator in self._actuator_services:
            (get_action_srv, set_action_srv) = self._actuator_services[actuator]
            actions = get_action_srv()
            rospy.logdebug("Actuator {} received action: {}".format(actuator, actions.value))
            set_action_srv(actions.value)
        self.rate.sleep()
        return True

    def _reset(self):
        return True

    def _close(self):
        return True