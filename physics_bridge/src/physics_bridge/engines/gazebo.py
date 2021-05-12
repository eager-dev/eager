import rospy
import functools
import sensor_msgs.msg
from physics_bridge import PhysicsBridge
from ros_env.srv import BoxSpace, BoxSpaceResponse
from action_server.servers.follow_joint_trajectory_action_server import FollowJointTrajectoryActionServer
from utils.sensor_conversion import create_sensor_converter
from std_srvs.srv import Empty, EmptyRequest

class GazeboBridge(PhysicsBridge):

    def __init__(self, name = 'physics_bridge'):
        rospy.wait_for_service('/gazebo/pause_physics')
        rospy.wait_for_service('/gazebo/unpause_physics')
        self._pause =  rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self._unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        self._step_time = 0.1
        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []
        self._sensor_converters = dict()
        self._action_servers = dict()
        self._actuator_services = dict()

        super(GazeboBridge, self).__init__("gazebo", name)

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
          joint_names = actuators[actuator]["joint_names"]
          server_name = actuators[actuator]["server_name"]
          self._action_servers[actuator] = FollowJointTrajectoryActionServer(joint_names, server_name)
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
            self._action_servers[actuator].act(actions.value)
            self._unpause(EmptyRequest())
            rospy.sleep(self._step_time)
            self._pause(EmptyRequest())
        return True

    def _reset(self):
        return True

    def _close(self):
        return True
