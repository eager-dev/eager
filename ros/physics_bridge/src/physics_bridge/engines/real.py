import rospy
import functools
from physics_bridge import PhysicsBridge
from ros_env.srv import BoxSpace, BoxSpaceResponse
from action_servers.moveit_action_server import MoveitActionServer
from physics_bridge.srv import SetFloat
from std_srvs.srv import SetBool
from std_msgs.msg import Float64Stamped

class RealBridge(PhysicsBridge):

    def __init__(self, name = 'physics_bridge'):

        self._step_time = 80

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        super(RealBridge, self).__init__("real", name)

    def _register_object(self, topic, name, params):

        self._init_sensors(topic, name, params['sensors'])

        self._init_actuators(topic, name, params['actuators'])

        return True
    
    def _init_sensors(self, topic, name, sensors):
        for sensor in sensors:
            topic_list = sensors[sensor]
            self._sensor_buffer[sensor] = [0.0]*len(topic_list)
            for idx, real_sensor_name in enumerate(topic_list):
                enable_sensor = rospy.ServiceProxy(name + "/" + real_sensor_name + "/enable", SetBool)
                response = enable_sensor(True)
                if response.success:
                    self._sensor_subscribers.append(rospy.Subscriber(name + "/" + real_sensor_name + "/value",
                        Float64Stamped, functools.partial(self._sensor_callback, sensor=sensor, pos=idx)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, BoxSpace, functools.partial(self._sensor_service, sensor=sensor)))
    
    def _init_actuators(self, topic, name, actuators):
      set_action_srvs = []
      for actuator in actuators:
          if actuator["type"].lower() == "moveit":
              MoveitActionServer(topic, name, actuator)
          set_action_srvs.append(rospy.ServiceProxy(name + "/" + topic + "/set_position", SetFloat))
          get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, BoxSpace)
          self._actuator_services[actuator] = (get_action_srv, set_action_srvs)

    def _sensor_callback(self, data, sensor, pos):
        self._sensor_buffer[sensor][pos] = data.data
    
    def _sensor_service(self, req, sensor):
        return BoxSpaceResponse(self._sensor_buffer[sensor])

    def _step(self):

        for actuator in self._actuator_services:
            (get_action_srv, set_action_srvs) = self._actuator_services[actuator]
            actions = get_action_srv()
            for idx, set_srv in enumerate(set_action_srvs):
                response = set_srv(actions.value[idx])
                if not response.success:
                    rospy.logwarn("Not all actions for %s could be set", actuator)

        return True

    def _reset(self):
        return True

    def _close(self):
        return True