import rospy
import functools
from physics_bridge import PhysicsBridge
from ros_env.srv import BoxSpace, BoxSpaceResponse
from action_servers.moveit_action_server import MoveitActionServer
from physics_bridge.srv import SetFloat

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
            self._sensor_buffer[sensor] = []
            msg_topic = sensor["topic"]
            msg_name = sensor["msg_name"]
            msg_package = __import__(sensor["msg_package"]),
            msg_type = getattr(msg_package, msg_name)
            self._sensor_subscribers.append(rospy.Subscriber(
                msg_topic,
                msg_type, 
                functools.partial(self._sensor_callback, sensor=sensor)))
            self._sensor_services.append(rospy.Service(msg_topic + "/" + sensor, BoxSpace, functools.partial(self._sensor_service, sensor=sensor)))
    
    def _init_actuators(self, topic, name, actuators):
      set_action_srvs = []
      for actuator in actuators:
          if actuator["type"].strip().lower() == "moveit":
              MoveitActionServer(topic, name, actuator)
          else:
              rospy.logerr("Currently only actuators of type \"MoveIt\" are implemented")
          set_action_srvs.append(rospy.ServiceProxy(name + "/" + actuator + "/set_action", SetFloat))
          get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, BoxSpace)
          self._actuator_services[actuator] = (get_action_srv, set_action_srvs)

    def _sensor_callback(self, data, sensor):
        self._sensor_buffer[sensor] = data.data
    
    def _sensor_service(self, req, sensor):
        return BoxSpaceResponse(self._sensor_buffer[sensor])

    def _step(self):

        for actuator in self._actuator_services:
            (get_action_srv, set_action_srv) = self._actuator_services[actuator]
            actions = get_action_srv()
            response = set_action_srv(actions.value)
            if not response.success:
                rospy.logwarn("Not all actions for %s could be set", actuator)

        return True

    def _reset(self):
        return True

    def _close(self):
        return True