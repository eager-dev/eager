import rospy, rosservice
import functools
import re
from ros_gym_core.physics_bridge import PhysicsBridge
from ros_gym_core.srv import BoxSpace, BoxSpaceResponse


class PyBulletBridge(PhysicsBridge):

    def __init__(self, name='pybullet_bridge', timestep=80, frame_skip=1):
        # todo: dt= timestep*frame_skip

        self._step_time = 80

        self._supervisor_name = self._get_supervisor()

        self._step_service = rospy.ServiceProxy(self._supervisor_name + "/robot/time_step", set_int)

        self._sensor_buffer = dict()
        self._sensor_subscribers = []
        self._sensor_services = []

        self._actuator_services = dict()

        super(PyBulletBridge, self).__init__("pybullet", name)

    def _get_supervisor(cls):
        supervisor_checks = 0
        supervisors = [x for x in rosservice.get_service_list() if 'supervisor' in x]
        while not supervisors:
            if supervisor_checks > 20:
                rospy.logfatal("Could not find WeBots supervisor.")
                raise Exception
            supervisor_checks += 1
            rospy.sleep(1)
            supervisors = [x for x in rosservice.get_service_list() if 'supervisor' in x]

        return re.search("[^\/]+(?=\/supervisor)", supervisors[0]).group()

    def _register_object(self, topic, name, params):
        # type: "px150"
        # name: "px150"
        #todo: add urdf to simulator based on type & name
        #todo: register actuators and sensors based on type & name
        #todo: register object requires more info (e.g. base pose) --> How to get info from ROSenv?

        self._init_sensors(topic, name, params['sensors'])

        self._init_actuators(topic, name, params['actuators'])

        return True

    def _init_sensors(self, topic, name, sensors):
        # todo: register sensors of px150 to joints in pybullet
        robot_sensors = dict()
        for sensor in sensors:
            topic_list = sensors[sensor]
            robot_sensors[sensor] = [0.0] * len(topic_list)
            for idx, webots_sensor_name in enumerate(topic_list):
                enable_sensor = rospy.ServiceProxy(name + "/" + webots_sensor_name + "/enable", set_int)
                success = enable_sensor(self._step_time)
                if success:
                    self._sensor_subscribers.append(rospy.Subscriber(name + "/" + webots_sensor_name + "/value",
                                                                     Float64Stamped,
                                                                     functools.partial(self._sensor_callback, name=name,
                                                                                       sensor=sensor, pos=idx)))
            self._sensor_services.append(rospy.Service(topic + "/" + sensor, BoxSpace,
                                                       functools.partial(self._sensor_service, name=name,
                                                                         sensor=sensor)))
        self._sensor_buffer[name] = robot_sensors

    def _init_actuators(self, topic, name, actuators):
        # todo: register actuators of px150 to joints in pybullet
        robot_actuators = dict()
        for actuator in actuators:
            topic_list = actuators[actuator]
            set_action_srvs = []
            for webots_actuator_name in topic_list:
                set_action_srvs.append(
                    rospy.ServiceProxy(name + "/" + webots_actuator_name + "/set_position", set_float))

            get_action_srv = rospy.ServiceProxy(topic + "/" + actuator, BoxSpace)
            robot_actuators[actuator] = (get_action_srv, set_action_srvs)
        self._actuator_services[name] = robot_actuators

    def _sensor_callback(self, data, name, sensor, pos):
        self._sensor_buffer[name][sensor][pos] = data.data

    def _sensor_service(self, req, name, sensor):
        return BoxSpaceResponse(self._sensor_buffer[name][sensor])

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

        return True

    def _reset(self):
        return True

    def _close(self):
        return True