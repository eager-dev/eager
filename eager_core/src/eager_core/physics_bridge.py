import abc
import rospy
import ast
from eager_core.srv import Register, StepEnv, ResetEnv
from eager_core.msg import Seed
from eager_core.utils.file_utils import load_yaml

# Abstract Base Class compatible with Python 2 and 3
ABC = abc.ABCMeta('ABC', (object,), {'__slots__': ()})


class PhysicsBridge(ABC):

    def __init__(self, bridge_type):
        self._bridge_type = bridge_type
        self.__register_service = rospy.Service('register', Register, self.__register_handler)
        self.__step_service = rospy.Service('step', StepEnv, self.__step_handler)
        self.__reset_service = rospy.Service('reset', ResetEnv, self.__reset_handler)
        self.__seed_subscriber = rospy.Subscriber('seed', Seed, self.__seed_handler)
        rospy.on_shutdown(self._close)

    @abc.abstractmethod
    def _register_object(self, topic, name, package, object_type, args, config):
        pass

    @abc.abstractmethod
    def _step(self):
        pass

    @abc.abstractmethod
    def _reset(self):
        pass

    @abc.abstractmethod
    def _close(self):
        pass

    @abc.abstractmethod
    def _seed(self, seed):
        pass

    def __register_handler(self, req):
        for object in req.objects:
            object_type = object.type.split('/')
            args = ast.literal_eval(object.args)
            params = load_yaml(object_type[0], object_type[1])
            if self._bridge_type not in params:
                str_err = ('Cannot register object "%s",' % object.name +
                           ' because config (.yaml) file of object is not defined for "%s".' % self._bridge_type)
                rospy.logfatal(str_err)
                raise Exception(str_err)
            br_params = params[self._bridge_type]
            if 'sensors' in br_params:
                for sensor in br_params['sensors']:
                    br_params['sensors'][sensor]['space'] = params['sensors'][sensor]
            if 'actuators' in br_params:
                for actuator in br_params['actuators']:
                    br_params['actuators'][actuator]['space'] = params['actuators'][actuator]
            if 'states' in br_params:
                for state in br_params['states']:
                    br_params['states'][state]['space'] = params['states'][state]

            self._register_object("objects/" + object.name, object.name, object_type[0], object_type[1], args, br_params)

        return ()  # Success

    def __step_handler(self, req):
        if self._step():
            return False  # Success
        else:
            return None  # Error

    def __reset_handler(self, req):
        if self._reset():
            return ()  # Success
        else:
            return None  # Error

    def __seed_handler(self, data):
        self._seed(data.seed)
