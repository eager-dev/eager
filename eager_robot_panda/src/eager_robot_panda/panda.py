import gym, gym.spaces
from gym.spaces import space
from eager_core.objects import Object, Sensor, Actuator

class Panda(Object):
    
    def __init__(self, name: str) -> None:

        sensors = [
            Sensor(None, "joint_sensors", space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)))
        ]

        actuators = [
            Actuator(None, "joints", space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)))
        ]


        super().__init__("eager_robot_panda/panda", name, sensors, actuators)
