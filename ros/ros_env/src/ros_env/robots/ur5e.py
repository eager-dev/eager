import gym, gym.spaces
from gym.spaces import space
from ..ros_env import Robot, Sensor, Actuator

class UR5e(Robot):
    
    def __init__(self, name: str) -> None:

        sensors = [
            Sensor(None, "joint_sensors", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(6,)))
        ]

        actuators = [
            Actuator(None, "joints", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(6,)))
        ]

        super().__init__("ur5e", name, sensors, actuators)