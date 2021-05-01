import gym, gym.spaces
from gym.spaces import space
from ..ros_env import Robot, Sensor, Actuator

class UR5e(Robot):
    
    def __init__(self, name: str) -> None:

        sensors = [
            Sensor("joint_sensors", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(7,)))
        ]

        actuators = [
            Actuator("joints", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(7,)))
        ]

        super().__init__(name, sensors, actuators)