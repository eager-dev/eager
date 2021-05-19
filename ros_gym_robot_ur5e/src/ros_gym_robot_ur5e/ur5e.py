import gym, gym.spaces
from gym.spaces import space
from ros_gym_core.objects import Robot, Sensor, Actuator

class UR5e(Robot):
    
    def __init__(self, name: str) -> None:

        sensors = [
            Sensor(None, "joint_sensors", space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)))
        ]

        actuators = [
            Actuator(None, "joints", space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)))
        ]

        super().__init__("ur5e", name, sensors, actuators)
