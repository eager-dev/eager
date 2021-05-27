import gym, gym.spaces
from gym.spaces import space
from eager_core.objects import Robot, Sensor, Actuator

class Px150(Robot):

    def __init__(self, name: str) -> None:
        sensors = [
            Sensor(None, "joint_sensors", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(5,))),
            Sensor(None, "gripper", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))),
        ]

        actuators = [
            Actuator(None, "joints", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(5,))),
            Actuator(None, "gripper", space=gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))),
        ]

        super().__init__("eager_robot_px150/px150", name, sensors, actuators)