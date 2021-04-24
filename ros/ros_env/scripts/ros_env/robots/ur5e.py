import gym, gym.spaces
from ..ros_env import Robot, Sensor, Actuator

class UR5eJointActuators(Actuator):

    def __init__(self, name: str) -> None:
        super().__init__(name)
    
    def action_space(self) -> gym.Space:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(7))

class UR5eJointSensors(Sensor):

    def __init__(self, name: str) -> None:
        super().__init__(name)
    
    def observation_space(self) -> gym.Space:
        return gym.spaces.Box(low=-1.0, high=1.0, shape=(7))


class UR5e(Robot):
    
    def __init__(self, name: str) -> None:

        sensors = [
            UR5eJointSensors("joint_sensors")
        ]

        actuators = [
            UR5eJointActuators("joints")
        ]




        super().__init__(name, sensors, actuators)