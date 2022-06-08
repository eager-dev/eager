#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.eager_env import BaseEagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_core.utils.file_utils import launch_node
from eager_bridge_webots.webots_engine import WebotsEngine  # noqa: F401
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor

from gym import spaces
import numpy as np
from stable_baselines3 import PPO


class MyEnv(BaseEagerEnv):

    def __init__(self, engine, name="custom_env"):
        super().__init__(engine, name=name)

        self.STEPS_PER_ROLLOUT = 100
        self.steps = 0

        # Create ur5e robot
        self.ur5e = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')

        # Add preprocessing so that commanded actions are safe
        processor = SafeActionsProcessor(duration=0.1,
                                         checks_per_rad=15,
                                         vel_limit=3.0,
                                         robot_type='ur5e',
                                         collision_height=0.01,
                                         )
        self.ur5e.actuators['joints'].add_preprocess(
            processor=processor,
            observations_from_objects=[self.ur5e],
            action_space=spaces.Box(low=-np.pi, high=np.pi, shape=(6,)))
        self.camera = Object.create('ms21', 'eager_sensor_multisense_s21', 'dual_cam',
                                    position=[1.0, 0.0, 0.65],
                                    orientation=[-0.1305262, 0.0, 0.9914449, 0.0],
                                    )

        self._init_nodes([self.camera, self.ur5e])

        self.observation_space = self.ur5e.observation_space
        self.action_space = self.ur5e.action_space

    def step(self, action):
        # Set actions before stepping
        self.ur5e.set_action(action)

        # Step the environment
        self._step()
        self.steps += 1

        # Get observations
        obs = self.ur5e.get_obs()

        return obs, self._get_reward(obs), self._is_done(obs), self.ur5e.get_state()

    def reset(self) -> object:
        self.steps = 0

        # Set desired reset state
        reset_states = dict()
        reset_states['joint_pos'] = np.array([0, -np.pi / 2, 0, 0, 0, 0], dtype='float32')
        reset_states['joint_vel'] = np.array([0, 0, 0, 0, 0, 0], dtype='float32')
        self.ur5e.reset(states=reset_states)

        # Reset the environment
        self._reset()

        # Get new observations
        return self.ur5e.get_obs()

    def _get_reward(self, obs):
        return -(obs['joint_pos'][5] - 2)**2

    def _is_done(self, obs):
        return self.steps >= self.STEPS_PER_ROLLOUT


if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine()
    # engine = PyBulletEngine()

    env = MyEnv(engine, name="my_env")
    env = Flatten(env)

    env.seed(42)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=100000)

    env.close()
