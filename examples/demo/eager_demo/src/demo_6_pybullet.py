#!/usr/bin/env python3

import rospy
import numpy as np

# Import eager packages
from eager_core.utils.file_utils import launch_roscore, load_yaml
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401
from eager_core.utils.file_utils import substitute_xml_args
from stable_baselines3 import PPO
from gym import spaces

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor

class NormalizeActions(object):
    """
    if and(low, high) not np.inf: scales action from [-1, 1] back to the unnormalized action
    if or(low,high) np.inf: no normalization of the actions, and true action must be used"""

    def __init__(self, env):
        self._env = env
        low, high = env.action_space.low, env.action_space.high
        self._enabled = np.logical_and(np.isfinite(low), np.isfinite(high))
        self._low = np.where(self._enabled, low, -np.ones_like(low))
        self._high = np.where(self._enabled, high, np.ones_like(low))

    def __getattr__(self, name):
        return getattr(self._env, name)

    @property
    def action_space(self):
        space = self._env.action_space
        low = np.where(self._enabled, -np.ones_like(space.low), space.low)
        high = np.where(self._enabled, np.ones_like(space.high), space.high)
        return spaces.Box(low, high, dtype=space.dtype)

    def step(self, action):
        # de-normalize action
        # action = (action + 1) / 2 * (self._high - self._low) + self._low
        action = self.denormalize_action(action)

        # apply action
        obs, reward, done, info = self._env.step(action)

        # normalize applied action (in case action was above maximum)
        # info['action'] = (2*info['action'] - (self._high + self._low))/(self._high - self._low)
        # info['action'] = self.normalize_action(info['action'])
        return obs, reward, done, info

    def denormalize_action(self, action):
        return (action + 1) / 2 * (self._high - self._low) + self._low

    def normalize_action(self, action):
        return (2*action - (self._high + self._low))/(self._high - self._low)

def reward_fn(obs):
    reward = -np.sum(np.power(obs['robot']['joint_pos'],2))
    return reward

if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = PyBulletEngine(gui=False)

    # Create robot
    robot = Object.create('robot', 'eager_robot_vx300s', 'vx300s')
    # Add action preprocessing
    processor = SafeActionsProcessor(robot_type='vx300s',
                                     vel_limit=0.25,
                                     collision_height=0.15,
                                     duration=0.08
                                     )
    robot.actuators['joints'].add_preprocess(
        processor=processor,
        observations_from_objects=[robot],
    )

    # Create environment
    env = EagerEnv(name='demo6',
                   engine=engine,
                   objects=[robot],
                   reward_fn=reward_fn,
                   max_steps=200,
                   )
    env = Flatten(env)
    env = NormalizeActions(Flatten(env))

    env.render()
    obs = env.reset()  # TODO: if code does not close properly, render seems to keep a thread open....
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=substitute_xml_args("$(find eager_demo)/logs/demo"))
    model.learn(total_timesteps=100000)
    save_path = substitute_xml_args("$(find eager_demo)/models/demo.zip")
    model.save(save_path)
    # todo: create a env.close(): close render screen, and env.shutdown() to shutdown the environment cleanly.
    env.close()
