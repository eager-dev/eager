#!/usr/bin/env python3

import rospy

# Import eager packages
from eager_core.utils.file_utils import launch_roscore, load_yaml
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401
from eager_bridge_real.real_engine import RealEngine  # noqa: F401

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = RealEngine()

    # Create robot

    # Add preprocessing

    # Add a camera for rendering

    # Create environment
    env = EagerEnv()
    env = Flatten(env)

    env.render()
    obs = env.reset()
    for i in range(200):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

    env.close()
