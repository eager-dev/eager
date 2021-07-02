#!/usr/bin/env python3

# ROS packages required
import rospy
from random import random
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine

from eager_core.utils.env_checker import check_env


def reset_func(env: EagerEnv):
    for obj in env.objects:
        if obj.name == 'can':
            # WeBots cannot handle exactly 0 rotations
            states = dict(position=[random(), 0, random()], orientation=[0.0000001, 0, 0, 0.9999999])
            obj.reset(states)


if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine(physics_step=0.01, seed=42)

    # Initialize environment
    robot = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
    can = Object.create('can', 'eager_solid_other', 'can', position=[0, 0, 1])
    env = EagerEnv(engine=engine, objects=[robot, can], name='ros_env', reset_fn=reset_func, max_steps=20)
    env = Flatten(env)
    check_env(env)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    env.close()
