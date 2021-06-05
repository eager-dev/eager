#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.ros_env import RosEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine

import numpy as np
import pybullet_data
from stable_baselines3 import PPO

if __name__ == '__main__':

    rospy.init_node('example_safe_actions', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false')

    # Create a grid of ur5e robots
    objects = []
    grid = [-1.5, 0, 1.5]
    for x in grid:
        for y in grid:
            idx = len(objects)
            objects.append(Object.create('ur5e%d' % idx, 'eager_robot_ur5e', 'ur5e', position=[x, y, 0]))

    # Add a camera for rendering
    cam = Object.create('ms21', 'eager_sensor_multisense_s21', 'dual_cam')
    objects.append(cam)

    # Create environment
    env = RosEnv(engine=engine, objects=objects, name='multi_env', render_obs=cam.sensors['camera_right'].get_obs, max_steps=100)
    env = Flatten(env)

    env.seed(42)

    rospy.loginfo("Training starts")

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=100000)

    env.close()