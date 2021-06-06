#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import pybullet_data
import time
import numpy as np

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    # engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e_cam.wbt')
    engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false')

    # Initialize environment
    cam = Object.create('ms21', 'eager_sensor_multisense_s21', 'dual_cam')
    ur5e1 = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
    env = EagerEnv(engine=engine, objects=[ur5e1, cam], name='ros_env', render_obs=cam.sensors['camera_right'].get_obs)
    env = Flatten(env)
    check_env(env)

    rospy.loginfo("Training starts")

    #model = PPO('MlpPolicy', env, verbose=1)
    
    #model.learn(total_timesteps=100000)

    num_steps = 100
    for i in range(3):
        times = np.zeros(num_steps)
        frames = []
        obs = env.reset()
        for i in range(num_steps):
            start = time.time()
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            frames.append(env.render())
            if done:
                obs = env.reset()

            stop = time.time()
            duration = (stop - start)
            if (duration):
                fps = 1. / duration
            else:
                fps = 0
            times[i] = fps
        print("mean: {0} for {1} steps".format(np.mean(times), num_steps))
    
    env.close()
