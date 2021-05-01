#!/usr/bin/env python3

# ROS packages required
import rospy
import time
from ros_env.ros_env import RosEnv
from ros_env.robots.ur5e import UR5e

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env


if __name__ == '__main__':
    rospy.init_node('ur5e_example',
                    anonymous=True, log_level=rospy.WARN)

    env = RosEnv(robots=[UR5e("ur5e1")])

    print(env.observation_space)

    time.sleep(10)

    check_env(env)

    rospy.loginfo("Training starts")

    model = PPO('MlpPolicy', env, verbose=1)
    
    model.learn(total_timesteps=100000)

    obs = env.reset()
    
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    
    env.close()
