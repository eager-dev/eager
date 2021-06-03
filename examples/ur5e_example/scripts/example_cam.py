#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.ros_env import RosEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e_cam.wbt')

    # Initialize environment
    env = RosEnv(engine=engine, objects=[
        Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e'),
        Object.create('ms21', 'eager_sensor_multisense_s21', 'dual_cam')
        ],
        name='ros_env')
    env = Flatten(env)
    check_env(env)

    rospy.loginfo("Training starts")

    #model = PPO('MlpPolicy', env, verbose=1)
    
    #model.learn(total_timesteps=100000)

    obs = env.reset()
    
    for i in range(10000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    
    env.close()
