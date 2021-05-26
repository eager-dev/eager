#!/usr/bin/env python3

# ROS packages required
import rospy
import time
from ros_gym_core.ros_env import RosEnv
from ros_gym_robot_ur5e.ur5e import UR5e
from ros_gym_core.wrappers.flatten import Flatten

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':
    rospy.init_node('ur5e_example',
                    anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    gb_params = dict()
    gb_params['bridge_type'] = 'gazebo'
    gb_params['launch_file'] = '$(find ros_gym_bridge_gazebo)/launch/gazebo.launch'
    gb_params['no_gui'] = 'false'
    gb_params['world'] = '$(find ros_gym_bridge_gazebo)/worlds/ros_gym_empty.world'
    gb_params['time_step']  = 0.001
    gb_params['max_update_rate'] = 0.0 # 0.0 means simulate gazebo fast as possible

    # Initialize environment
    env = Flatten(RosEnv(robots=[UR5e("ur5e1")], name='ros_env', engine_params=gb_params))

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