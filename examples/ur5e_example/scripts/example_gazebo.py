#!/usr/bin/env python3

# ROS packages required
import rospy
import time
from ros_gym_core.ros_env import RosEnv
from ros_gym_robot_ur5e.ur5e import UR5e
from ros_gym_core.wrappers.flatten import Flatten
from gym.spaces import space
import gym, gym.spaces

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
    ur5e1 = UR5e("ur5e1")
    ur5e1.actuators["joints"].add_preprocess(processed_space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)), 
                                             launch_path='$(find safe_actions)/launch/safe_actions.launch',
                                             node_type='service', 
                                             stateless=True,
                                             group_name='manipulator',
                                             checks_per_rad=25,
                                             vel_limit=3.14,
                                             step_time=0.1,
                                             duration=0.5)
    
    env = Flatten(RosEnv(robots=[ur5e1], name='ros_env', engine_params=gb_params))
    
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
