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
    wb_params = dict()
    wb_params['bridge_type'] = 'webots'
    wb_params['launch_file'] = '/home/akeijzer/ros_gym_ws/src/ros-gym/ros_gym_bridge_webots/launch/%s.launch' % wb_params['bridge_type']
    wb_params['mode'] = 'fast'
    wb_params['no_gui'] = 'false'
    wb_params['world'] = '$(find ur5e_example)/worlds/ur5e'

    # Initialize environment
    env = Flatten(RosEnv(robots=[UR5e("ur5e1")], name='ros_env', engine_params=wb_params))

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
