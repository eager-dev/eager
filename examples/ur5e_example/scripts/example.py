#!/usr/bin/env python3

# ROS packages required
import rospy
from ros_gym_core.ros_env import RosEnv
from ros_gym_core.objects import Robot
from ros_gym_core.wrappers.flatten import Flatten
from ros_gym_bridge_webots.webots_engine import WebotsEngine
from ros_gym_bridge_gazebo.gazebo_engine import GazeboEngine

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e.wbt')
    # engine = GazeboEngine()

    # Initialize environment
    env = RosEnv(engine=engine, robots=[Robot.create('ur5e1', 'ros_gym_robot_ur5e', 'ur5e')], name='ros_env')
    env = Flatten(env)
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
