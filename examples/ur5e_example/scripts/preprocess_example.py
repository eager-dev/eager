#!/usr/bin/env python3

# ROS packages required
import rospy
import time
from ros_gym_core.ros_env import RosEnv
from ros_gym_core.objects import Robot
from ros_gym_core.wrappers.flatten import Flatten
from gym.spaces import space
from ros_gym_bridge_gazebo.gazebo_engine import GazeboEngine
import gym, gym.spaces

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':
    rospy.init_node('ur5e_example',
                    anonymous=True, log_level=rospy.WARN)

    engine = GazeboEngine()
    
    # Initialize environment
    env_name = 'ros_env'
    robot_name= 'ur5e1'
    ur5e1 = Robot.create('ur5e1', 'ros_gym_robot_ur5e', 'ur5e')
    ur5e1.actuators["joints"].add_preprocess(processed_space=gym.spaces.Box(low=-3.14, high=3.14, shape=(6,)), 
                                             launch_path='$(find ros_gym_process_safe_actions)/launch/safe_actions.launch',
                                             env_name=env_name,
                                             robot_name=robot_name,
                                             moveit_package='ur5_e_moveit_config',
                                             sensor_topic='joint_sensors',
                                             actuator_topic='joints',
                                             joint_names=['shoulder_pan_joint',
                                                          'shoulder_lift_joint',
                                                          'elbow_joint',
                                                          'wrist_1_joint',
                                                          'wrist_2_joint',
                                                          'wrist_3_joint'],
                                             group_name='manipulator')
    
    env = Flatten(RosEnv(engine=engine, robots=[ur5e1], name=env_name))
    
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
