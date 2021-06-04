#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.ros_env import RosEnv
from eager_core.objects import Robot
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine
from eager_bridge_gazebo.gazebo_engine import GazeboEngine
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine
from eager_core.msg import Object
import gym, gym.spaces
import pybullet_data

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':
    rospy.init_node('ur5e_example',
                    anonymous=True, log_level=rospy.WARN)
    
    dt = 0.08
    engine = WebotsEngine(dt=dt, world='$(find ur5e_example)/worlds/ur5e.wbt')
    # engine = GazeboEngine(dt=dt)
    # engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false')
    
    # Initialize environment
    env_name = 'ros_env'
    robot_name = 'ur5e1'
    robot_type = 'ur5e'
    
    process_args = {}
    process_args['moveit_package'] = 'ur5_e_moveit_config'
    process_args['joint_names'] = ['shoulder_pan_joint',
                                     'shoulder_lift_joint',
                                     'elbow_joint',
                                     'wrist_1_joint',
                                     'wrist_2_joint',
                                     'wrist_3_joint'
                                     ]
    process_args['group_name'] = 'manipulator'
    process_args['duration'] = 0.1
    process_args['dt'] = dt
    process_args['object_frame'] = 'base_link'
    process_args['checks_per_rad'] = 25
    process_args['vel_limit'] = 3.0
    process_args['robot_type'] = robot_type
    
    ur5e1 = Robot.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
    ur5e1.actuators["joints"].add_preprocess(launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
                                             launch_args=process_args,
                                             observations_from_objects=[ur5e1]
                                             )
    
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
