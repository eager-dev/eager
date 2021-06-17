#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine
from eager_bridge_gazebo.gazebo_engine import GazeboEngine
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import pybullet_data

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine()
    # engine = GazeboEngine()
    # engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false', dt=0.0165)

    # Initialize environment
    robot = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
    robot2 = Object.create('ur5e2', 'eager_robot_ur5e', 'ur5e', position=[1, 0, 0])
    env = EagerEnv(engine=engine, objects=[robot, robot2], name='ros_env')
    env = Flatten(env)
    check_env(env)

    rospy.loginfo("Training starts")
    
    model = PPO('MlpPolicy', env, verbose=1)

    # model.learn(total_timesteps=100000)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    
    env.close()