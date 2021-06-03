#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.ros_env import RosEnv
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
    engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e.wbt')
    # engine = GazeboEngine()
    # engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false')

    # Initialize environment
    env = RosEnv(engine=engine, objects=[Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')], name='ros_env')
    env = Flatten(env)
    # todo: As of now, check_env fails because we do not wrap the angles of the joints to [-pi, pi].
    #  This causes the observations to sometimes not be in the observation_space. This was a problem before,
    #  but went unnoticed because we always initialized at zero, and the few actions check_env took, never
    #  steered it outside of the observation_space. New issue: how to implement the wrapping?
    # check_env(env)

    rospy.loginfo("Training starts")

    env.close()
    """
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
    """
