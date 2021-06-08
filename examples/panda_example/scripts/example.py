#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine

import pybullet_data

if __name__ == '__main__':

    rospy.init_node('panda_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false', dt=0.0165)

    # Initialize environment
    print("line20")
    robot = Object.create('panda1', 'eager_robot_panda', 'panda')
    print("line22")
    env = EagerEnv(engine=engine, objects=[robot], name='ros_env')
    print("line24")
    env = Flatten(env)
    check_env(env)

    rospy.loginfo("Training starts")
    
    #model = PPO('MlpPolicy', env, verbose=1)

    # model.learn(total_timesteps=100000)

    obs = env.reset()
    for i in range(100000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    env.close()
