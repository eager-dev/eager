#!/usr/bin/env python3

import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_gazebo.gazebo_engine import GazeboEngine

from eager_core.utils.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = GazeboEngine(seed=42, gui=True)

    # Initialize environment
    robot = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
    robot2 = Object.create('rs', 'eager_sensor_realsense', 'd435',
                           position=[-0.6318217780002455, 0.10058530736837151, 0.39298725648972427],
                           orientation=[0.9803519522608385, -0.0005924858889668179, -0.19393664231674468, 0.03603161702922013],
                           )
    env = EagerEnv(engine=engine, objects=[robot, robot2], name='ros_env')
    env = Flatten(env)

    check_env(env)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    env.close()
