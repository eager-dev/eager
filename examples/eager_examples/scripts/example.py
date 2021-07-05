#!/usr/bin/env python3

import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine  # noqa: F401
from eager_bridge_gazebo.gazebo_engine import GazeboEngine  # noqa: F401
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401

from eager_core.utils.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine(physics_step=0.01, seed=42)
    # engine = GazeboEngine(seed=42)
    # engine = PyBulletEngine(dt=0.0165)

    # Initialize environment
    robot = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e', fixed_base=False)
    robot2 = Object.create('ur5e2', 'eager_robot_ur5e', 'ur5e', position=[1, 0, 0], self_collision=False)
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
