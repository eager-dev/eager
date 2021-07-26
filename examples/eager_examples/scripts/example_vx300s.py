#!/usr/bin/env python3

import rospy
import numpy as np
from gym import spaces
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_gazebo.gazebo_engine import GazeboEngine  # noqa: F401
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor

from eager_core.utils.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('vx300s_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = GazeboEngine(seed=42, gui=True)

    # Initialize environment
    robot = Object.create('vx300s', 'eager_robot_vx300s', 'vx300s')

    process_args = SafeActionsProcessor(moveit_package='eager_robot_vx300s',
                                        urdf_path='$(find interbotix_xsarm_descriptions)/urdf/vx300s.urdf.xacro',
                                        joint_names=['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle',
                                                     'wrist_rotate'],
                                        group_name='manipulator',
                                        duration=0.1,
                                        object_frame='vx300s/base_link',
                                        checks_per_rad=15,
                                        vel_limit=2.0,
                                        robot_type='vx300s',
                                        )
    robot.actuators['joints'].add_preprocess(
            launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
            launch_args=process_args.__dict__,
            observations_from_objects=[robot],
            action_space=spaces.Box(low=-np.pi, high=np.pi, shape=(6,)))


    env = EagerEnv(engine=engine, objects=[robot], name='ros_env')
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
