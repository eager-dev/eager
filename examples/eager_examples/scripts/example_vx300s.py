#!/usr/bin/env python3
"""
    Example for training the vx300s to move to a desired end-effector location
"""
import rospy
import numpy as np
from gym import spaces
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_gazebo.gazebo_engine import GazeboEngine  # noqa: F401
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor
from random import random

from eager_core.utils.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('vx300s_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = GazeboEngine(seed=42, gui=True)

    # Initialize environment

    # Create an Interbotix vx300s
    robot = Object.create('vx300s',
                          'eager_robot_vx300s',
                          'vx300s')

    # Create a coke can
    can = Object.create('can',
                        'eager_solids_other',
                        'can',
                        position=[0.5, 0.0, 0.0],
                        orientation=[0.0, 0.0, 0.0, 1.0]
                        )

    # Add a processing step such that actions are save
    process_args = SafeActionsProcessor(moveit_package='eager_robot_vx300s',
                                        urdf_path='$(find interbotix_xsarm_descriptions)/urdf/vx300s.urdf.xacro',
                                        joint_names=['waist', 'shoulder', 'elbow',
                                                     'forearm_roll', 'wrist_angle', 'wrist_rotate'],
                                        group_name='interbotix_arm',
                                        duration=0.5,
                                        object_frame='vx300s/base_link',
                                        checks_per_rad=15,
                                        vel_limit=2.0,
                                        robot_type='vx300s',
                                        collision_height=0.1,
                                        base_length=0.4,
                                        workspace_length=2.4,
                                        )

    robot.actuators['joints'].add_preprocess(
            launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
            launch_args=process_args.__dict__,
            observations_from_objects=[robot],
            action_space=spaces.Box(low=-np.pi, high=np.pi, shape=(6,)))

    # Create a reset function
    def reset_func(env: EagerEnv):
        for obj in env.objects:
            if obj.name == 'can':
                # The coke can will be placed at a random location in front of the robot
                x = 0.2 + random() * 0.4
                y = 0.2 - random() * 0.4
                z = 0.0
                states = dict(position=[x, y, z], orientation=[0.0, 0.0, 0.0, 1.0])
                obj.reset(states)

    def is_done_func(env: EagerEnv):
        return env.steps >= env.STEPS_PER_ROLLOUT

    env = EagerEnv(engine=engine, objects=[robot, can], name='ros_env', reset_fn=reset_func, is_done_fn=is_done_func,
                   max_steps=1000)
    env = Flatten(env)

    check_env(env)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()

    env.close()
