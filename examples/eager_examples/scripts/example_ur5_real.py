#!/usr/bin/env python3

# Copyright 2021 - present, OpenDR European Project

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ROS packages required
import rospy
from eager_core.eager_env import BaseEagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_real.real_engine import RealEngine
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor

from gym import spaces
import numpy as np
from stable_baselines3 import PPO


class MyEnv(BaseEagerEnv):

    def __init__(self, engine, name="my_env"):
        super().__init__(engine, name=name)

        self.STEPS_PER_ROLLOUT = 100
        self.steps = 0

        # Create ur5e object
        self.ur5e = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e', position=[0, 0, 0])

        # Add preprocessing so that commanded actions are safe
        process_args = SafeActionsProcessor(moveit_package='ur5_moveit_config',
                                            urdf_path='$(find ur_description)/urdf/ur5_robot.urdf.xacro',
                                            joint_names=['shoulder_pan_joint',
                                                         'shoulder_lift_joint',
                                                         'elbow_joint',
                                                         'wrist_1_joint',
                                                         'wrist_2_joint',
                                                         'wrist_3_joint'],
                                            group_name='manipulator',
                                            duration=0.5,
                                            object_frame='base_link',
                                            checks_per_rad=15,
                                            vel_limit=0.05,
                                            robot_type='ur5e',
                                            collision_height=0.1,
                                            base_length=0.4,
                                            workspace_length=2.4,
                                            )
        self.ur5e.actuators['joints'].add_preprocess(
            launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
            launch_args=process_args.__dict__,
            observations_from_objects=[self.ur5e],
            action_space=spaces.Box(low=-np.pi, high=np.pi, shape=(6,)))

        # Initialize all the services of the robots
        self._init_nodes([self.ur5e])

        # Define the spaces
        self.observation_space = self.ur5e.observation_space
        self.action_space = self.ur5e.action_space

    def step(self, action):
        # Set actions before stepping
        self.ur5e.set_action(action)

        # Step the environment
        self._step()
        self.steps += 1

        # Get observations
        obs = self.ur5e.get_obs()

        return obs, self._get_reward(obs), self._is_done(obs), self.ur5e.get_state()

    def _get_reward(self, obs):
        # Quadratic reward - move to goal position [0, -np.pi/2, 0, 0, 0, 0]
        return -((obs['joint_sensors'] - np.array([0, -np.pi / 2, 0, 0, 0, 0], dtype='float32')) ** 2).sum()

    def _is_done(self, obs):
        return self.steps >= self.STEPS_PER_ROLLOUT

    def reset(self) -> object:
        pass


if __name__ == '__main__':

    rospy.init_node('example_safe_actions', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = RealEngine(dt=0.3)

    # Create environment
    env = MyEnv(engine, name="my_env")
    env = Flatten(env)

    env.seed(42)

    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)

    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=100000)

    env.close()
