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
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_real.real_engine import RealEngine  # noqa: F401

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor
from gym import spaces  # todo: can we avoid having to import this?

# Dummy reward function - Here, we output a batch reward for each ur5e.
# Anything can be defined here. All observations for each object in "objects" is included in obs
def reward_fn(obs):
    rwd = []
    for obj in obs:
        if 'robot' in obj:
            rwd.append(-(obs[obj]['joint_sensors'] ** 2).sum())
    return rwd


if __name__ == '__main__':

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    # todo: safely start from home position
    # todo: safely go to sleep position on env.close()
    engine = RealEngine()

    # Create robot
    # todo: change to Viper
    # todo: add calibrated position & orientation
    robot = Object.create('robot', 'eager_robot_ur5e', 'ur5e')

    # Add action preprocessing
    # todo: clean-up processor arguments with object config files ('ur5e', 'viperX') inside action processor ROS package
    process_args = SafeActionsProcessor(moveit_package='ur5_e_moveit_config',
                                        urdf_path='$(find ur_e_description)/urdf/ur5e_robot.urdf.xacro',
                                        joint_names=['shoulder_pan_joint',
                                                     'shoulder_lift_joint',
                                                     'elbow_joint',
                                                     'wrist_1_joint',
                                                     'wrist_2_joint',
                                                     'wrist_3_joint'],
                                        group_name='manipulator',
                                        duration=0.1,
                                        object_frame='base_link',
                                        checks_per_rad=15,
                                        vel_limit=2.0,
                                        robot_type='ur5e',
                                        collision_height=0.01,
                                        base_length=0.4,
                                        workspace_length=2.4,
                                        )
    robot.actuators['joints'].add_preprocess(
        launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
        launch_args=process_args.__dict__,
        observations_from_objects=[robot],
        action_space=spaces.Box(low=-3.14, high=3.14, shape=(6,)))

    # Add a camera for rendering
    # todo: add calibrated position & orientation
    cam = Object.create('cam', 'eager_sensor_realsense', 'd435')

    # Create environment
    env = EagerEnv(engine=engine,
                   objects=[robot, cam],
                   name='demo_env',
                   render_obs=cam.sensors['camera_rgb'].get_obs,
                   max_steps=100,
                   reward_fn=reward_fn)
    env = Flatten(env)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        # todo: implement render modes ("human", "rgb_array")
        env.render()
        if done:
            obs = env.reset()

    env.close()
