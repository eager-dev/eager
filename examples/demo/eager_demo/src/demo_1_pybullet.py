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
from eager_core.utils.file_utils import launch_roscore, load_yaml
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor


# Dummy reward function - Here, we output a batch reward for each ur5e.
# Anything can be defined here. All observations for each object in "objects" is included in obs
def reward_fn(obs):
    rwd = []
    for obj in obs:
        if 'robot' in obj:
            rwd.append(-(obs[obj]['joint_pos'] ** 2).sum())
    return rwd


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = PyBulletEngine(gui=True)

    # Create robot
    robot = Object.create('robot', 'eager_robot_vx300s', 'vx300s')
    # Add action preprocessing
    processor = SafeActionsProcessor(duration=0.5,
                                     checks_per_rad=15,
                                     vel_limit=0.25,
                                     robot_type='vx300s',
                                     collision_height=0.1,
                                     )
    robot.actuators['joints'].add_preprocess(
        processor=processor,
        observations_from_objects=[robot],
    )

    # Add a camera for rendering
    calibration = load_yaml('eager_demo', 'calibration')
    cam = Object.create('cam', 'eager_sensor_realsense', 'd435',
                        position=calibration['position'],
                        orientation=calibration['orientation'],
                        )

    # Create environment
    env = EagerEnv(engine=engine,
                   objects=[robot, cam],
                   name='demo_env',
                   render_sensor=cam.sensors['camera_rgb'],
                   max_steps=100,
                   reward_fn=reward_fn,
                   )
    env = Flatten(env)

    obs = env.reset()  # TODO: if code does not close properly, render seems to keep a thread open....
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        rgb = env.render()
        if done:
            obs = env.reset()

    # todo: create a env.close(): close render screen, and env.shutdown() to shutdown the environment cleanly.
    env.close()
    exit(1)
