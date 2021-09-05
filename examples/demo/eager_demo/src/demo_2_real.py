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
from eager_core.utils.file_utils import load_yaml
from eager_bridge_real.real_engine import RealEngine  # noqa: F401

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor


# Dummy reward function - Here, we output a batch reward for each robot.
# Anything can be defined here. All observations for each object in "objects" is included in obs
def reward_fn(obs):
    rwd = []
    for obj in obs:
        if 'robot' in obj:
            rwd.append(-(obs[obj]['joint_pos'] ** 2).sum())
    return rwd


if __name__ == '__main__':

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)

    # Define the engine
    engine = RealEngine(dt=0.4)

    # Create robot
    robot = Object.create('robot', 'eager_robot_vx300s', 'vx300s',
                          position=[0, 0, 0],
                          orientation=[0, 0, 0, 1],
                          )

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
    # First load calibrated position & orientation
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
                   max_steps=10,
                   reward_fn=reward_fn)
    env = Flatten(env)

    obs = env.reset()
    for i in range(20):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        # todo: implement render modes ("human", "rgb_array")
        env.render()
        if done:
            obs = env.reset()

    env.close()
