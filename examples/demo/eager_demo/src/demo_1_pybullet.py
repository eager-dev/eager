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
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401


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
    engine = PyBulletEngine(gui=True)

    # Create robot
    # todo: change to Viper
    # todo: add calibrated position & orientation
    robot = Object.create('robot', 'eager_robot_ur5e', 'ur5e')

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
