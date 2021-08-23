#!/usr/bin/env python3

import rospy
import numpy as np
import kinpy as kp
import os
import random
from gym import spaces
from collections import OrderedDict
from scipy.spatial.transform import Rotation as R
from eager_core.eager_env import BaseEagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_gazebo.gazebo_engine import GazeboEngine  # noqa: F401
# from eager_bridge_webots.webots_engine import WebotsEngine  # noqa: F401
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor
# from eager_core.utils.env_checker import check_env
from std_msgs.msg import Bool
from stable_baselines3 import PPO


class DemoEnv(BaseEagerEnv):

    def __init__(self, engine, name="demo_env"):
        super().__init__(engine, name=name)

        self.STEPS_PER_ROLLOUT = 1000
        self.steps = 0

        # Create a kinematic chain for calculating forward kinematics
        urdf_location = substitute_xml_args("$(find eager_demo)/urdf/vx300s.urdf")
        self.chain = kp.build_chain_from_urdf(open(urdf_location).read())

        # Create an Interbotix vx300s
        self.robot = Object.create('vx300s',
                                   'eager_robot_vx300s',
                                   'vx300s',
                                   )

        # Add a processing step such that actions are save
        process_args = SafeActionsProcessor(moveit_package='eager_robot_vx300s',
                                            urdf_path='$(find interbotix_xsarm_descriptions)/urdf/vx300s.urdf.xacro',
                                            joint_names=['waist', 'shoulder', 'elbow',
                                                         'forearm_roll', 'wrist_angle', 'wrist_rotate'],
                                            group_name='interbotix_arm',
                                            duration=0.5,
                                            object_frame='vx300s/base_link',
                                            checks_per_rad=1,
                                            vel_limit=2.0,
                                            robot_type='vx300s',
                                            collision_height=0.12,
                                            base_length=0.4,
                                            workspace_length=2.4,
                                            )

        # Add preprocessing such that actions are save
        self.robot.actuators['joints'].add_preprocess(
            launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
            launch_args=process_args.__dict__,
            observations_from_objects=[self.robot],
            action_space=spaces.Box(low=-3.14, high=3.14, shape=(6,)),
            )

        # Create a can
        self.can = Object.create('can',
                                 'eager_solid_other',
                                 'can',
                                 position=[0.4 + random.random() * 0.3, 0.2 - random.random() * 0.4, 0.0],
                                 orientation=[0.0, 0.0, 0.0, 1.0],
                                 )
        self._init_nodes([self.robot, self.can])
        obs_spaces = OrderedDict()
        obs_spaces['vx300s'] = self.robot.observation_space
        obs_spaces['can'] = spaces.Box(low=-3.4e38, high=3.4e38, shape=(2,))
        self.observation_space = spaces.Dict(spaces=obs_spaces)
        self.action_space = spaces.Dict({'joints': spaces.Box(
            np.asarray([-np.pi, -np.pi, -np.pi, -np.pi], dtype=np.float32),
            np.asarray([np.pi, np.pi, np.pi, 0.0], dtype=np.float32))
            })
        self.in_collision = False
        rospy.Subscriber('/demo_env/objects/vx300s/actuators/joints/safe_actions/in_collision', Bool, self.in_collision_cb)

    def in_collision_cb(self, msg):
        self.in_collision = msg.data

    def step(self, action):
        # Set actions before stepping
        action['joints'] = np.append(np.insert(action['joints'], -1, 0.0), 0.0)

        self.robot.set_action(action)

        # Step the environment
        self._step()
        self.steps += 1

        # Get observations and states
        robot_obs = self.robot.get_obs()
        can_state = self.can.get_state()
        obs = {'vx300s': robot_obs, 'can': can_state['position'][:2]}
        reward, done = self._get_reward(obs)

        return obs, reward, done, {}

    def reset(self):
        self.steps = 0

        # Reset robot
        robot_states = dict()
        robot_states['joint_pos'] = np.array([1.99 * np.pi * (-1.0 + 2.0 * random.random()),
                                              0.25 * np.pi * (-1.0 + 2.0 * random.random()),
                                              0.25 * np.pi * (-1.0 + 2.0 * random.random()),
                                              0.0,
                                              -np.pi * (random.random()),
                                              0.0,
                                              ],
                                             dtype='float32')
        self.robot.reset(states=robot_states)

        # Reset can position
        can_pos = [0.5 + random.random() * 0.3, 0.2 - random.random() * 0.4, 0.0]
        can_states = dict(position=can_pos)
        self.can.reset(can_states)

        # Reset the environment
        self._reset()

        # Get new observations
        robot_obs = self.robot.get_obs()
        can_state = self.can.get_state()
        obs = {'vx300s': robot_obs, 'can': can_state['position'][:2]}
        return obs

    def _get_reward(self, obs):
        robot_obs = obs['vx300s']
        can_pos = obs['can']
        joint_positions = robot_obs['joint_pos']
        angles = {'waist': joint_positions[0],
                  'shoulder': joint_positions[1],
                  'elbow': joint_positions[2],
                  'forearm_roll': joint_positions[3],
                  'wrist_angle': joint_positions[4],
                  'wrist_rotate': joint_positions[5],
                  }
        # Use forward kinematics to calculate end-effector position and orientation
        ee_transformation = self.chain.forward_kinematics(angles)['vx300s/ee_gripper_link']
        ee_pos = np.asarray(ee_transformation.pos)
        object_distance = np.linalg.norm(can_pos[0:2])
        pre_grasp_distance = object_distance - 0.1
        object_angle = np.arctan2(can_pos[1], can_pos[0])
        grasp_pos = [np.cos(object_angle) * pre_grasp_distance, np.sin(object_angle) * pre_grasp_distance, 0.2]

        # Calculate distance between can and end effector
        distance = np.linalg.norm(grasp_pos - ee_pos)
        ee_rotation = R.from_quat(np.roll(ee_transformation.rot, -1))
        # Get Euler angles from orientation of end effector
        ee_rot_angles = ee_rotation.as_euler('zyx')
        for i, angle in enumerate(ee_rot_angles):
            ee_rot_angles[i] = (angle + np.pi) % (2 * np.pi) - np.pi
        ee_angle_error = [object_angle - ee_rot_angles[0], ee_rot_angles[1], ee_rot_angles[2]]
        # Get the directional vector of the end effector
        ee_vector = ee_rotation.as_matrix()[:, 0]
        # We want to grasp the can with the end effector pointed horizontally from the base to the can
        grasp_vector = np.append(can_pos[0:2], 0.0)
        # Get the projection of the end effector directional vector on the grasp directional vector
        alignment = np.dot(grasp_vector / np.linalg.norm(grasp_vector), ee_vector / np.linalg.norm(ee_vector))

        if distance < 0.01 and np.all(np.abs(ee_angle_error) < 0.05):
            done = True
            reward = self.STEPS_PER_ROLLOUT - self.steps
            print('Congratulations! Goal position and orientation are reached.')
        elif self.in_collision:
            done = True
            reward = 0.0
        else:
            reward = alignment / (1 + 100 * distance)
            # reward = - np.sum(np.square(ee_angle_error)) * distance
            done = self.steps >= self.STEPS_PER_ROLLOUT
        return reward, done


class ActionRepeat(object):
    """Repeat the agent action multiple steps."""

    def __init__(self, env, amount):
        self._env = env
        self._amount = amount

    def __getattr__(self, name):
        return getattr(self._env, name)

    def step(self, action):
        done = False
        total_reward = 0
        current_step = 0
        while current_step < self._amount and not done:
            observ, reward, done, info = self._env.step(action)
            total_reward += reward
            current_step += 1
        return observ, total_reward, done, info


class NormalizeActions(object):
    """
    if and(low, high) not np.inf: scales action from [-1, 1] back to the unnormalized action
    if or(low,high) np.inf: no normalization of the actions, and true action must be used"""

    def __init__(self, env):
        self._env = env
        low, high = env.action_space.low, env.action_space.high
        self._enabled = np.logical_and(np.isfinite(low), np.isfinite(high))
        self._low = np.where(self._enabled, low, -np.ones_like(low))
        self._high = np.where(self._enabled, high, np.ones_like(low))

    def __getattr__(self, name):
        return getattr(self._env, name)

    @property
    def action_space(self):
        space = self._env.action_space
        low = np.where(self._enabled, -np.ones_like(space.low), space.low)
        high = np.where(self._enabled, np.ones_like(space.high), space.high)
        return spaces.Box(low, high, dtype=space.dtype)

    def step(self, action):
        # de-normalize action
        # action = (action + 1) / 2 * (self._high - self._low) + self._low
        action = self.denormalize_action(action)

        # apply action
        obs, reward, done, info = self._env.step(action)

        # normalize applied action (in case action was above maximum)
        # info['action'] = (2*info['action'] - (self._high + self._low))/(self._high - self._low)
        # info['action'] = self.normalize_action(info['action'])
        return obs, reward, done, info

    def denormalize_action(self, action):
        return (action + 1) / 2 * (self._high - self._low) + self._low

    def normalize_action(self, action):
        return (2*action - (self._high + self._low))/(self._high - self._low)


if __name__ == '__main__':

    rospy.init_node('demo_env', anonymous=True, log_level=rospy.INFO)
    seed = 42

    # Engine specific parameters
    engine = GazeboEngine(dt=0.4, seed=seed, gui=True)
    # engine = WebotsEngine(physics_step=0.01, seed=42)
    random.seed(seed)

    env = DemoEnv(engine, name="demo_env")
    env = NormalizeActions(Flatten(env))
    # check_env(env)
    env.seed(seed)

    n = 0
    save_path = substitute_xml_args("$(find eager_demo)/models/demo_{}.zip".format(n))
    while os.path.exists(save_path):
        n += 1
        save_path = substitute_xml_args("$(find eager_demo)/models/demo_{}.zip".format(n))
    model = PPO('MlpPolicy', env, verbose=1, tensorboard_log=substitute_xml_args("$(find eager_demo)/logs/demo_{}".format(n)))
    # model = PPO.load(substitute_xml_args("$(find eager_demo)/models/demo_{}".format(n)), verbose=1,
    #                  tensorboard_log=substitute_xml_args("$(find eager_demo)/logs/demo_{}".format(n)))
    # model.set_env(env)
    for i in range(10):
        model.learn(total_timesteps=100000)
        model.save(save_path)
    env.close()
