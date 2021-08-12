#!/usr/bin/env python3

import rospy
import numpy as np
import kinpy as kp
from gym import spaces
from collections import OrderedDict
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_core.utils.file_utils import substitute_xml_args
from eager_bridge_gazebo.gazebo_engine import GazeboEngine  # noqa: F401
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor
from random import random
from eager_core.utils.env_checker import check_env

from stable_baselines3 import PPO


class DemoEnv(EagerEnv):

    def __init__(self, engine, objects, name="demo_env"):
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
                                            checks_per_rad=15,
                                            vel_limit=2.0,
                                            robot_type='vx300s',
                                            collision_height=0.1,
                                            base_length=0.4,
                                            workspace_length=2.4,
                                            )

        self.robot.actuators['joints'].add_preprocess(
            launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
            launch_args=process_args.__dict__,
            observations_from_objects=[self.robot],
            action_space=spaces.Box(low=-np.pi, high=np.pi, shape=(6,)),
            )

        # Create a can
        self.can = Object.create('can',
                                 'eager_solid_other',
                                 'can',
                                 position=[0.2 + random() * 0.4, 0.2 - random() * 0.4, 0.0],
                                 orientation=[0.0, 0.0, 0.0, 1.0],
                                 )
        objects = [self.robot, self.can]
        super().__init__(engine=engine, objects=objects, name=name)

    def step(self, action):
        """
        Steps the environment

        :param action: A dictionary of actions to perform
        :return: observations, reward, done, states
        """
        self.steps += 1

        for object in self.objects:
            if object.action_space:
                object.set_action(action[object.name])

        self._step()

        state = self._get_states()
        obs = self._get_obs()
        reward = self._get_reward(obs)
        done = self._is_done(obs)
        return obs, reward, done, {'state': state}

    def reset(self):
        self.steps = 0

        # Reset robot
        robot_states = dict()
        robot_states['joint_pos'] = np.array([0, 0, 0, 0, 0, 0], dtype='float32')
        robot_states['joint_vel'] = np.array([0, 0, 0, 0, 0, 0], dtype='float32')
        self.robot.reset(states=robot_states)

        # Reset can position
        can_pos = [0.2 + random() * 0.4, 0.2 - random() * 0.4, 0.0]
        can_states = dict(position=can_pos, orientation=[0.0, 0.0, 0.0, 1.0])
        self.can.reset(can_states)

        # Reset the environment
        self._reset()

        # Get new observations
        return self._get_obs()

    def _get_reward(self, obs):
        can_pos = self.can.get_state(['base_pos'])
        joint_positions = obs['vx300s']['joint_sensors']
        angles = {'waist': joint_positions[0],
                  'shoulder': joint_positions[1],
                  'elbow': joint_positions[2],
                  'forearm_roll': joint_positions[3],
                  'wrist_angle': joint_positions[4],
                  'wrist_rotate': joint_positions[5],
                  }
        ee_transformation = self.chain.forward_kinematics(angles)['vx300s/ee_gripper_link']
        ee_pos = np.asarray(ee_transformation.pos)
        pos_error = np.linalg.norm(ee_pos - can_pos)
        return -pos_error

    def _is_done(self, obs):
        return self.steps >= self.STEPS_PER_ROLLOUT

    def _get_obs(self):
        """
        Gets the observations of all objects and observers in this environment

        :return: observations
        """
        obs = OrderedDict()

        for object in self.objects:
            if object.observation_space:
                obs[object.name] = object.get_obs()

        for observer in self.observers:
            obs[observer.name] = observer.get_obs()

        return obs

    def _get_states(self) -> 'OrderedDict[str, object]':
        """
        Gets the states of all objects and observers in this environment

        :return: states
        """
        state = OrderedDict()

        for object in self.objects:
            if object.state_space:
                state[object.name] = object.get_state()

        return state


if __name__ == '__main__':

    rospy.init_node('demo_env', anonymous=True, log_level=rospy.INFO)

    # Engine specific parameters
    engine = GazeboEngine(seed=42, gui=True)

    env = DemoEnv(engine, name="demo_env")
    env = Flatten(env)
    check_env(env)

    env.seed(42)

    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)

        env.render()
        if done:
            obs = env.reset()

    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=100000)

    env.close()
