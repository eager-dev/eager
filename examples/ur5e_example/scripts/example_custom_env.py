#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.ros_env import BaseRosEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor

from stable_baselines3 import PPO

class MyEnv(BaseRosEnv):

    def __init__(self, engine):
        super().__init__(engine)

        self.STEPS_PER_ROLLOUT = 1000
        self.steps = 0

        self.ur5e = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')

        # Dit is wel een beetje lang
        process_args = SafeActionsProcessor(moveit_package = 'ur5_e_moveit_config',
                                    urdf_path = '$(find ur_e_description)/urdf/ur5e_robot.urdf.xacro',
                                    joint_names = ['shoulder_pan_joint',
                                                    'shoulder_lift_joint',
                                                    'elbow_joint',
                                                    'wrist_1_joint',
                                                    'wrist_2_joint',
                                                    'wrist_3_joint'],
                                    group_name = 'manipulator',
                                    duration = 0.1,
                                    object_frame = 'base_link',
                                    checks_per_rad = 15,
                                    vel_limit = 3.0,
                                    robot_type = 'ur5e',
                                    )
        self.ur5e.actuators['joints'].add_preprocess(launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
                                                    launch_args=process_args.__dict__,
                                                    observations_from_objects=[self.ur5e])


        #Wat gaan we doen met de camera? Er is nog geen preprocessing voor dit toch?
        #Iets van (nep) classificatie is wel nice
        self.camera = Object.create('ms21', 'eager_sensor_multisense_s21', 'dual_cam')

        self._init_nodes([self.ur5e, self.camera])

        self.observation_space = self.ur5e.observation_space
        self.action_space = self.ur5e.action_space
    
    def step(self, action):
        
        self.ur5e.set_action(action)
        self._step()

        self.steps += 1

        obs = self.ur5e.get_obs()

        return obs, self._get_reward(obs), self._is_done(obs), self.ur5e.get_state()
    
    def reset(self) -> object:

        self.ur5e.reset(self.ur5e.state_space.sample())
        self._reset()

        self.steps = 0

        return self.ur5e.get_obs()
    
    def _get_reward(self, obs):
        return -(self.ur5e.get_state(['joint_pos'])['joint_pos'][5] - 2)**2 # Je mag hier iets verzinnen Bas
    
    def _is_done(self, obs):
        return self.steps >= self.STEPS_PER_ROLLOUT


if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e.wbt')

    env = MyEnv(engine)
    env = Flatten(env)

    env.seed(42)

    rospy.loginfo("Training starts")

    model = PPO('MlpPolicy', env, verbose=1)
    
    model.learn(total_timesteps=100000)

    obs = env.reset()
    
    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()
    
    env.close()