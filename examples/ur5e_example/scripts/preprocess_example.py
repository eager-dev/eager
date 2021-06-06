#!/usr/bin/env python3

# ROS packages required
import rospy
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine
from eager_bridge_gazebo.gazebo_engine import GazeboEngine
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor
import gym, gym.spaces
import pybullet_data

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':
    rospy.init_node('ur5e_example',
                    anonymous=True, log_level=rospy.WARN)
    
    dt = 0.08
    engine = WebotsEngine(dt=dt, world='$(find ur5e_example)/worlds/ur5e.wbt')
    # engine = GazeboEngine(dt=dt)
    # engine = PyBulletEngine(world='%s/%s.urdf' % (pybullet_data.getDataPath(), 'plane'), no_gui='false')
    
    # Initialize environment
    env_name = 'ros_env'
    robot_name = 'ur5e1'
    robot_type = 'ur5e'
    
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
                                        robot_type = robot_type,
                                        )
    
    ur5e1 = Object.create(robot_name, 'eager_robot_ur5e', robot_type)
    ur5e1.actuators['joints'].add_preprocess(launch_path='$(find eager_process_safe_actions)/launch/safe_actions.launch',
                                             launch_args=process_args.__dict__,
                                             observations_from_objects=[ur5e1],
                                             )
    
    env = EagerEnv(engine=engine, objects=[ur5e1], name=env_name)
    env = Flatten(env)
    # todo: As of now, check_env fails because we do not wrap the angles of the joints to [-pi, pi].
    #  This causes the observations to sometimes not be in the observation_space. This was a problem before,
    #  but went unnoticed because we always initialized at zero, and the few actions check_env took, never
    #  steered it outside of the observation_space. New issue: how to implement the wrapping?
    # check_env(env)

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