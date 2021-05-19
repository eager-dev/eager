# ROS packages required
import rospy
from ros_gym_core.ros_env import RosEnv
from ros_gym_core.objects import Robot
from ros_gym_core.wrappers.flatten import Flatten

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('ur5e_example', anonymous=True, log_level=rospy.WARN)

    # Engine specific parameters
    # todo: initialize a engine-specific dictionary for each engine with pre-defined keys and default values.
    #  i.e. WebotsParams(dict), PybulletParams(dict), with standard key-value pairs initialized to default values.
    #  Environment specific parameters must be supplied as arguments when the parameters are initialized.
    wb_params = dict()
    wb_params['bridge_type'] = 'webots'
    wb_params['launch_file'] = '/home/akeijzer/ros_gym_ws/src/ros-gym/ros_gym_bridge_webots/launch/%s.launch' % wb_params['bridge_type']
    wb_params['mode'] = 'fast'
    wb_params['no_gui'] = 'false'
    wb_params['world'] = '$(find ur5e_example)/worlds/ur5e'

    # Initialize environment
    env = RosEnv(robots=[Robot.create('ur5e1', 'ros_gym_robot_ur5e', 'ur5e')], name='ros_env', engine_params=wb_params)
    env = Flatten(env)
    check_env(env)

    # Initialize policy
    model = PPO('MlpPolicy', env, verbose=1)

    # Perform random actions
    obs = env.reset()
    for i in range(1000):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    # Start training
    rospy.loginfo("Training starts")
    obs = env.reset()
    model.learn(total_timesteps=100000)

    env.close()
