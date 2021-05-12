# ROS packages required
import rospy
import roslaunch
from ros_env.ros_env import RosEnv
from ros_env.robots.px150 import Px150
from ros_env.wrappers.flatten import Flatten

from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

if __name__ == '__main__':

    rospy.init_node('px150_example', anonymous=True, log_level=rospy.WARN)

    # launch simulator & physics bridge
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/bas/ros_gym_ws/src/ros/px150_example/launch/px150_dynamic_example.launch"], is_core=False)
    launch.start()
    rospy.loginfo("started")

    env = Flatten(RosEnv(robots=[Px150("px150")]))

    check_env(env)

    rospy.loginfo("Training starts")

    model = PPO('MlpPolicy', env, verbose=1)

    model.learn(total_timesteps=100000)

    obs = env.reset()

    for i in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        # action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    env.close()
