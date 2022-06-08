#!/usr/bin/env python3

import rospy

# Import eager packages
from eager_core.utils.file_utils import launch_roscore, load_yaml
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_pybullet.pybullet_engine import PyBulletEngine  # noqa: F401

# Required for action processor
from eager_process_safe_actions.safe_actions_processor import SafeActionsProcessor


if __name__ == '__main__':
    roscore = launch_roscore()  # First launch roscore

    rospy.init_node('eager_demo', anonymous=True, log_level=rospy.WARN)
    rate = rospy.Rate(1/0.08)

    # Define the engine
    engine = PyBulletEngine(gui=True)

    # Create robot
    robot = Object.create('robot', 'eager_robot_vx300s', 'vx300s')
    # Add action preprocessing
    processor = SafeActionsProcessor(robot_type='vx300s',
                                     vel_limit=0.25,
                                     collision_height=0.15,
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
    env = EagerEnv(name='demo1',
                   engine=engine,
                   objects=[robot, cam],
                   render_sensor=cam.sensors['camera_rgb'],
                   )
    env = Flatten(env)

    env.render()
    obs = env.reset()  # TODO: if code does not close properly, render seems to keep a thread open....
    for i in range(100):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        if done:
            obs = env.reset()
        rate.sleep()

    # todo: create a env.close(): close render screen, and env.shutdown() to shutdown the environment cleanly.
    env.close()
