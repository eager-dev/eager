#!/usr/bin/env python3

import sys
import rospy
import unittest
from eager_core.eager_env import EagerEnv
from eager_core.objects import Object
from eager_core.wrappers.flatten import Flatten
from eager_bridge_webots.webots_engine import WebotsEngine

from eager_core.utils.env_checker import check_env

PKG = 'eager_bridge_webots'
NAME = 'integration_test'

class IntegrationTest(unittest.TestCase):

    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node('integration_test_webots', anonymous=True)

    def test_check_env(self):
        engine = WebotsEngine(world='$(find ur5e_example)/worlds/ur5e.wbt')

        # Initialize environment
        robot = Object.create('ur5e1', 'eager_robot_ur5e', 'ur5e')
        env = EagerEnv(engine=engine, objects=[robot], name='ros_env')
        env = Flatten(env)
        try:
            check_env(env)
        except Exception as e:
            self.fail(
                "Stable Baselines 3 check env failed"
                " Msg: {}".format(e)
            )

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)