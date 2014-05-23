#!/usr/bin/env python
# Copyright <vision_data_fusion_test.py>

PKG = 'pandora_alert_handler'
NAME = 'vision_data_fusion_test'

import sys

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import actionlib

from node_tests_msgs.msg import *

class VisionDataFusionTest(unittest.TestCase):

    @classmethod
    def connect(cls):

        cls.bagClient = actionlib.SimpleActionClient("/test/replay_bags", ReplayBagsAction)
        cls.bagClient.wait_for_server()
        goal = ReplayBagsGoal(start = True)
        cls.bagClient.send_goal(goal)
        cls.bagClient.wait_for_result()

    def test_cool(self):

        pass

if __name__ == '__main__':

    rospy.sleep(20)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    VisionDataFusionTest.connect()
    rostest.rosrun(PKG, NAME, VisionDataFusionTest, sys.argv)

