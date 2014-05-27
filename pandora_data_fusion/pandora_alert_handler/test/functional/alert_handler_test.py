#!/usr/bin/env python
# Copyright <alert_handler_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_test'

import sys

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

import test_base
from test_base import distance
from test_base import direction

class AlertHandlerTest(test_base.TestBase):

    def test_saloon(self):

        rospy.sleep(20)
        self.assertEqual(len(self.currentVictimList), 2)
        self.assertGreater(distance(self.currentVictimList[0].victimPose.pose.position,
          self.currentVictimList[1].victimPose.pose.position), 0.2)


if __name__ == '__main__':

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    AlertHandlerTest.connect()
    rostest.rosrun(PKG, NAME, AlertHandlerTest, sys.argv)
    AlertHandlerTest.disconnect()

