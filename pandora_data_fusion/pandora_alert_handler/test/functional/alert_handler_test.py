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

    def setUp(self):

        pass

    def test_saloon(self):

        out = []
        self.fillInfo(out)
        
        self.assertEqual(len(out[0].holes), 2)
        self.assertEqual(len(out[0].victimsToGo), 2)
        self.assertEqual(distance(out[0].holes[0].pose.position,
          out[0].victimsToGo[0].pose.position), 0)
        self.assertEqual(distance(out[0].holes[1].pose.position,
          out[0].victimsToGo[1].pose.position), 0)
        self.assertGreater(distance(out[0].holes[0].pose.position,
          out[0].holes[1].pose.position), 0.2)

if __name__ == '__main__':

    rospy.sleep(20)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    AlertHandlerTest.connect()
    rostest.rosrun(PKG, NAME, AlertHandlerTest, sys.argv)
    AlertHandlerTest.disconnect()

