#!/usr/bin/env python
# Copyright <alert_handler_static_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_functional_static_test'

import sys

import unittest
import rostest

import rospy
import alert_delivery

from data_fusion_communications.srv import GetObjectsSrv
from data_fusion_communications.srv import GetObjectsSrvResponse

class TestAlertHandlerStatic(unittest.TestCase):

    def setUp(self):
        
        self.deliveryBoy = alert_delivery.AlertDeliveryBoy()
        rospy.init_node(NAME, anonymous=True)

    def test_works(self):

        self.assertEqual(1+1, 2)

    def test_simple_test(self):

        rospy.wait_for_service('/data_fusion/get_objects')
        self.deliveryBoy.deliverHazmatOrder(0.4, 0, 1)
        rospy.sleep(1.2)
        get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv)
        try:
            resp = get_objects()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        pose = resp.hazmats.pop().pose
        self.assertAlmostEqual(pose.position.x, 2.10527276)
        self.assertAlmostEqual(pose.position.y, 0.53269821)
        self.assertAlmostEqual(pose.position.z, 1)
        self.assertAlmostEqual(pose.orientation.x, 0)
        self.assertAlmostEqual(pose.orientation.y, 0)
        self.assertAlmostEqual(pose.orientation.z, 0.70710679)
        self.assertAlmostEqual(pose.orientation.w, 0.70710679)
        

if __name__ == '__main__':
  rospy.sleep(1)
  rostest.rosrun(PKG, NAME, TestAlertHandlerStatic, sys.argv)

