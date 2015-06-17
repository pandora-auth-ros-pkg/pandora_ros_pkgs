#!/usr/bin/env python

""" Tests for the client factory functionality """

import unittest

import roslib
roslib.load_manifest('pandora_end_effector_controller')
import rospy


from pandora_end_effector_controller.effector_clients import SensorClient, LinearClient, HeadClient
from pandora_end_effector_controller.client_factory import ClientFactory

class TestClientFactory(unittest.TestCase):

    def setUp(self):
        self.factory = ClientFactory()

    def test_make_linear_client(self):
      client = self.factory.make_client('linear_client')
      self.assertEqual(type(client) ,type(LinearClient()))

    def test_make_sensor_client(self):
      client = self.factory.make_client('sensor_client')
      self.assertEqual(type(client), type(SensorClient()))

    def test_make_head_client(self):
      client = self.factory.make_client('head_client')
      self.assertEqual(type(client), type(HeadClient()))

if __name__ == '__main__':
    unittest.main()
