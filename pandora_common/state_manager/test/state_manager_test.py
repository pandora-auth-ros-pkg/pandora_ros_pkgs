#!/usr/bin/env python

PKG = 'state_manager'
NODE_NAME = 'state_manager_python_client'

import unittest

import rospy
import rostest
import roslib
roslib.load_manifest(PKG)

from state_manager.state_client import StateClient
from state_manager_msgs.srv import GetStateInfoRequest


class TestStateManager(unittest.TestCase):

    def setUp(self):
        self.client = StateClient(silent=False)
        self.client.client_initialize()

    def test_current_state(self):
        self.client.change_state_and_wait(0)

        response = self.client.get_current_state()

        self.assertEqual(0, response)

    def test_previous_state(self):
        self.client.change_state_and_wait(2)
        self.client.change_state_and_wait(3)

        response = self.client.get_previous_state()

        self.assertEqual(2, response)

    def test_state_info_wrong_request(self):
        req = GetStateInfoRequest()
        req.option = 99
        try:
            res = self.client._state_info(req)
        except rospy.ServiceException:
            rospy.logerr('Service GetStateInfo failed to respond.')

        self.assertEqual(res.state, -1)


if __name__ == '__main__':
    rospy.init_node(NODE_NAME, anonymous=True)
    rostest.rosrun(PKG, NODE_NAME, TestStateManager)
