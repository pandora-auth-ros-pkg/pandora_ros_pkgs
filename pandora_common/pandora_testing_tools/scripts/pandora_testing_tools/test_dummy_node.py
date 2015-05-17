#!/usr/bin/env python
import sys

import unittest

PKG = "pandora_testing_tools"
NAME = "DummyNodeTester"

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import test_base
from std_msgs.msg import Int32

class DummyNodeTester(test_base.TestBase):

    def setUp(self):
        super(DummyNodeTester, self).setUp()

    def test_trivial_test_infrastructure(self):

        rospy.logdebug("Now test publishing...")
        self.mockPublish("/test/listen", "/test/answer", Int32(0))

        rospy.logdebug("Now test asserting...")
        self.assertTrue(self.repliedList["/test/answer"])
        self.assertEqual(len(self.messageList["/test/answer"]), 1)
        output_data = self.messageList["/test/answer"][0].data
        self.assertEqual(output_data, 4)

    def test_simple_functional_node(self):

        self.assertSimpleResult("/test/listen", Int32(10),
                                "/test/answer", Int32(4))

if __name__ == "__main__":
    subscriber_topics = [
        ("/test/answer", "std_msgs", "Int32")]
    publisher_topics = [
        ("/test/listen", "std_msgs", "Int32")]

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    DummyNodeTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, DummyNodeTester, sys.argv)
    DummyNodeTester.disconnect()
