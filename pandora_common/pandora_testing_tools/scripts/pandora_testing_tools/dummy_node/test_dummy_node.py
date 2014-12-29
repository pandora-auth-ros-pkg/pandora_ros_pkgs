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

    def test_trivial(self):

        self.mockPublish("/test/listen", Int32(10))

        self.assertTrue(self.repliedList["/test/listen"])
        self.assertEqual(self.messageList["/test/answer"], 4)

if __name__ == "__main__":
    subscriber_topics = [
        ("/test/answer", "std_msgs", "Int32")]
    publisher_topics = [
        ("/test/listen", "std_msgs", "Int32")]

    rospy.init_node(NAME, anonymous=True)
    DummyNodeTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, DummyNodeTester, sys.argv)
    DummyNodeTester.disconnect()
