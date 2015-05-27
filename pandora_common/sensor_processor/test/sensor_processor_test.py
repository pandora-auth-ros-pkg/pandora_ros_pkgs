#!/usr/bin/env python
# encoding: utf-8

import sys

import timeit
import unittest

PKG = "sensor_processor"
NAME = "SensorProcessorTest"

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import time
import threading

from pandora_testing_tools.testing_interface import test_base
from std_msgs.msg import Int32

class SensorProcessorTest(test_base.TestBase):

    def setUp(self):
        super(SensorProcessorTest, self).setUp()

    def test_functionality_of_sensor_processor(self):

        self.assertSimpleResult("/test/listen", Int32(10),
                                "/test/answer", Int32(4))

    def test_time_of_sensor_processor(self):

        self.benchmarking = True
        duration = self.simpleBenchmark("/test/listen", "/test/answer", Int32(10))
        print "Time of plain sensor_processor is: " + str(duration) + " ms."
        self.assertGreater(2, duration)


if __name__ == "__main__":
    subscriber_topics = [
        ("/test/answer", "std_msgs", "Int32")]
    publisher_topics = [
        ("/test/listen", "std_msgs", "Int32")]

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    SensorProcessorTest.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, SensorProcessorTest, sys.argv)
    SensorProcessorTest.disconnect()

