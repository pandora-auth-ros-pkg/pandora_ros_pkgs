#!/usr/bin/env python
PKG = "pandora_vision_qrcode"
NAME = "qrcode_benchmark_test"
import os
import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg
import math

from pandora_vision_support.pandora_vision_testing_interface import vision_benchmark_test_base

PKG_PATH = rospkg.RosPack().get_path(PKG)


class BenchmarkTester(vision_benchmark_test_base.VisionBenchmarkTestBase):
    def test_benchmark(self):
        self.datasetCamera = rospy.get_param("dataset_camera")

        self.imageHFOV = rospy.get_param("/kinect_optical_frame/hfov")
        self.imageVFOV = rospy.get_param("/kinect_optical_frame/vfov")

        self.imageHFOV *= math.pi / 180
        self.imageVFOV *= math.pi / 180

        self.algorithm = rospy.get_param("algorithm")
        if rospy.has_param("benchmarkFlag"):
            benchmarkFlag = rospy.get_param("benchmarkFlag")
        else:
            benchmarkFlag = True

        imagePath = rospy.get_param("dataset_path")
        self.benchmarkTest(PKG_PATH + imagePath,
                           publisherTopic, subscriberTopic, benchmarkFlag)

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)

    publisherTopic = None
    publisherMessagePackage = None
    publisherMessageType = None
    if rospy.has_param("kinect/topic_name"):
        publisherTopic = rospy.get_param("kinect/topic_name")
    else:
        rospy.logfatal("Could not find the Image Topic name parameter!")
    if rospy.has_param("publisherMessagePackage"):
        publisherMessagePackage = rospy.get_param("publisherMessagePackage")
    else:
        rospy.logfatal("Could not find the Message Package parameter!")
    if rospy.has_param("publisherMessageType"):
        publisherMessageType = rospy.get_param("publisherMessageType")
    else:
        rospy.logfatal("Could not find the Message Type parameter!")

    subscriberTopic = None
    subscriberMessagePackage = None
    subscriberMessageType = None

    if rospy.has_param("subscriberTopic"):
        subscriberTopic = rospy.get_param("subscriberTopic")
    else:
        rospy.logfatal("Could not find the Processor Image Topic name" +
                       " parameter!")

    if rospy.has_param("subscriberMessagePackage"):
        subscriberMessagePackage = rospy.get_param("subscriberMessagePackage")
    else:
        rospy.logfatal("Could not find the Processor Message Package" +
                       " parameter!")
    if rospy.has_param("subscriberMessageType"):
        subscriberMessageType = rospy.get_param("subscriberMessageType")
    else:
        rospy.logfatal("Could not find the Processor Message Type parameter!")

    subscriberAlertTopic = None
    subscriberAlertMessagePackage = None
    subscriberAlertMessageType = None

    if rospy.has_param("subscriberAlertTopic"):
        subscriberAlertTopic = rospy.get_param("subscriberAlertTopic")
    else:
        rospy.logfatal("Could not find the Alert Image Topic name" +
                       " parameter!")

    if rospy.has_param("subscriberAlertMessagePackage"):
        subscriberAlertMessagePackage = rospy.get_param("subscriberAlert" +
                                                        "MessagePackage")
    else:
        rospy.logfatal("Could not find the Alert Message Package" +
                       " parameter!")
    if rospy.has_param("subscriberAlertMessageType"):
        subscriberAlertMessageType = rospy.get_param("subscriberAlert" +
                                                     "MessageType")
    else:
        rospy.logfatal("Could not find the Alert Message Type parameter!")

    if (subscriberTopic and subscriberMessagePackage and
       subscriberMessageType and subscriberAlertTopic and publisherTopic and
       publisherMessageType and publisherMessagePackage):

        subscriber_topics = [(subscriberTopic, subscriberMessagePackage,
                              subscriberMessageType),
                             (subscriberAlertTopic,
                              subscriberAlertMessagePackage,
                              subscriberAlertMessageType)]

        subscriberTopic = [subscriberTopic, subscriberAlertTopic]
        publisher_topics = [
            (publisherTopic, publisherMessagePackage, publisherMessageType)]
        rospy.loginfo("Test is Starting!")
        BenchmarkTester.connect(subscriber_topics, publisher_topics, 2, False)
        rostest.rosrun(PKG, NAME, BenchmarkTester, sys.argv)
        BenchmarkTester.disconnect()

    else:
        rospy.logerr("Could not execute the benchmark test for the package :" +
                     " %s !", PKG)
