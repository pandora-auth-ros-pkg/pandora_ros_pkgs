#!/usr/bin/env python
PKG = "pandora_vision_landoltc"
NAME = "landoltc_benchmark_test"
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
        datasetCamera = rospy.get_param("dataset_camera")
        self.imageHFOV = rospy.get_param("kinect_optical_frame/hfov")
        self.imageVFOV = rospy.get_param("kinect_optical_frame/vfov")
        self.imageHFOV *= math.pi / 180
        self.imageVFOV *= math.pi / 180

        self.algorithm = rospy.get_param("algorithm")
        imagePath = rospy.get_param("dataset_path")
        self.benchmarkTest(PKG_PATH + imagePath,
                           publisherTopic, subscriberTopic)

if __name__ == "__main__":
    publisherTopic = rospy.get_param("kinect/topic_name")
    publisherMessagePackage = rospy.get_param("publisherMessagePackage")
    publisherMessageType = rospy.get_param("publisherMessageType")

    subscriberTopic = rospy.get_param("subscriberTopic")
    subscriberMessagePackage = rospy.get_param("subscriberMessagePackage")
    subscriberMessageType = rospy.get_param("subscriberMessageType")

    subscriberAlertTopic = rospy.get_param("subscriberAlertTopic")
    subscriberAlertMessagePackage = rospy.get_param("subscriberAlertMessagePackage")
    subscriberAlertMessageType = rospy.get_param("subscriberAlertMessageType")

    subscriber_topics = [
        (subscriberTopic, subscriberMessagePackage, subscriberMessageType),
        (subscriberAlertTopic, subscriberAlertMessagePackage, subscriberAlertMessageType)]
    publisher_topics = [
        (publisherTopic, publisherMessagePackage, publisherMessageType)]
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")
    BenchmarkTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, BenchmarkTester, sys.argv)
    BenchmarkTester.disconnect()
