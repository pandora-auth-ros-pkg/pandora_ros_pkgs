#!/usr/bin/env python
PKG = "pandora_vision_hole"
NAME = "hole_benchmark_test"
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
        datasetCamera = rospy.get_param("dataset_camera")
        camerasList = rospy.get_param("camera_sensors")
        for ii in xrange(len(camerasList)):
            if datasetCamera == camerasList[ii]["name"]:
                self.imageWidth = camerasList[ii]["image_width"]
                self.imageHeight = camerasList[ii]["image_height"]
                self.imageHFOV = camerasList[ii]["hfov"]
                self.imageVFOV = camerasList[ii]["vfov"]
                break
        self.imageHFOV *= math.pi / 180
        self.imageVFOV *= math.pi / 180

        self.algorithm = rospy.get_param("algorithm")
        imagePath = rospy.get_param("dataset_path")
        self.benchmarkTest(PKG_PATH + imagePath,
                           publisherTopic, subscriberTopic)
if __name__ == "__main__":
    publisherTopic = rospy.get_param("publisherTopic")
    publisherMessagePackage = rospy.get_param("publisherMessagePackage")
    publisherMessageType = rospy.get_param("publisherMessageType")
    
    subscriberTopic = rospy.get_param("subscriberTopic")
    subscriberMessagePackage = rospy.get_param("subscriberMessagePackage")
    subscriberMessageType = rospy.get_param("subscriberMessageType")
    
    subscriber_topics = [
        (subscriberTopic, subscriberMessagePackage, subscriberMessageType)]
    publisher_topics = [
        (publisherTopic, publisherMessagePackage, publisherMessageType)]
    rospy.sleep(15)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")
    BenchmarkTester.connect(subscriber_topics, publisher_topics, 2, False)
    rospy.sleep(2)
    rostest.rosrun(PKG, NAME, BenchmarkTester, sys.argv)
    BenchmarkTester.disconnect()
