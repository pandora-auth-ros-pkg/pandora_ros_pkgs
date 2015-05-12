#!/usr/bin/env python

PKG = "pandora_vision_qrcode"
NAME = "accuracy_test"

import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg

PKG_PATH = rospkg.RosPack().get_path(PKG)

from pandora_testing_tools.testing_interface import vision_test_base


class AccuracyTester(vision_test_base.VisionTestBase):
    def test_accuracy(self):
        self.accuracyTest(PKG_PATH + "/test/functional/images",
                          "/camera/image_raw", "/vision/qr_alert")

if __name__ == "__main__":
    subscriber_topics = [
        ("/vision/qr_alert", "pandora_vision_msgs", "QRAlertsVectorMsg")]
    publisher_topics = [
        ("/camera/image_raw", "sensor_msgs", "Image")]

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")

    AccuracyTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, AccuracyTester, sys.argv)
    AccuracyTester.disconnect()
