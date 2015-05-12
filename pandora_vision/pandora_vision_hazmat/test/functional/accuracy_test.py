#!/usr/bin/env python

PKG = "pandora_vision_hazmat"
NAME = "accuracy_test"

import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg

PKG_PATH = rospkg.RosPack().get_path(PKG)

from pandora_testing_tools.testing_interface import vision_test_base
from sensor_msgs.msg import Image

class AccuracyTester(test_base.TestBase):
    def readImages(self, imagePath):
        self.images = []
        imageTypes = [".png", ".jpg", ".bmp"]
        bridge = CvBridge()
        if not os.path.isdir(imagePath):
            print("ERROR : Incorrect Path")

        for fileName in os.listdir(imagePath):
            isImage = False
            for type in imageTypes:
                if (type in fileName):
                    isImage = True
                    break
            if (not isImage):
                print fileName, "is not an image"
                continue
            # Read the next image.
            currentImg = cv2.imread(os.path.join(imagePath,
                                    fileName), -1)
>>>>>>> Split source into correct folders,add training code [ci skip]


class AccuracyTester(vision_test_base.VisionTestBase):
    def test_accuracy(self):
        self.accuracyTest(PKG_PATH + "/test/functional/images",
                          "/camera/image_raw", "/alert/hazmat")

if __name__ == "__main__":
    
    subscriber_topics = [
        ("/alert/hazmat", "std_msgs", "Bool")]
    publisher_topics = [
        ("/camera/image_raw", "sensor_msgs", "Image")]

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    rospy.loginfo("Test is Starting!")

    AccuracyTester.connect(subscriber_topics, publisher_topics, 2, False)
    rostest.rosrun(PKG, NAME, AccuracyTester, sys.argv)
    AccuracyTester.disconnect()
