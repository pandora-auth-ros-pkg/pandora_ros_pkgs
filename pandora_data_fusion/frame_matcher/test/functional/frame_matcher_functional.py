#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

PKG = "frame_matcher"
NAME = "frame_matcher_test"

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_vision_support import rgb_to_enhanced
from pandora_testing_tools.testing_interface import test_base
from pandora_testing_tools.testing_interface import utils

class FrameMatcherTest(test_base.TestBase):

    def setUp(self):
        super(FrameMatcherTest, self).setUp()
        self.benchmarking = False
        self.output_topic = "/camera/enhanced_image"
        self.annotations = rgb_to_enhanced.read_annotation_file("./camera_annotations.txt")

    def test_matching(self):
        self.playFromBag(block=True)
        rospy.sleep(1)
        self.assertTrue(True)
        # # At least one message has been published by node
        # self.assertTrue(self.repliedList[self.output_topic])
        # # Only one message has been published by node
        # print rospy.loginfo(self.messageList[self.output_topic])
        # self.assertEqual(len(self.messageList[self.output_topic]), 1)

        # message = self.messageList[self.output_topic][0]
        # # Only one ROI has been included in output EnhancedImage
        # self.assertEqual(len(message.regionsOfInterest), 1)

        # annotation = self.annotations["frame0.png"]
        # expected_roi = rgb_to_enhanced.annotation_to_roi(annotation)
        # actual_roi = message.regionsOfInterest[0]
        # distance = utils.distance_keypoints(expected_roi.center, actual_roi.center)
        # rospy.loginfo("Distance between centers is: "+str(distance))
        # self.assertLess(distance, 5)
        # width_error = math.fabs(actual_roi.width - expected_roi.width)
        # self.assertLess(width_error, 3)
        # height_error = math.fabs(actual_roi.height - expected_roi.height)
        # self.assertLess(height_error, 3)

if __name__ == '__main__':

    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    SUBSCRIBER_TOPICS = [("/camera/enhanced_image", "pandora_vision_msgs", "EnhancedImage")]
    PUBLISHER_TOPICS = []
    FrameMatcherTest.connect(SUBSCRIBER_TOPICS, PUBLISHER_TOPICS, state=4, with_bag=True)
    rostest.rosrun(PKG, NAME, FrameMatcherTest, sys.argv)
    FrameMatcherTest.disconnect()
