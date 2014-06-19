#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
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

PKG = 'pandora_vision_hole_detector'
NAME = 'hole_detector_test'

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

import cv2

from pandora_testing_tools.testing_interface import test_base
from vision_communications.msg import HolesDirectionsVectorMsg
from vision_communications.msg import CandidateHolesVectorMsg
from vision_communications.msg import EnhancedHolesVectorMsg

class HoleDetectorTest(test_base.TestBase):

    # tests the function of the hole detector package
    def test_hole_detector(self):

        self.assertTrue(self.replied)

        width = 640
        height = 480

        holesDirectionsMsgs = self.messageList[0]

        # how many alert messages have been published
        self.assertEqual(len(holesDirectionsMsgs), 1)

        # how many alerts have been sent with the first message
        self.assertEqual(len(holesDirectionsMsgs[0].holesDirections), 2)
        hole_0 = holesDirectionsMsgs[0].holesDirections[0]
        hole_1 = holesDirectionsMsgs[0].holesDirections[1]

        # the coordinates of the first valid hole's keypoint
        x = 352.9
        y = 238.2

        # the coordinates of the first valid hole's keypoint relative to the
        # center of the image
        expectedX = x - width / 2
        expectedY = height / 2 - y

        # the hole's expected yaw
        expectedYaw = math.atan((2 * float(expectedX) / width) * math.tan(58 * math.pi / 360))

        # the hole's expected pitch
        expectedPitch = math.atan((2 * float(expectedY) / height) * math.tan(45 * math.pi / 360))

        self.assertAlmostEqual(hole_0.yaw, expectedYaw, 3)
        self.assertAlmostEqual(hole_0.pitch, expectedPitch, 3)
        self.assertGreater(hole_0.probability, 0.9)

        # the coordinates of the second valid hole's keypoint
        x = 328
        y = 106


        # the coordinates of the first valid hole's keypoint relative to the
        # center of the image
        expectedX = x - width / 2
        expectedY = height / 2 - y

        # the hole's expected yaw
        expectedYaw = math.atan((2 * float(expectedX) / width) * math.tan(58 * math.pi / 360))

        # the hole's expected pitch
        expectedPitch = math.atan((2 * float(expectedY) / height) * math.tan(45 * math.pi / 360))

        self.assertAlmostEqual(hole_1.yaw, expectedYaw, 3)
        self.assertAlmostEqual(hole_1.pitch, expectedPitch, 3)
        self.assertGreater(hole_1.probability, 0.9)


    # tests the function of the depth node
    def test_depth_node(self):

        self.assertTrue(self.replied)

        candidateHolesMsgs = self.messageList[1]

        # how many alert messages have been published
        self.assertEqual(len(candidateHolesMsgs), 2)

        # how many alerts have been sent with the first message
        self.assertEqual(len(candidateHolesMsgs[0].candidateHoles), 2)

        # make assertions about the first candidate hole
        hole_0 = candidateHolesMsgs[0].candidateHoles[0]

        # the coordinates of the first valid hole's data
        # are halved because of the wavelet analysis used

        # The first hole's keypoint
        x = hole_0.keypointX * 2
        self.assertLess(346, x);
        self.assertGreater(348, x);

        y = hole_0.keypointY * 2
        self.assertLess(237, y);
        self.assertGreater(239, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_0.verticesX), len(hole_0.verticesY))

        # The first hole's bounding box is itself bounded
        # in a bounding box
        for i in range(0, len(hole_0.verticesX)):
            self.assertGreater(2 * hole_0.verticesX[i], 300)
            self.assertLess(2 * hole_0.verticesX[i], 500)

        for i in range(0, len(hole_0.verticesY)):
            self.assertGreater(2 * hole_0.verticesY[i], 300)
            self.assertLess(2 * hole_0.verticesY[i], 420)

        # There should be equal number of elements in both outline vectors
        self.assertEqual(len(hole_0.outlineX), len(hole_0.outlineY))

        # The minimum x coordinate of the bounding box's points
        minX = min(hole_0.verticesX)

        # The minimum x coordinate of the bounding box's points
        minY = min(hole_0.verticesY)

        # The maximum x coordinate of the bounding box's points
        maxX = max(hole_0.verticesX)

        # The maximum x coordinate of the bounding box's points
        maxY = max(hole_0.verticesY)

        # The first hole's outline is bounded by its bounding box
        for i in range(0, len(hole_0.outlineX)):
            self.assertGreaterEqual(hole_0.outlineX[i], minX)
            self.assertLessEqual(hole_0.outlineX[i], maxX)


        # make assertions about the second candidate hole
        hole_1 = candidateHolesMsgs[0].candidateHoles[1]

        # the coordinates of the second valid hole's data
        # are halved because of the wavelet analysis used

        # The second hole's keypoint
        x = hole_1.keypointX * 2
        self.assertLess(330, x);
        self.assertGreater(328, x);

        y = hole_1.keypointY * 2
        self.assertLess(108, y);
        self.assertGreater(106, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_1.verticesX), len(hole_1.verticesY))

        # The second hole's bounding box is itself bounded in a bounding box
        for i in range(0, len(hole_1.verticesX)):
            self.assertGreater(2 * hole_1.verticesX[i], 360)
            self.assertLess(2 * hole_1.verticesX[i], 460)

        for i in range(0, len(hole_1.verticesY)):
            self.assertGreater(2 * hole_1.verticesY[i], 120)
            self.assertLess(2 * hole_1.verticesY[i], 220)

        # There should be equal number of elements in both outline vectors
        self.assertEqual(len(hole_1.outlineX), len(hole_1.outlineY))

        # The minimum x coordinate of the bounding box's points
        minX = min(hole_1.verticesX)

        # The minimum x coordinate of the bounding box's points
        minY = min(hole_1.verticesY)

        # The maximum x coordinate of the bounding box's points
        maxX = max(hole_1.verticesX)

        # The maximum x coordinate of the bounding box's points
        maxY = max(hole_1.verticesY)

        # The first hole's outline is bounded by its bounding box
        for i in range(0, len(hole_1.outlineX)):
            self.assertGreaterEqual(hole_1.outlineX[i], minX)
            self.assertLessEqual(hole_1.outlineX[i], maxX)


    # tests the function of the rgb node
    def test_rgb_node(self):

        self.assertTrue(self.replied)

        candidateHolesMsgs = self.messageList[2]

        # how many alert messages have been published
        self.assertEqual(len(candidateHolesMsgs), 2)

        # how many alerts have been sent with the first message
        self.assertEqual(len(candidateHolesMsgs[1].candidateHoles), 3)

        # make assertions about the first candidate hole
        hole_0 = candidateHolesMsgs[1].candidateHoles[0]

        # the coordinates of the first valid hole's data
        # are halved because of the wavelet analysis used

        # The first hole's keypoint
        x = hole_0.keypointX * 2
        self.assertLess(380, x);
        self.assertGreater(384, x);

        y = hole_0.keypointY * 2
        self.assertLess(359, y);
        self.assertGreater(361, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_0.verticesX), len(hole_0.verticesY))

        # The first hole's bounding box is itself bounded
        # in a bounding box
        for i in range(0, len(hole_0.verticesX)):
            self.assertGreater(2 * hole_0.verticesX[i], 300)
            self.assertLess(2 * hole_0.verticesX[i], 480)

        for i in range(0, len(hole_0.verticesY)):
            self.assertGreater(2 * hole_0.verticesY[i], 300)
            self.assertLess(2 * hole_0.verticesY[i], 420)

        # There should be equal number of elements in both outline vectors
        self.assertEqual(len(hole_0.outlineX), len(hole_0.outlineY))

        # The minimum x coordinate of the bounding box's points
        minX = min(hole_0.verticesX)

        # The minimum x coordinate of the bounding box's points
        minY = min(hole_0.verticesY)

        # The maximum x coordinate of the bounding box's points
        maxX = max(hole_0.verticesX)

        # The maximum x coordinate of the bounding box's points
        maxY = max(hole_0.verticesY)

        # The first hole's outline is bounded by its bounding box
        for i in range(0, len(hole_0.outlineX)):
            self.assertGreaterEqual(hole_0.outlineX[i], minX)
            self.assertLessEqual(hole_0.outlineX[i], maxX)


        # make assertions about the second candidate hole
        hole_1 = candidateHolesMsgs[1].candidateHoles[1]

        # the coordinates of the second valid hole's data
        # are halved because of the wavelet analysis used

        # The second hole's keypoint
        x = hole_1.keypointX * 2
        self.assertLess(550, x);
        self.assertGreater(560, x);

        y = hole_1.keypointY * 2
        self.assertLess(270, y);
        self.assertGreater(274, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_1.verticesX), len(hole_1.verticesY))

        # The second hole's bounding box is itself bounded in a bounding box
        for i in range(0, len(hole_1.verticesX)):
            self.assertLess(530, 2 * hole_1.verticesX[i])
            self.assertGreater(600, 2 * hole_1.verticesX[i])

        for i in range(0, len(hole_1.verticesY)):
            self.assertLess(2 * hole_1.verticesY[i], 320)
            self.assertGreater(2 * hole_1.verticesY[i], 130)

        # There should be equal number of elements in both outline vectors
        self.assertEqual(len(hole_1.outlineX), len(hole_1.outlineY))

        # The minimum x coordinate of the bounding box's points
        minX = min(hole_1.verticesX)

        # The minimum x coordinate of the bounding box's points
        minY = min(hole_1.verticesY)

        # The maximum x coordinate of the bounding box's points
        maxX = max(hole_1.verticesX)

        # The maximum x coordinate of the bounding box's points
        maxY = max(hole_1.verticesY)

        # The first hole's outline is bounded by its bounding box
        for i in range(0, len(hole_1.outlineX)):
            self.assertGreaterEqual(hole_1.outlineX[i], minX)
            self.assertLessEqual(hole_1.outlineX[i], maxX)


        # make assertions about the third candidate hole
        hole_2 = candidateHolesMsgs[1].candidateHoles[2]

        # the coordinates of the second valid hole's data
        # are halved because of the wavelet analysis used

        # The third hole's keypoint
        x = hole_2.keypointX * 2
        self.assertLess(400, x);
        self.assertGreater(410, x);

        y = hole_2.keypointY * 2
        self.assertLess(160, y);
        self.assertGreater(164, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_2.verticesX), len(hole_2.verticesY))

        # The second hole's bounding box is itself bounded in a bounding box
        for i in range(0, len(hole_2.verticesX)):
            self.assertGreater(2 * hole_2.verticesX[i], 340)
            self.assertLess(2 * hole_2.verticesX[i], 480)

        for i in range(0, len(hole_2.verticesY)):
            self.assertGreater(2 * hole_2.verticesY[i], 100)
            self.assertLess(2 * hole_2.verticesY[i], 230)

        # There should be equal number of elements in both outline vectors
        self.assertEqual(len(hole_2.outlineX), len(hole_2.outlineY))

        # The minimum x coordinate of the bounding box's points
        minX = min(hole_2.verticesX)

        # The minimum x coordinate of the bounding box's points
        minY = min(hole_2.verticesY)

        # The maximum x coordinate of the bounding box's points
        maxX = max(hole_2.verticesX)

        # The maximum x coordinate of the bounding box's points
        maxY = max(hole_2.verticesY)

        # The first hole's outline is bounded by its bounding box
        for i in range(0, len(hole_2.outlineX)):
            self.assertGreaterEqual(hole_2.outlineX[i], minX)
            self.assertLessEqual(hole_2.outlineX[i], maxX)


    def test_enhanced_holes(self):

        self.assertTrue(self.replied)

        enhancedHolesMsgs = self.messageList[3]

        # how many alert messages have been published
        self.assertEqual(len(enhancedHolesMsgs), 1)

        # how many alerts have been sent with the first message
        self.assertEqual(len(enhancedHolesMsgs[0].enhancedHoles), 2)

        # depth analysis is possible
        self.assertTrue(enhancedHolesMsgs[0].isDepth)

        # the first hole
        hole_0 = enhancedHolesMsgs[0].enhancedHoles[0]

        # The first hole's keypoint
        x = hole_0.keypointX
        self.assertLess(398, x);
        self.assertGreater(400, x);

        y = hole_0.keypointY
        self.assertLess(360, y);
        self.assertGreater(361, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_0.verticesX), len(hole_0.verticesY))

        # The first hole's bounding box is itself bounded
        # in a bounding box
        for i in range(0, len(hole_0.verticesX)):
            self.assertGreater(hole_0.verticesX[i], 300)
            self.assertLess(hole_0.verticesX[i], 500)

        for i in range(0, len(hole_0.verticesY)):
            self.assertGreater(hole_0.verticesY[i], 300)
            self.assertLess(hole_0.verticesY[i], 420)


        # the second hole
        hole_1 = enhancedHolesMsgs[0].enhancedHoles[1]

        # The second hole's keypoint
        x = hole_1.keypointX
        self.assertLess(407, x);
        self.assertGreater(408, x);

        y = hole_1.keypointY
        self.assertLess(162, y);
        self.assertGreater(164, y);

        # There should be equal number of elements in both vertices vectors
        self.assertEqual(len(hole_1.verticesX), len(hole_1.verticesY))

        # The second hole's bounding box is itself bounded in a bounding box
        for i in range(0, len(hole_1.verticesX)):
            self.assertGreater(hole_1.verticesX[i], 340)
            self.assertLess(hole_1.verticesX[i], 480)

        for i in range(0, len(hole_1.verticesY)):
            self.assertGreater(hole_1.verticesY[i], 100)
            self.assertLess(hole_1.verticesY[i], 230)

if __name__ == '__main__':

    subscriber_topics = [
        ("/vision/holes_direction", "vision_communications", "HolesDirectionsVectorMsg"),
        ("/pandora_vision/hole_detector/synchronized/depth/candidate_holes", "vision_communications", "CandidateHolesVectorMsg"),
        ("/pandora_vision/hole_detector/synchronized/rgb/candidate_holes", "vision_communications", "CandidateHolesVectorMsg"),
        ("/vision/enhanced_holes", "vision_communications", "EnhancedHolesVectorMsg")]

    rospy.sleep(15)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    HoleDetectorTest.connect(subscriber_topics, [])
    HoleDetectorTest.playFromBag(block = True)
    rospy.sleep(2)
    rostest.rosrun(PKG, NAME, HoleDetectorTest, sys.argv)
    HoleDetectorTest.disconnect()
