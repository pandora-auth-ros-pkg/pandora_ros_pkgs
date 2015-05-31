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

PKG = 'pandora_vision_hole_exploration'
NAME = 'accuracy_test'

import os
import sys
import roslib; roslib.load_manifest(PKG)
import rostest
import rospy
import rospkg
import math

from pandora_testing_tools.testing_interface import test_base
from pandora_vision_msgs.msg import HolesDirectionsVectorMsg
from pandora_vision_msgs.msg import ExplorerCandidateHolesVectorMsg

PKG_PATH = rospkg.RosPack().get_path(PKG)

class AccuracyTester(test_base.TestBase):

  # tests the function of the hole exploration package
    def test_accuracy(self):

      self.assertTrue(self.replied)

      width = 640
      height = 480

      holesDirectionsMsgs = self.messageList[0]

      # how many alert messages have been published
      self.assertEqual(len(holesDirectionsMsgs), 1)

      # how many alerts have been sent with the first message
      self.assertEqual(len(holesDirectionsMsgs[0].holesDirections), 1)
      hole_0 = holesDirectionsMsgs[0].holesDirections[0]

      # the coordinates of the first valid hole's keypoint
      x = 352.86
      y = 238.17

      # the coordinates of the first valid hole's keypoint relative to the
      # center of the image
      expectedX = x - width / 2
      expectedY = height / 2 - y

      # the hole's expected yaw
      expectedYaw = math.atan((2 * float(expectedX) / width) * math.tan(58 * math.pi / 360))

      # the hole's expected pitch
      expectedPitch = math.atan((2 * float(expectedY) / height) * math.tan(45 * math.pi / 360))

      self.assertAlmostEqual(hole_0.yaw, expectedYaw, 2)
      self.assertAlmostEqual(hole_0.pitch, expectedPitch, 2)
      self.assertGreater(hole_0.probability, 0.5)



    # tests the function of the depth node
    def test_depth_node(self):

      self.assertTrue(self.replied)

      explorerCandidateHolesMsgs = self.messageList[1]

      # how many alert messages have been published
      self.assertEqual(len(explorerCandidateHolesMsgs), 2)

      # how many alerts have been sent with the first message
      self.assertEqual(len(explorerCandidateHolesMsgs[0].candidateHoles), 2)

      # make assertions about the first candidate hole
      hole_0 = explorerCandidateHolesMsgs[0].explorerCandidateHoles[0]

      # The first hole's keypoint
      x = hole_0.keypointX
      self.assertLess(356, x);
      self.assertGreater(358, x);

      y = hole_0.keypointY
      self.assertLess(239, y);
      self.assertGreater(241, y);

      # The first hole's bounding box is itself bounded
      # in a bounding box
      self.assertLess(260, hole_0.verticeX)
      self.assertGreater(450, hole_0.verticeY)

      self.assertLess(195, hole_0.verticeY)
      self.assertGreater(285, hole_0.verticeY)


      # The minimum x coordinate of the bounding box's points
      minX = hole_0.verticeX

      # The minimum y coordinate of the bounding box's points
      minY = hole_0.verticeY

      # The maximum x coordinate of the bounding box's points
      maxX = hole_0.verticeX + 2 * (hole_0.keypointX - hole_0.verticeX) 

      # The maximum y coordinate of the bounding box's points
      maxY = hole_0.verticeY + 2 * (hole_0.keypointY - hole_0.verticeY) 


      # make assertions about the second candidate hole
      hole_1 = explorerCandidateHolesMsgs[0].explorerCandidateHoles[1]

      # The second hole's keypoint
      x = hole_1.keypointX
      self.assertLess(330, x);
      self.assertGreater(340, x);

      y = hole_1.keypointY
      self.assertLess(100, y);
      self.assertGreater(110, y);

      # The second hole's bounding box is itself bounded in a bounding box
      self.assertLess(260, hole_1.verticeX)
      self.assertGreater(410, hole_1.verticeX)

      self.assertLess(70, hole_1.verticeY)
      self.assertGreater(140, hole_1.verticeY)


      # The minimum x coordinate of the bounding box's points
      minX = hole_1.verticeX

      # The minimum y coordinate of the bounding box's points
      minY = hole_1.verticeY

      # The maximum x coordinate of the bounding box's points
      maxX = hole_1.verticeX + 2 * (hole_1.keypointX - hole_1.verticeX) 

      # The maximum y coordinate of the bounding box's points
      maxY = hole_1.verticeY + 2 * (hole_1.keypointY - hole_1.verticeY) 



    # tests the function of the rgb node
    def test_rgb_node(self):

      self.assertTrue(self.replied)

      explorerCandidateHolesMsgs = self.messageList[2]

      # how many alert messages have been published
      self.assertEqual(len(explorerCandidateHolesMsgs), 2)

      # how many alerts have been sent with the first message
      self.assertEqual(len(explorerCandidateHolesMsgs[1].candidateHoles), 2)

      # make assertions about the first candidate hole
      hole_0 = explorerCandidateHolesMsgs[1].candidateHoles[0]

      # The first hole's keypoint
      x = hole_0.keypointX
      self.assertLess(345, x);
      self.assertGreater(348, x);

      y = hole_0.keypointY
      self.assertLess(236, y);
      self.assertGreater(238, y);

      # The first hole's bounding box is itself bounded
      # in a bounding box
      self.assertLess(255, hole_0.verticeX)
      self.assertGreater(440, hole_0.verticeX)

      self.assertLess(194, hole_0.verticeY)
      self.assertGreater(280, hole_0.verticeY)


      # The minimum x coordinate of the bounding box's points
      minX = hole_0.verticeX

      # The minimum x coordinate of the bounding box's points
      minY = hole_0.verticeY

      # The maximum x coordinate of the bounding box's points
      maxX = hole_0.verticeX + 2 * (hole_0.keypointX - hole_0.verticeX) 

      # The maximum y coordinate of the bounding box's points
      maxY = hole_0.verticeY + 2 * (hole_0.keypointY - hole_0.verticeY) 



      # make assertions about the second candidate hole
      hole_1 = explorerCandidateHolesMsgs[1].candidateHoles[1]


      # The second hole's keypoint
      x = hole_1.keypointX
      self.assertLess(320, x);
      self.assertGreater(330, x);

      y = hole_1.keypointY
      self.assertLess(100, y);
      self.assertGreater(110, y);


      # The second hole's bounding box is itself bounded in a bounding box
      self.assertLess(257, hole_1.verticeX)
      self.assertGreater(400, hole_1.verticeX)

      self.assertLess(70, hole_1.verticeY)
      self.assertGreater(145, hole_1.verticeY)


      # The minimum x coordinate of the bounding box's points
      minX = hole_1.verticeX

      # The minimum y coordinate of the bounding box's points
      minY = hole_1.verticeY

      # The maximum x coordinate of the bounding box's points
      maxX = hole_1.verticeX + 2 * (hole_1.keypointX - hole_1.verticeX) 

      # The maximum y coordinate of the bounding box's points
      maxY = hole_1.verticeY + 2 * (hole_1.keypointY - hole_1.verticeY) 




    #def test_enhanced_holes(self):

    #    self.assertTrue(self.replied)

    #    enhancedHolesMsgs = self.messageList[3]

    #    # how many alert messages have been published
    #    self.assertEqual(len(enhancedHolesMsgs), 1)

    #    # how many alerts have been sent with the first message
    #    self.assertEqual(len(enhancedHolesMsgs[0].enhancedHoles), 1)

    #    # depth analysis is possible
    #    self.assertTrue(enhancedHolesMsgs[0].isDepth)

    #    # the first hole
    #    hole_0 = enhancedHolesMsgs[0].enhancedHoles[0]

    #    # The first hole's keypoint
    #    x = hole_0.keypointX
    #    self.assertLess(350, x);
    #    self.assertGreater(360, x);

    #    y = hole_0.keypointY
    #    self.assertLess(230, y);
    #    self.assertGreater(240, y);

    #    # There should be equal number of elements in both vertices vectors
    #    self.assertEqual(len(hole_0.verticesX), len(hole_0.verticesY))

    #    # The first hole's bounding box is itself bounded
    #    # in a bounding box
    #    for i in range(0, len(hole_0.verticesX)):
    #        self.assertLess(255, hole_0.verticesX[i])
    #        self.assertGreater(450, hole_0.verticesX[i])

    #    for i in range(0, len(hole_0.verticesY)):
    #        self.assertLess(190, hole_0.verticesY[i])
    #        self.assertGreater(290, hole_0.verticesY[i])



if __name__ == '__main__':

  subscriber_topics = [
      ("/vision/holes_direction_alert", "pandora_vision_msgs", "HolesDirectionsVectorMsg"),
      ("/pandora_vision/hole_exploration/synchronized/depth/candidate_holes", "pandora_vision_msgs", "ExplorerCandidateHolesVectorMsg"),
      ("/pandora_vision/hole_exploration/synchronized/rgb/candidate_holes", "pandora_vision_msgs", "ExplorerCandidateHolesVectorMsg")]
      #    ("/vision/enhanced_holes", "pandora_vision_msgs", "EnhancedHolesVectorMsg")]
  publisher_topics = [
      ("/camera/depth_registered/points", "sensor_msgs", "PointCloud2")]

  rospy.sleep(15)
  rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
  rospy.loginfo("Test is Starting!")

  AccuracyTester.connect(subscriber_topics, publisher_topics, 2, False)
  #HoleExplorationTest.playFromBag(block = True)
  rospy.sleep(2)
  rostest.rosrun(PKG, NAME, AccuracyTester, sys.argv)
  AccuracyTester.disconnect()
