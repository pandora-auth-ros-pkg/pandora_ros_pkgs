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

PKG = 'pandora_integration_testing'
NAME = 'hole_data_fusion_test'

import sys

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import test_base
from pandora_testing_tools.testing_interface import alert_delivery
from pandora_testing_tools.mocks import mock_navigation
from pandora_testing_tools.mocks import mock_gui
from pandora_data_fusion_msgs.srv import GetObjectsSrv
from pandora_data_fusion_msgs.srv import GetObjectsSrvResponse
from state_manager_communications.msg import robotModeMsg
from geometry_msgs.msg import PoseStamped, Point

class DataFusionAgentTest(test_base.TestBase):

    @classmethod
    def establishMocks(cls):

        rospy.wait_for_service('/data_fusion/get_objects')
        cls.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        rospy.loginfo("setting up")
        cls.mockNavigation = mock_navigation.MockNavigation(
            do_exploration_topic = '/navigation/do_exploration',
            move_base_topic = '/move_base')
        rospy.loginfo("movk navi ok")
        cls.mockGui = mock_gui.MockGui(
            gui_validation_topic = '/gui/validate_victim')
        rospy.loginfo("movk gui ok")
        cls.deliveryBoy = alert_delivery.AlertDeliveryBoy()
        rospy.loginfo("alert deliverer ok")

        cls.victimPose = PoseStamped() 
        cls.victimBefore = Point()
        cls.victimAfter = Point()
        cls.victim_probability = 0.0

    #def tearDown(self):

    #    self.mockGui.__del__()
    #    self.mockNavigation.__del__()

    def test_part_a(self):
        
        rospy.sleep(0.5)
        #  A hole is found.
        self.deliveryBoy.deliverHoleOrder(orderYaw=0, orderPitch=0, orderProbability=0.8)
        self.deliveryBoy.deliverHoleOrder(orderYaw=0, orderPitch=0, orderProbability=0.9)
        self.deliveryBoy.deliverHoleOrder(orderYaw=0, orderPitch=0, orderProbability=0.9)
        #  Qualifies to a victim and is sent to agent.
        self.assertEqual(3, len(self.messageList[1]))
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        rospy.loginfo(self.messageList[1][0].victims)
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(0, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(0, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(0, len(visitedVictims))
        self.victimPose = outs.victimsToGo[0]
        rospy.sleep(1.)

    #def test_part_b(self):

        #  Agent recognises new victim and orders navigation to go to it.
        self.assertEqual(robotModeMsg.MODE_IDENTIFICATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.mockNavigation.navigation_succedes = True
        self.assertEqual(0, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(self.victimPose.pose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          self.victimPose.pose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.reply = True
        rospy.sleep(1.)

    #def test_part_c(self):

        #  Navigation went to the victim and a face is associated with the victim.
        self.assertEqual(robotModeMsg.MODE_DF_HOLD, self.messageList[0][-1].mode)
        self.deliveryBoy.deliverFaceOrder(orderYaw=0, orderPitch=0, orderProbability=0.85)
        self.deliveryBoy.deliverFaceOrder(orderYaw=0, orderPitch=0, orderProbability=0.9)
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(0, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(0, len(visitedVictims))
        self.victimBefore = outs.victimsToGo[0].pose.position
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(0, len(visitedVictims))
        self.assertEqual(["FACE"], victims[0].sensors)
        self.victim_probability = victims[0].probability
        rospy.sleep(12.)

    #def test_part_d(self):

        #  Agent recognises the change in victim and verifies it. Asks Gui for
        #  validation. Gui validates the victim.
        self.mockGui.victimValid = True
        self.assertEqual(self.victim_probability, self.mockGui.probability)
        self.assertEqual(["FACE"], self.mockGui.sensorIDsFound)
        self.mockGui.reply = True
        rospy.sleep(2.)

    #def test_part_e(self):

        #  Agent informs Data Fusion of victim's validation status and orders
        #  navigation to explore. Victim is transfered to victimsVisited list.
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsVisited))
        self.assertEqual(0, len(outs.victimsToGo))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(0, len(victims))
        self.assertEqual(1, len(visitedVictims))
        self.assertTrue(visitedVictims[0].valid)
        self.victimAfter = outs.victimsVisited[0].pose.position
        self.assertEqual(0, test_base.distance(self.victimBefore, self.victimAfter))
        self.assertEqual(robotModeMsg.MODE_EXPLORATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)
        rospy.sleep(1.)

    #def test_part_f(self):

        #  Another hole is detected and qualifies to victim.
        self.deliveryBoy.deliverHoleOrder(orderYaw=-0.5, orderPitch=0, orderProbability=0.5)
        self.deliveryBoy.deliverHoleOrder(orderYaw=-0.5, orderPitch=0, orderProbability=0.8)
        self.deliveryBoy.deliverHoleOrder(orderYaw=-0.5, orderPitch=0, orderProbability=0.8)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(1, len(visitedVictims))
        self.victimPose = outs.victimsToGo[0]
        rospy.sleep(1.)

    #def test_part_g(self):

        #  Agent is informed and orders navigation to go to it but navigation
        #  aborts the order...
        self.assertEqual(robotModeMsg.MODE_IDENTIFICATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.mockNavigation.navigation_succedes = False
        self.assertEqual(1, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(self.victimPose.pose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          self.victimPose.pose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.reply = True
        rospy.sleep(2.)

    #def test_part_h(self):

        #  Agents acknowledges victim as potential noise and orders Data Fusion
        #  to delete it. Navigation is ordered to return to exploration.
        outs = self.get_objects()
        self.assertEqual(1, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(0, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(0, len(victims))
        self.assertEqual(1, len(visitedVictims))
        self.assertEqual(robotModeMsg.MODE_EXPLORATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)
        rospy.sleep(1.)

    #def test_part_i(self):

        #  Another hole is detected and qualifies to a victim.
        self.deliveryBoy.deliverHoleOrder(orderYaw = 1, orderPitch = 0, orderProbability = 0.8)
        self.deliveryBoy.deliverHoleOrder(orderYaw = 1, orderPitch = 0, orderProbability = 0.8)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(1, len(visitedVictims))
        self.victimPose = outs.victimsToGo[0]
        rospy.sleep(1.)

    #def test_part_k(self):

        #  Agent recognises new victim and orders navigation to go to it.
        self.assertEqual(robotModeMsg.MODE_IDENTIFICATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.assertEqual(1, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(self.victimPose.pose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          self.victimPose.pose.orientation, self.mockNavigation.target_orientation))
        rospy.sleep(1.5)

    #def test_part_l(self):

        #  Current victim is thought to be displaced.
        self.deliveryBoy.deliverHoleOrder(orderYaw=0.97, orderPitch=0, orderProbability=0.9)
        self.deliveryBoy.deliverHoleOrder(orderYaw=0.95, orderPitch=0, orderProbability=0.9)
        self.deliveryBoy.deliverHoleOrder(orderYaw=0.94, orderPitch=0, orderProbability=0.9)
        self.assertEqual(1, len(self.messageList[1][-1].victims))
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(1, len(outs.victimsToGo))
        self.assertEqual(1, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(1, len(victims))
        self.assertEqual(1, len(visitedVictims))
        self.victimPose = outs.victimsToGo[0]
        rospy.sleep(1.)

    #def test_part_m(self):

        #  Agent think that displacement cannot be ignored and re-orders navigation.
        #  Navigation target has changed.
        self.assertEqual(robotModeMsg.MODE_IDENTIFICATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.moved_base)
        self.assertEqual(4, self.mockNavigation.preempted)
        self.assertEqual(0, 
            test_base.distance(self.victimPose.pose.position, self.mockNavigation.target_position))
        self.assertTrue(test_base.isSameOrientation(
          self.victimPose.pose.orientation, self.mockNavigation.target_orientation))
        self.mockNavigation.navigation_succedes = True
        self.mockNavigation.reply = True
        rospy.sleep(1.)

    #def test_part_n(self):

        #  Agent waits for verification but victim fails to verify...
        #  Agent orders Data Fusion to think this victim as non valid.
        #  Navigation returns to exploration.
        rospy.sleep(15.)
        outs = self.get_objects()
        self.assertEqual(2, len(outs.holes))
        self.assertEqual(1, len(outs.faces))
        self.assertEqual(0, len(outs.victimsToGo))
        self.assertEqual(2, len(outs.victimsVisited))
        victims = self.messageList[1][-1].victims
        visitedVictims = self.messageList[1][-1].visitedVictims
        self.assertEqual(0, len(victims))
        self.assertEqual(2, len(visitedVictims))
        self.assertFalse(visitedVictims[1].valid)
        self.assertEqual(robotModeMsg.MODE_EXPLORATION, self.messageList[0][-1].mode)
        self.assertTrue(self.mockNavigation.entered_exploration)

if __name__ == '__main__':

    rospy.sleep(5.)
    rospy.init_node(NAME)
    subscriber_topics = [
        ("/robot/state/clients", "state_manager_communications", "robotModeMsg"),
        ("/data_fusion/world_model", "pandora_data_fusion_msgs", "WorldModelMsg")]
    DataFusionAgentTest.connect(subscriber_topics, list(), robotModeMsg.MODE_START_AUTONOMOUS, False)
    DataFusionAgentTest.establishMocks()
    rostest.rosrun(PKG, NAME, DataFusionAgentTest, sys.argv)
    DataFusionAgentTest.disconnect()

