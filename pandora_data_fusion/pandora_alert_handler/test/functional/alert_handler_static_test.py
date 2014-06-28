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

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_static_test'

import os
import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import alert_delivery
from test_base import distance
from test_base import direction
import test_base

DIR = os.path.dirname(os.path.realpath(__file__))

class AlertHandlerStaticTest(test_base.TestBase):

    def test_a_simple_alert(self):

        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        outs = []
        self.fillInfo(outs)
        pose = outs[0].hazmats.pop().pose

        self.assertAlmostEqual(pose.position.x, 0.99525856, 3)
        self.assertAlmostEqual(pose.position.y, 0.52000838, 3)
        self.assertAlmostEqual(pose.position.z, 1)
        self.assertAlmostEqual(pose.orientation.x, 0)
        self.assertAlmostEqual(pose.orientation.y, 0)
        self.assertAlmostEqual(pose.orientation.z, 0.70710679, 3)
        self.assertAlmostEqual(pose.orientation.w, 0.70710679, 3)

    def test_objects_coexist(self):

        self.deliveryBoy.getOrderListFromBoss(DIR + '/orders/mixed_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            self.fillInfo(outs)
        
        self.assertEqual(len(outs[0].holes), 1)
        self.assertEqual(len(outs[0].hazmats), 0)
        self.assertEqual(len(outs[0].qrs), 0)
        self.assertEqual(len(outs[0].thermals), 0)
        self.assertEqual(len(outs[0].dataMatrices), 0)
        self.assertEqual(len(outs[0].landoltcs), 0)
        self.assertEqual(len(outs[0].faces), 0)
        self.assertEqual(len(outs[0].motions), 0)
        self.assertEqual(len(outs[0].sounds), 0)
        self.assertEqual(len(outs[0].co2s), 0)

        self.assertEqual(len(outs[1].holes), 1)
        self.assertEqual(len(outs[1].hazmats), 0)
        self.assertEqual(len(outs[1].qrs), 1)
        self.assertEqual(len(outs[1].thermals), 0)
        self.assertEqual(len(outs[1].dataMatrices), 0)
        self.assertEqual(len(outs[1].landoltcs), 0)
        self.assertEqual(len(outs[1].faces), 0)
        self.assertEqual(len(outs[1].motions), 0)
        self.assertEqual(len(outs[1].sounds), 0)
        self.assertEqual(len(outs[1].co2s), 0)

        self.assertEqual(len(outs[2].holes), 1)
        self.assertEqual(len(outs[2].hazmats), 1)
        self.assertEqual(len(outs[2].qrs), 1)
        self.assertEqual(len(outs[2].thermals), 0)
        self.assertEqual(len(outs[2].dataMatrices), 0)
        self.assertEqual(len(outs[2].landoltcs), 0)
        self.assertEqual(len(outs[2].faces), 0)
        self.assertEqual(len(outs[2].motions), 0)
        self.assertEqual(len(outs[2].sounds), 0)
        self.assertEqual(len(outs[2].co2s), 0)

        self.assertEqual(len(outs[3].holes), 1)
        self.assertEqual(len(outs[3].hazmats), 1)
        self.assertEqual(len(outs[3].qrs), 1)
        self.assertEqual(len(outs[3].thermals), 0)
        self.assertEqual(len(outs[3].dataMatrices), 0)
        self.assertEqual(len(outs[3].landoltcs), 0)
        self.assertEqual(len(outs[3].faces), 0)
        self.assertEqual(len(outs[3].motions), 0)
        self.assertEqual(len(outs[3].sounds), 0)
        self.assertEqual(len(outs[3].co2s), 0)

        self.assertEqual(len(outs[4].holes), 1)
        self.assertEqual(len(outs[4].hazmats), 1)
        self.assertEqual(len(outs[4].qrs), 1)
        self.assertEqual(len(outs[4].thermals), 1)
        self.assertEqual(len(outs[4].dataMatrices), 0)
        self.assertEqual(len(outs[4].landoltcs), 0)
        self.assertEqual(len(outs[4].faces), 0)
        self.assertEqual(len(outs[4].motions), 0)
        self.assertEqual(len(outs[4].sounds), 0)
        self.assertEqual(len(outs[4].co2s), 0)

        self.assertEqual(len(outs[5].holes), 1)
        self.assertEqual(len(outs[5].hazmats), 1)
        self.assertEqual(len(outs[5].qrs), 1)
        self.assertEqual(len(outs[5].thermals), 1)
        self.assertEqual(len(outs[5].dataMatrices), 0)
        self.assertEqual(len(outs[5].landoltcs), 0)
        self.assertEqual(len(outs[5].faces), 1)
        self.assertEqual(len(outs[5].motions), 0)
        self.assertEqual(len(outs[5].sounds), 0)
        self.assertEqual(len(outs[5].co2s), 0)

        self.assertEqual(len(outs[6].holes), 1)
        self.assertEqual(len(outs[6].hazmats), 1)
        self.assertEqual(len(outs[6].qrs), 1)
        self.assertEqual(len(outs[6].thermals), 1)
        self.assertEqual(len(outs[6].dataMatrices), 0)
        self.assertEqual(len(outs[6].landoltcs), 0)
        self.assertEqual(len(outs[6].faces), 1)
        self.assertEqual(len(outs[6].motions), 1)
        self.assertEqual(len(outs[6].sounds), 0)
        self.assertEqual(len(outs[6].co2s), 0)

        self.assertEqual(len(outs[7].holes), 1)
        self.assertEqual(len(outs[7].hazmats), 1)
        self.assertEqual(len(outs[7].qrs), 1)
        self.assertEqual(len(outs[7].thermals), 1)
        self.assertEqual(len(outs[7].dataMatrices), 0)
        self.assertEqual(len(outs[7].landoltcs), 0)
        self.assertEqual(len(outs[7].faces), 1)
        self.assertEqual(len(outs[7].motions), 1)
        self.assertEqual(len(outs[7].sounds), 1)
        self.assertEqual(len(outs[7].co2s), 0)

        self.assertEqual(len(outs[8].holes), 1)
        self.assertEqual(len(outs[8].hazmats), 1)
        self.assertEqual(len(outs[8].qrs), 1)
        self.assertEqual(len(outs[8].thermals), 1)
        self.assertEqual(len(outs[8].dataMatrices), 0)
        self.assertEqual(len(outs[8].landoltcs), 0)
        self.assertEqual(len(outs[8].faces), 1)
        self.assertEqual(len(outs[8].motions), 1)
        self.assertEqual(len(outs[8].sounds), 1)
        self.assertEqual(len(outs[8].co2s), 1)

        self.assertEqual(len(outs[9].holes), 1)
        self.assertEqual(len(outs[9].hazmats), 1)
        self.assertEqual(len(outs[9].qrs), 1)
        self.assertEqual(len(outs[9].thermals), 1)
        self.assertEqual(len(outs[9].dataMatrices), 0)
        self.assertEqual(len(outs[9].landoltcs), 1)
        self.assertEqual(len(outs[9].faces), 1)
        self.assertEqual(len(outs[9].motions), 1)
        self.assertEqual(len(outs[9].sounds), 1)
        self.assertEqual(len(outs[9].co2s), 1)

        self.assertEqual(len(outs[10].holes), 1)
        self.assertEqual(len(outs[10].hazmats), 1)
        self.assertEqual(len(outs[10].qrs), 1)
        self.assertEqual(len(outs[10].thermals), 1)
        self.assertEqual(len(outs[10].dataMatrices), 1)
        self.assertEqual(len(outs[10].landoltcs), 1)
        self.assertEqual(len(outs[10].faces), 1)
        self.assertEqual(len(outs[10].motions), 1)
        self.assertEqual(len(outs[10].sounds), 1)
        self.assertEqual(len(outs[10].co2s), 1)

    def test_kalman_filter_of_one_object(self):
        
        self.deliveryBoy.getOrderListFromBoss(DIR + '/orders/one_kalman_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            self.fillInfo(outs)
            # The order had only holes in it!
            self.assertEqual(len(outs[-1].holes), 1)
            self.assertEqual(len(outs[-1].hazmats), 0)
            self.assertEqual(len(outs[-1].qrs), 0)
            self.assertEqual(len(outs[-1].thermals), 0)
            self.assertEqual(len(outs[-1].faces), 0)
            self.assertEqual(len(outs[-1].motions), 0)
            self.assertEqual(len(outs[-1].sounds), 0)
            self.assertEqual(len(outs[-1].co2s), 0)
            self.assertEqual(len(outs[-1].landoltcs), 0)
            self.assertEqual(len(outs[-1].dataMatrices), 0)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[1].holes[0].pose.position
        position4 = outs[4].holes[0].pose.position
        position5 = outs[5].holes[0].pose.position
        position6 = outs[6].holes[0].pose.position
        
        # If measurement does not differ from the expected position, the updated
        # expected position will not change.
        self.assertEqual(position0.x, position1.x)
        self.assertEqual(position0.y, position1.y)
        self.assertEqual(position0.z, position1.z)
        # I expect that 3-4 alerts would bring the conviction high enough to be
        # recognised as a victim, but no less.
        self.assertEqual(len(outs[0].victimsToGo), 0)
        self.assertEqual(len(outs[1].victimsToGo), 0)
        for i in range(3, 12):
            if len(outs[i-1].victimsToGo) == 1:
                break
        rospy.logdebug("Victim found in %s-th alert!", str(i))
        self.assertTrue(i == 3 or i == 4)
        # If the measurement differs the same way, then the updated expected position
        # should draw closer to that measurement - towards the same direction.  That
        # means that expected position's distance from the initial expected position
        # should be greater as the different measurement insists.
        self.assertLess(distance(position0, position4), distance(position0, position5))
        dir4 = direction(position0, position4)
        dir5 = direction(position0, position5)
        self.assertAlmostEqual(dir4.x, dir5.x)
        self.assertAlmostEqual(dir4.y, dir5.y)
        self.assertAlmostEqual(dir4.z, dir5.z)

        self.assertLess(distance(position0, position6), distance(position0, position5))

        # We consider that conviction of the object about its position is lot higher now.
        # I assume that its expected position will be around position0.  So:
        self.assertLess(distance(position0, outs[10].holes[0].pose.position), 0.05)

    def test_robustness_of_conviction(self):

        self.deliveryBoy.getOrderListFromBoss(DIR + '/orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            self.fillInfo(outs)
            # The order had only holes in it!
            self.assertEqual(len(outs[-1].holes), 1)
            self.assertEqual(len(outs[-1].hazmats), 0)
            self.assertEqual(len(outs[-1].qrs), 0)
            self.assertEqual(len(outs[-1].thermals), 0)
            if len(outs) > 1:
                self.assertEqual(distance(outs[-1].holes[0].pose.position, 
                  outs[-2].holes[0].pose.position), 0)

        # A measurement off will not throw away very much a stable object.
        self.deliveryBoy.deliverHoleOrder(0.13, 0, 1)
        self.fillInfo(outs)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[-1].holes[0].pose.position
        distanceLessConviction = distance(position0, position1)
        rospy.logdebug("Less conviction distance: %f", distanceLessConviction)
        self.assertLess(distanceLessConviction, 0.03)
        
        self.setUp()
        self.deliveryBoy.getOrderListFromBoss(DIR + '/orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break

        # That measurement off will throw away the object even less, if more
        # stable measurements have occured.
        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        self.fillInfo(outs)
        self.deliveryBoy.deliverHoleOrder(0.13, 0, 1)
        self.fillInfo(outs)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[1].holes[0].pose.position
        distanceMoreConviction = distance(position0, position1)
        rospy.logdebug("More conviction distance: %f", distanceMoreConviction)
        self.assertLess(distanceMoreConviction, distanceLessConviction)

    def test_kalman_resistance_to_gaussian(self):

        import numpy as np

        mu, sigma, size = 0, 0.03, 10
        k = np.random.normal(mu, sigma, size)
        l = np.random.normal(mu, sigma, size)
        yaw = np.arctan(k)
        pitch = np.arctan(l * np.cos(yaw))
        out = []
        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        rospy.sleep(0.1)
        self.fillInfo(out)
        for i in range(size):
            self.deliveryBoy.deliverHoleOrder(yaw[i], pitch[i], 1)
        self.fillInfo(out)
        self.assertEqual(len(out[0].holes), 1)
        self.assertEqual(len(out[0].victimsToGo), 0)
        self.assertEqual(len(out[1].holes), 1)
        self.assertEqual(len(out[1].victimsToGo), 1)
        self.assertEqual(distance(out[1].holes[0].pose.position,
          out[1].victimsToGo[0].pose.position), 0)
        # Filtering makes object resistant to gaussian noise!
        self.assertLess(distance(out[0].holes[0].pose.position,
          out[1].victimsToGo[0].pose.position), 0.04)

    def test_victim_pipeline(self):

        self.assertEqual(self.worldModelPublished, 0)
        self.deliveryBoy.deliverHoleOrder(0, 0, 1, 0.8)
        self.deliveryBoy.deliverHoleOrder(0, 0, 1, 0.6)
        self.assertEqual(self.worldModelPublished, 2)
        self.assertEqual(len(self.currentVictimList), 1)
        self.assertEqual(len(self.currentVictimList[0].sensors), 0)
        self.assertEqual(self.currentVictimList[0].probability, 0)
        self.deliveryBoy.deliverThermalOrder(0, 0, 0.6)
        self.deliveryBoy.deliverThermalOrder(0, 0, 0.6)
        self.assertEqual(self.worldModelPublished, 4)
        self.assertEqual(len(self.currentVictimList), 1)
        self.assertEqual(len(self.currentVictimList[0].sensors), 1)
        self.assertEqual(set(self.currentVictimList[0].sensors), 
            set(['THERMAL']))
        self.assertGreater(self.currentVictimList[0].probability, 0.6/2)
        self.deliveryBoy.deliverCo2Order(0, 0, 0.8)
        self.deliveryBoy.deliverCo2Order(0, 0, 0.8)
        self.assertEqual(self.worldModelPublished, 6)
        self.assertEqual(len(self.currentVictimList), 1)
        self.assertEqual(len(self.currentVictimList[0].sensors), 2)
        self.assertEqual(set(self.currentVictimList[0].sensors), 
            set(['THERMAL', 'CO2']))
        self.assertGreater(self.currentVictimList[0].probability, (0.6+0.8)/2)
        self.deliveryBoy.deliverFaceOrder(0, 0, 0.9)
        self.deliveryBoy.deliverFaceOrder(0, 0, 0.93)
        self.assertEqual(self.worldModelPublished, 8)
        self.assertEqual(len(self.currentVictimList), 1)
        self.assertEqual(len(self.currentVictimList[0].sensors), 3)
        self.assertEqual(set(self.currentVictimList[0].sensors), 
            set(['THERMAL', 'FACE', 'CO2']))
        #self.assertGreater(self.currentVictimList[0].probability, 0.9) #

if __name__ == '__main__':

    rospy.sleep(2)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    AlertHandlerStaticTest.connect()
    rostest.rosrun(PKG, NAME, AlertHandlerStaticTest, sys.argv)
    AlertHandlerStaticTest.disconnect()

