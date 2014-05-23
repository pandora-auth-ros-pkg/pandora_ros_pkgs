#!/usr/bin/env python
# Copyright <alert_handler_static_test.py>

PKG = 'pandora_alert_handler'
NAME = 'alert_handler_static_test'

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

import alert_delivery
from test_base import distance
from test_base import direction
import test_base

class AlertHandlerStaticTest(test_base.TestBase):

    def test_works(self):

        self.assertEqual(1+1, 2)

    def test_a_simple_alert(self):

        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(0.1)
        outs = []
        self.fillInfo(outs)
        pose = outs[0].hazmats.pop().pose

        self.assertAlmostEqual(pose.position.x, 0.99525856)
        self.assertAlmostEqual(pose.position.y, 0.52000838)
        self.assertAlmostEqual(pose.position.z, 1)
        self.assertAlmostEqual(pose.orientation.x, 0)
        self.assertAlmostEqual(pose.orientation.y, 0)
        self.assertAlmostEqual(pose.orientation.z, 0.70710679)
        self.assertAlmostEqual(pose.orientation.w, 0.70710679)

    def test_objects_coexist(self):

        self.deliveryBoy.getOrderListFromBoss('orders/mixed_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
        
        self.assertEqual(len(outs[0].holes), 1)
        self.assertEqual(len(outs[0].hazmats), 0)
        self.assertEqual(len(outs[0].qrs), 0)
        self.assertEqual(len(outs[0].thermals), 0)
        self.assertEqual(len(outs[1].holes), 1)
        self.assertEqual(len(outs[1].hazmats), 0)
        self.assertEqual(len(outs[1].qrs), 1)
        self.assertEqual(len(outs[1].thermals), 0)
        self.assertEqual(len(outs[2].holes), 1)
        self.assertEqual(len(outs[2].hazmats), 1)
        self.assertEqual(len(outs[2].qrs), 1)
        self.assertEqual(len(outs[2].thermals), 0)
        self.assertEqual(len(outs[3].holes), 1)
        self.assertEqual(len(outs[3].hazmats), 1)
        self.assertEqual(len(outs[3].qrs), 1)
        self.assertEqual(len(outs[3].thermals), 0)
        self.assertEqual(len(outs[4].holes), 1)
        self.assertEqual(len(outs[4].hazmats), 1)
        self.assertEqual(len(outs[4].qrs), 1)
        self.assertEqual(len(outs[4].thermals), 1)

    def test_kalman_filter_of_one_object(self):
        
        self.deliveryBoy.getOrderListFromBoss('orders/one_kalman_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
            # The order had only holes in it!
            self.assertEqual(len(outs[-1].holes), 1)
            self.assertEqual(len(outs[-1].hazmats), 0)
            self.assertEqual(len(outs[-1].qrs), 0)
            self.assertEqual(len(outs[-1].thermals), 0)
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

        self.deliveryBoy.getOrderListFromBoss('orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
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
        rospy.sleep(0.1)
        self.fillInfo(outs)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[-1].holes[0].pose.position
        distanceLessConviction = distance(position0, position1)
        rospy.logdebug("Less conviction distance: %f", distanceLessConviction)
        self.assertLess(distanceLessConviction, 0.03)
        
        self.setUp()
        self.deliveryBoy.getOrderListFromBoss('orders/frequent_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)

        # That measurement off will throw away the object even less, if more
        # stable measurements have occured.
        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        rospy.sleep(0.1)
        self.fillInfo(outs)
        self.deliveryBoy.deliverHoleOrder(0.13, 0, 1)
        rospy.sleep(0.1)
        self.fillInfo(outs)
        position0 = outs[0].holes[0].pose.position
        position1 = outs[1].holes[0].pose.position
        distanceMoreConviction = distance(position0, position1)
        rospy.logdebug("More conviction distance: %f", distanceMoreConviction)
        self.assertLess(distanceMoreConviction, distanceLessConviction)
        

    def test_2_objects_colliding(self):

        self.deliveryBoy.getOrderListFromBoss('orders/but_it_was_the_same_order.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)
        
        self.assertEqual(len(outs[0].victimsToGo), 0)
        self.assertEqual(len(outs[0].holes), 1)

        self.assertEqual(len(outs[1].victimsToGo), 0)
        self.assertEqual(len(outs[1].holes), 1)

        self.assertEqual(len(outs[2].victimsToGo), 1)
        self.assertEqual(len(outs[2].holes), 1)
        self.assertEqual(distance(outs[2].holes[0].pose.position,
          outs[2].victimsToGo[0].pose.position), 0)

        self.assertEqual(len(outs[3].victimsToGo), 1)
        self.assertEqual(len(outs[3].holes), 2)
        self.assertEqual(distance(outs[3].holes[0].pose.position,
          outs[3].victimsToGo[0].pose.position), 0)

        self.assertEqual(len(outs[4].victimsToGo), 1)
        self.assertEqual(len(outs[4].holes), 2)
        self.assertEqual(distance(outs[4].holes[0].pose.position,
          outs[4].victimsToGo[0].pose.position), 0)
        holesDistancePrev = distance(outs[4].holes[0].pose.position,
            outs[4].holes[1].pose.position)

        # New alert updates both holes, so that they will close their distance.
        # If this pattern continues, eventually will result in the holes being 
        # clustered as one victim, which replaces the previous victims and 
        # locates itself between the 2 holes.
        # Also, hole0 has bigger conviction that hole1, so victim will be closer
        # to hole0 than to hole1.
        self.assertEqual(len(outs[5].victimsToGo), 2)
        self.assertEqual(len(outs[5].holes), 2)
        holesDistanceNext = distance(outs[5].holes[0].pose.position,
            outs[5].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertGreater(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[5].holes[0].pose.position,
          outs[5].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[5].holes[1].pose.position,
          outs[5].victimsToGo[1].pose.position)
        self.assertLess(holesDistanceNext, holesDistancePrev)
        self.assertEqual(victim2Hole0, 0)
        self.assertEqual(victim2Hole1, 0)
        holesDistancePrev = float(holesDistanceNext)

        self.assertEqual(len(outs[6].victimsToGo), 1)
        self.assertEqual(len(outs[6].holes), 2)
        holesDistanceNext = distance(outs[6].holes[0].pose.position,
            outs[6].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertLess(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[6].holes[0].pose.position,
          outs[6].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[6].holes[1].pose.position,
          outs[6].victimsToGo[0].pose.position)
        self.assertLess(holesDistanceNext, holesDistancePrev)
        self.assertLess(victim2Hole0, holesDistanceNext)
        self.assertLess(victim2Hole1, holesDistanceNext)
        self.assertLess(victim2Hole0, victim2Hole1)
        holesDistancePrev = float(holesDistanceNext)

        # This alert will update only hole0. Its position will shift
        # slightly to the left. Still hole0 and hole1 are grouped as
        # one victim.
        self.assertEqual(len(outs[7].victimsToGo), 1)
        self.assertEqual(len(outs[7].holes), 2)
        self.assertEqual(distance(outs[6].holes[1].pose.position, 
          outs[7].holes[1].pose.position), 0)
        holesDistanceNext = distance(outs[7].holes[0].pose.position,
            outs[7].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertLess(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[7].holes[0].pose.position,
          outs[7].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[7].holes[1].pose.position,
          outs[7].victimsToGo[0].pose.position)
        self.assertGreater(holesDistanceNext, holesDistancePrev)
        self.assertLess(victim2Hole0, holesDistanceNext)
        self.assertLess(victim2Hole1, holesDistanceNext)
        self.assertLess(victim2Hole0, victim2Hole1)
        holesDistancePrev = float(holesDistanceNext)

        # Next alerts are similar to the previous one, so that they will
        # update only hole0 - shifting it to the left - and not hole1.
        # To to filter's convergence shifting is goinf to be very slight,
        # so they will still be clustered as one victim.
        self.assertEqual(len(outs[8].victimsToGo), 1)
        self.assertEqual(len(outs[8].holes), 2)
        self.assertEqual(distance(outs[7].holes[1].pose.position, 
          outs[8].holes[1].pose.position), 0)
        holesDistanceNext = distance(outs[8].holes[0].pose.position,
            outs[8].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertLess(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[8].holes[0].pose.position,
          outs[8].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[8].holes[1].pose.position,
          outs[8].victimsToGo[0].pose.position)
        self.assertGreater(holesDistanceNext, holesDistancePrev)
        self.assertLess(victim2Hole0, holesDistanceNext)
        self.assertLess(victim2Hole1, holesDistanceNext)
        self.assertLess(victim2Hole0, victim2Hole1)
        holesDistancePrev = float(holesDistanceNext)

        self.assertEqual(len(outs[11].victimsToGo), 1)
        self.assertEqual(len(outs[11].holes), 2)
        self.assertEqual(distance(outs[10].holes[1].pose.position, 
          outs[11].holes[1].pose.position), 0)
        holesDistanceNext = distance(outs[11].holes[0].pose.position,
            outs[11].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertLess(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[11].holes[0].pose.position,
          outs[11].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[11].holes[1].pose.position,
          outs[11].victimsToGo[0].pose.position)
        self.assertGreater(holesDistanceNext, holesDistancePrev)
        self.assertLess(victim2Hole0, holesDistanceNext)
        self.assertLess(victim2Hole1, holesDistanceNext)
        self.assertLess(victim2Hole0, victim2Hole1)
        holesDistancePrev = float(holesDistanceNext)

    def test_2_objects_colliding2(self):

        self.deliveryBoy.getOrderListFromBoss('orders/but_it_was_the_same_order2.in')
        outs = []
        while(True):
            try:      
                self.deliveryBoy.deliverNextOrder()
            except alert_delivery.BadBossOrderFile as exc:
                break
            rospy.sleep(0.1)
            self.fillInfo(outs)

        self.assertEqual(len(outs[0].victimsToGo), 0)
        self.assertEqual(len(outs[0].holes), 1)

        self.assertEqual(len(outs[1].victimsToGo), 0)
        self.assertEqual(len(outs[1].holes), 1)

        self.assertEqual(len(outs[2].victimsToGo), 0)
        self.assertEqual(len(outs[2].holes), 2)

        self.assertEqual(len(outs[3].victimsToGo), 0)
        self.assertEqual(len(outs[3].holes), 2)
        holesDistancePrev = distance(outs[3].holes[0].pose.position,
            outs[3].holes[1].pose.position)
    
        # New alert updates hole1, so that they will close their distance.
        # If this pattern continues, eventually will result in the holes being 
        # clustered as one victim, which locates itself between the 2 holes.
        # Also, hole1 has bigger conviction that hole0, so victim will be closer
        # to hole1 than to hole0.
        self.assertEqual(len(outs[5].victimsToGo), 1)
        self.assertEqual(len(outs[5].holes), 2)
        holesDistanceNext = distance(outs[5].holes[0].pose.position,
            outs[5].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertLess(holesDistanceNext, 0.2)  ##
        victim2Hole0 = distance(outs[5].holes[0].pose.position,
          outs[5].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[5].holes[1].pose.position,
          outs[5].victimsToGo[0].pose.position)
        self.assertLess(holesDistanceNext, holesDistancePrev)
        self.assertLess(victim2Hole0, holesDistanceNext) ##
        self.assertLess(victim2Hole1, holesDistanceNext)
        self.assertGreater(victim2Hole0, victim2Hole1)
        holesDistancePrev = float(holesDistanceNext)
    
        # This alert will update only hole0. Its position will shift
        # slightly to the left.         
        self.assertEqual(len(outs[6].victimsToGo), 2)
        self.assertEqual(len(outs[6].holes), 2)
        self.assertEqual(distance(outs[5].holes[1].pose.position, 
          outs[6].holes[1].pose.position), 0)
        holesDistanceNext = distance(outs[6].holes[0].pose.position,
            outs[6].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertGreater(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[6].holes[0].pose.position,
          outs[6].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[6].holes[1].pose.position,
          outs[6].victimsToGo[1].pose.position)
        self.assertGreater(holesDistanceNext, holesDistancePrev)
        self.assertEqual(victim2Hole0, 0)
        self.assertEqual(victim2Hole1, 0)
    
        # Next alerts are similar to the previous one, so that they will
        # update only hole0 - shifting it to the left - and not hole1.
        # Holes stop being considered as the same victim. This results in
        # the last victim being deleted and replaced by two new victims
        # represented by each hole.
        self.assertEqual(len(outs[9].victimsToGo), 2)
        self.assertEqual(len(outs[9].holes), 2)
        self.assertEqual(distance(outs[8].holes[1].pose.position, 
          outs[9].holes[1].pose.position), 0)
        holesDistanceNext = distance(outs[9].holes[0].pose.position,
            outs[9].holes[1].pose.position)
        rospy.logdebug("holesDistance: %f", holesDistanceNext)
        self.assertGreater(holesDistanceNext, 0.2)
        victim2Hole0 = distance(outs[9].holes[0].pose.position,
          outs[9].victimsToGo[0].pose.position)
        victim2Hole1 = distance(outs[9].holes[1].pose.position,
          outs[9].victimsToGo[1].pose.position)
        self.assertGreater(holesDistanceNext, holesDistancePrev)
        self.assertEqual(victim2Hole0, 0)
        self.assertEqual(victim2Hole1, 0)

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
            rospy.sleep(0.1)
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


if __name__ == '__main__':

    rospy.sleep(1)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    AlertHandlerStaticTest.connect()
    rostest.rosrun(PKG, NAME, AlertHandlerStaticTest, sys.argv)
    AlertHandlerStaticTest.disconnect()

