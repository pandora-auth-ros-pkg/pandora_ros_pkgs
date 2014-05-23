#!/usr/bin/env python
# Copyright <alert_handler_test.py>

import math

import unittest

import rospy

import alert_delivery

from pandora_data_fusion_msgs.srv import GetObjectsSrv
from pandora_data_fusion_msgs.srv import GetObjectsSrvResponse
from std_srvs.srv import Empty
from geometry_msgs.msg import Point

def distance(a, b):

    return math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2 )

def direction(a, b):
        
    dire = Point()
    norm = distance(a, b)
    dire.x = (b.x - a.x)/norm
    dire.y = (b.y - a.y)/norm
    dire.z = (b.z - a.z)/norm
    return dire

class TestBase(unittest.TestCase):
     
    deliveryBoy = alert_delivery.AlertDeliveryBoy()

    @classmethod
    def connect(cls):

        cls.get_objects = rospy.ServiceProxy('/data_fusion/get_objects', GetObjectsSrv, True)
        rospy.wait_for_service('/data_fusion/get_objects')
        cls.flush_lists = rospy.ServiceProxy('/data_fusion/flush_queues', Empty, True)
        rospy.wait_for_service('/data_fusion/flush_queues')
        
    @classmethod
    def disconnect(cls):

        cls.get_objects.close()
        cls.flush_lists.close()

    def setUp(self):

        i = 0
        self.deliveryBoy.deliverHazmatOrder(0, 0, 1)
        rospy.sleep(0.2)
        while(True):
            try:
                self.flush_lists()
                break
            except rospy.ServiceException as exc:
                if (i > 3):
                    raise rospy.ServiceException()
                rospy.logdebug("!< flush_lists service failed >! reconnecting and retrying...")
                i += 1
                self.connect()
        self.deliveryBoy.clearOrderList()
      
    def fillInfo(self, outs):

        i = 0
        while(True):
            try:
                outs.append(self.get_objects())
                break
            except rospy.ServiceException as exc:
                if (i > 3):
                    raise rospy.ServiceException()
                rospy.logdebug("!< get_objects service failed >! reconnecting and retrying...")
                i += 1
                self.connect()

