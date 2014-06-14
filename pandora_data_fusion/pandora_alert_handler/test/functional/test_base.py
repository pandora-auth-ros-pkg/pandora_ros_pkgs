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

import math

import unittest

import rospy

from pandora_testing_tools.testing_interface import alert_delivery
from pandora_data_fusion_msgs.msg import WorldModelMsg 
from pandora_data_fusion_msgs.msg import VictimInfoMsg 
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

    def mockCallback(self, data):

      self.currentVictimList = data.victims
      self.worldModelPublished += 1
      #rospy.logdebug("yolo!"+str(self.worldModelPublished))
      #rospy.logdebug(self.currentVictimList)

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

        self.subscriber = rospy.Subscriber("/data_fusion/world_model", WorldModelMsg, self.mockCallback)
        self.currentVictimList = []
        self.worldModelPublished = 0
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

    def tearDown(self):

        self.subscriber.unregister()
      
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

