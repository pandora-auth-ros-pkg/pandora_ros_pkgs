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

PKG = 'pandora_sensor_processing'
NAME = 'co2_processor_test'

import sys
import math

import unittest

import roslib; roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import test_base
from pandora_arm_hardware_interface.msg import Co2Msg
from pandora_common_msgs.msg import GeneralAlertMsg 

class Co2ProcessorTest(test_base.TestBase):
    
    def publish_raw(self, percentage):

        self.raw = Co2Msg()
        self.raw.header.frame_id = "aer"
        self.raw.header.stamp = rospy.get_rostime()
        self.raw.co2_percentage = percentage

        self.publishers[0].publish(self.raw)
        self.replied = False
        rospy.sleep(0.1)

    def expect_success(self):

        self.assertTrue(self.replied)
        times = -1
        if len(self.alertList) == 1:
          times = 0
        self.assertGreater(self.alertList[times].probability, 0.4)
        self.assertEqual(self.alertList[times].yaw, 0)
        self.assertEqual(self.alertList[times].pitch, 0)
        self.assertEqual(self.alertList[times].header.frame_id, "aer")

    def expect_fail(self):

        self.assertFalse(self.replied)

    def test_mode_exploration(self):

        self.state_changer.transition_to_state(2)
        rospy.sleep(1.)
        self.publish_raw(0.1)
        self.publish_raw(0.15)
        self.expect_fail()

    def test_mode_identification(self):

        self.state_changer.transition_to_state(3)
        rospy.sleep(1.)
        self.publish_raw(0.1)
        self.expect_fail()
        rospy.sleep(0.3)
        self.publish_raw(0.2)
        self.expect_success()
        rospy.sleep(0.3)
        self.publish_raw(0.2)
        self.expect_success()
        rospy.sleep(3.)
        self.publish_raw(0.2)
        self.expect_fail()
        rospy.sleep(0.2)
        self.publish_raw(0.3)
        self.expect_success()
        rospy.sleep(0.2)
        self.publish_raw(0.125)
        self.expect_fail()
        rospy.sleep(0.2)
        self.publish_raw(0.3)
        self.expect_success()

    def test_toggle(self):

        self.state_changer.transition_to_state(2)
        rospy.sleep(1.)
        self.publish_raw(0.1)
        self.expect_fail()
        self.state_changer.transition_to_state(2)
        rospy.sleep(1.)
        self.publish_raw(0.1)
        self.expect_fail()
        self.state_changer.transition_to_state(3)
        rospy.sleep(1.)
        self.publish_raw(0.1)
        self.expect_fail()
        self.state_changer.transition_to_state(3)
        rospy.sleep(0.5)
        self.publish_raw(0.2)
        self.expect_success()

if __name__ == '__main__':

    rospy.sleep(5.)
    rospy.init_node(NAME, anonymous=True)
    publisher_topics = [("/test/raw_input", "pandora_arm_hardware_interface", "Co2Msg")]
    subscriber_topics = [("/test/alert_ouput", "pandora_common_msgs", "GeneralAlertMsg")]
    Co2ProcessorTest.connect(subscriber_topics, publisher_topics, 1, False)
    rostest.rosrun(PKG, NAME, Co2ProcessorTest, sys.argv)
    Co2ProcessorTest.disconnect()

