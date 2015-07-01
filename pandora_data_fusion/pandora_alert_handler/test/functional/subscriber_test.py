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
NAME = 'subscriber_test'

import os
import sys
import math

import unittest

import roslib
roslib.load_manifest(PKG)
import rostest
import rospy

from pandora_testing_tools.testing_interface import alert_delivery
from test_base import distance
from test_base import direction
import test_base

DIR = os.path.dirname(os.path.realpath(__file__))


class SubscriberTest(test_base.TestBase):

    def test_new_subscriber(self):

        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        self.deliveryBoy.deliverHoleOrder(0, 0, 1)
        outs = []
        self.fillInfo(outs)

        # Benchmark new subscriber style...
        self.assertEqual(len(outs[0].holes), 1)

if __name__ == '__main__':

    rospy.sleep(2)
    rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
    SubscriberTest.connect()
    rostest.rosrun(PKG, NAME, SubscriberTest, sys.argv)
    SubscriberTest.disconnect()
