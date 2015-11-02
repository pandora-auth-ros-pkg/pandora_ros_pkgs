#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, P.A.N.D.O.R.A. Team.
# All rights reserved.
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
# Author: Peppas Kostas

import rospy
from effector_clients import SensorClient, LinearActuatorClient, HeadClient


class ClientFactory(object):

    def __init__(self):
        """ Factory initialized """
        self._name = "/PANDORA_CONTROL/PANDORA_END_EFFECTOR_CONTROLLER"

        rospy.loginfo("[" + self._name + "] Initializing factory...")
        self.return_client = {
            'sensor_client': self.return_sensor,
            'linear_actuator_client': self.return_linear_actuator,
            'head_client': self.return_head
        }

    def make_client(self, client_name):
        """ Making appropriate client """
        rospy.loginfo("[" + self._name + "] Making client... " + client_name)
        return self.return_client[client_name]()

    def return_sensor(self):
        """ Returning SensorClient instance """
        return SensorClient()

    def return_linear_actuator(self):
        """ Returning LinearActuatorClient instance """
        return LinearActuatorClient()

    def return_head(self):
        """ Returning HeadClient instance """
        return HeadClient()
