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

import roslib
roslib.load_manifest('pandora_end_effector_controller')
import rospy

from actionlib import GoalStatus
from actionlib import SimpleActionServer as Server
from actionlib import SimpleActionClient as Client
from client_dict import sensor_client, linear_actuator_client, head_client
from command_mapping_dict import translate_command


class SensorClient(object):
    """docstring for SensorClient"""

    def __init__(self):
        self._name = "SensorClient"
        self.goal = sensor_client['goal']
        self.topic = sensor_client['topic']
        self.action = sensor_client['action']
        self.command_dict = translate_command['to_sensor']

        self.client = Client(self.topic, self.action)

    def get_name(self):
        return self._name

    def fill_goal(self, effector_goal):
        self.goal.command = self.command_dict[effector_goal.command]
        self.goal.point_of_interest = effector_goal.point_of_interest

    def send_goal(self):
        self.client.send_goal(self.goal)

    def wait_server(self):
        self.client.wait_for_server()

    def wait_result(self):
        self.client.wait_for_result()

    def has_succeeded(self):
        return self.client.get_state() == GoalStatus.SUCCEEDED

    def has_aborted(self):
        return self.client.get_state() == GoalStatus.ABORTED

    def has_preempted(self):
        return self.client.get_state() == GoalStatus.PREEMPTED

    def has_been_recalled(self):
        return self.client.get_state() == GoalStatus.RECALLED

    def preempt_if_active(self):
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_all_goals()


class LinearActuatorClient(object):
    """docstring for LinearActuatorClient"""

    def __init__(self):
        self._name = "LinearActuatorClient"
        self.goal = linear_actuator_client['goal']
        self.topic = linear_actuator_client['topic']
        self.action = linear_actuator_client['action']
        self.command_dict = translate_command['to_linear_actuator']

        self.client = Client(self.topic, self.action)

    def get_name(self):
        return self._name

    def fill_goal(self, effector_goal):
        self.goal.command = self.command_dict[effector_goal.command]
        self.goal.point_of_interest = effector_goal.point_of_interest
        self.goal.center_point = effector_goal.center_point

    def send_goal(self):
        self.client.send_goal(self.goal)

    def wait_server(self):
        self.client.wait_for_server()

    def wait_result(self):
        self.client.wait_for_result()

    def has_succeeded(self):
        return self.client.get_state() == GoalStatus.SUCCEEDED

    def has_aborted(self):
        return self.client.get_state() == GoalStatus.ABORTED

    def has_preempted(self):
        return self.client.get_state() == GoalStatus.PREEMPTED

    def has_been_recalled(self):
        return self.client.get_state() == GoalStatus.RECALLED

    def preempt_if_active(self):
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_all_goals()


class HeadClient(object):
    """docstring for HeadClient"""

    def __init__(self):
        self._name = "HeadClient"
        self.goal = head_client['goal']
        self.topic = head_client['topic']
        self.action = head_client['action']
        self.command_dict = translate_command['to_head']

        self.client = Client(self.topic, self.action)

    def get_name(self):
        return self._name

    def fill_goal(self, effector_goal):
        self.goal.command = self.command_dict[effector_goal.command]
        self.goal.point_of_interest = effector_goal.point_of_interest

    def send_goal(self):
        self.client.send_goal(self.goal)

    def wait_result(self):
        self.client.wait_for_result()

    def wait_server(self):
        self.client.wait_for_server()

    def has_succeeded(self):
        return self.client.get_state() == GoalStatus.SUCCEEDED

    def has_aborted(self):
        return self.client.get_state() == GoalStatus.ABORTED

    def has_preempted(self):
        return self.client.get_state() == GoalStatus.PREEMPTED

    def has_been_recalled(self):
        return self.client.get_state() == GoalStatus.RECALLED

    def preempt_if_active(self):
        if self.client.get_state() == GoalStatus.ACTIVE:
            self.client.cancel_all_goals()
