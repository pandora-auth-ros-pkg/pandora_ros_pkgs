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
from rospy import loginfo, logwarn, sleep, Subscriber
import roslib
roslib.load_manifest('pandora_end_effector_controller')
from std_msgs.msg import String, Bool

from actionlib import SimpleActionServer as ActionServer
from pandora_end_effector_controller.topics import move_end_effector_controller_topic, move_kinect_topic, \
    move_head_topic, move_linear_actuator_topic
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal
from pandora_linear_actuator_controller.msg import MoveLinearActuatorAction, MoveLinearActuatorGoal
from pandora_end_effector_controller.msg import MoveEndEffectorAction, MoveEndEffectorGoal


class MockActionServer(object):

    """ MockActionServer base class """

    def __init__(self, name, topic, action_type):
        """ Creating a custom mock action server."""

        self._topic = topic
        self._name = name
        self._action_type = action_type
        self.timeout = 1
        self.action_result = None
        self.prefered_callback = ' '

        Subscriber('mock/' + name, String, self.receive_commands)
        Subscriber('mock/gui_result', Bool, self.set_gui_result)
        self._server = ActionServer(self._topic, self._action_type,
                                    self.decide, False)
        self._server.start()
        loginfo('>>> Starting ' + self._name)

    def receive_commands(self, msg):
        """ Decides the result of the next call. """

        callback, timeout = msg.data.split(':')
        # self.timeout = float(timeout)
        self.prefered_callback = callback
        sleep(1)

    def decide(self, goal):
        """ Deciding outcome of goal """

        logwarn('Deciding callback...')
        sleep(self.timeout)
        if(self.prefered_callback == 'success'):
            logwarn('>>> ' + self._name + ': This goal will succeed.')
            self._server.set_succeeded()
        elif(self.prefered_callback == 'abort'):
            logwarn('>>> ' + self._name + ': This goal will be aborted.')
            self._server.set_aborted()
        elif(self.prefered_callback == 'preempt'):
            logwarn('>>> ' + self._name + ': This goal will be preempted.')
            self._server.set_preempted()
        else:
            logwarn('wtf?')

    def set_gui_result(self, msg):
        """ Sets the result of the goal. """

        self.action_result = ValidateVictimGUIResult()
        logwarn('>>> The gui response will be: ' + str(msg.data))
        self.action_result.victimValid = msg.data

if __name__ == '__main__':
    rospy.init_node('mock_node')

    # Action Servers
    MockActionServer('linear_actuator', move_linear_actuator_topic,
                     MoveLinearActuatorAction)
    MockActionServer('sensor', move_kinect_topic,
                     MoveSensorAction)
    MockActionServer('head', move_head_topic,
                     MoveSensorAction)

    rospy.spin()
