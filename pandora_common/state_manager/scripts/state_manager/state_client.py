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
# Authors: Chris Zalidis <zalidis@gmail.com>
#          Konstantinos Sideris <siderisk@auth.gr>

import rospy
from rospy import Publisher, Subscriber, ServiceProxy, loginfo, logerr
from actionlib import SimpleActionClient as ActionClient
from state_manager_msgs.msg import RobotModeMsg, RobotModeAction, RobotModeGoal
from state_manager_msgs.srv import RegisterNodeSrv, RegisterNodeSrvRequest
from state_manager_msgs.srv import GetStateInfo, GetStateInfoRequest


class StateClient(object):

    def __init__(self, do_register=True, silent=True):
        self._name = rospy.get_name()
        self._silent = silent
        self._acknowledge_publisher = Publisher('/robot/state/server',
                                                RobotModeMsg, latch=True,
                                                queue_size=5)
        self._state_subscriber = Subscriber('/robot/state/clients',
                                            RobotModeMsg,
                                            self.server_state_information,
                                            queue_size=10)
        self._state_info = ServiceProxy('/robot/state/info', GetStateInfo)
        self._state_changer = ActionClient('/robot/state/change',
                                           RobotModeAction)
        if do_register:
            self.client_register()

    def client_register(self):
        rospy.wait_for_service('/robot/state/register')
        self._register_service_client = rospy.ServiceProxy('/robot/state/register', RegisterNodeSrv)

        req = RegisterNodeSrvRequest()
        req.nodeName = self._name
        req.type = RegisterNodeSrvRequest.TYPE_STARTED
        try:
            self._register_service_client(req)
        except rospy.ServiceException:
            logerr("[%s] Failed to register node. Retrying...", self._name)

    def client_initialize(self):
        rospy.wait_for_service('/robot/state/register')
        self._register_service_client = rospy.ServiceProxy('/robot/state/register', RegisterNodeSrv)

        req = RegisterNodeSrvRequest()
        req.nodeName = self._name
        req.type = RegisterNodeSrvRequest.TYPE_INITIALIZED
        try:
            self._register_service_client(req)
        except rospy.ServiceException:
            logerr("[%s] Failed to register node. Retrying...", self._name)

    def start_transition(self, state):
        if not self._silent:
            loginfo("[%s] Starting Transition to state %i", self._name, state)
        self.transition_complete(state)

    def transition_complete(self, state):
        if not self._silent:
            loginfo("[%s] Node Transition to state %i Completed", self._name, state)
        msg = RobotModeMsg()
        msg.nodeName = self._name
        msg.mode = state
        msg.type = RobotModeMsg.TYPE_ACK

        self._acknowledge_publisher.publish(msg)

    def get_current_state(self):
        """ Returns the current robot state from the state manager. """

        req = GetStateInfoRequest()
        req.option = GetStateInfoRequest.CURRENT_STATE
        try:
            res = self._state_info(req)
            if res.state == -1:

                # There is a bug with the server.
                logerr('GetStateInfo: %s is not a valid request.', req.option)
            return res.state
        except rospy.ServiceException:
            logerr('Service GetStateInfo failed to respond.')

        return None

    def get_previous_state(self):
        """ Returns the previous robot state from the state manager. """

        req = GetStateInfoRequest()
        req.option = GetStateInfoRequest.PREVIOUS_STATE
        try:
            res = self._state_info(req)
            if res.state == -1:

                # There is a bug with the server.
                logerr('GetStateInfo: %s is not a valid request.', req.option)
            return res.state
        except rospy.ServiceException:
            logerr('Service GetStateInfo failed to respond.')

        return None

    def transition_to_state(self, state):
        if not self._silent:
            loginfo("[%s] Requesting transition to state %i", self._name, state)
        msg = RobotModeMsg()
        msg.nodeName = self._name
        msg.mode = state
        msg.type = RobotModeMsg.TYPE_REQUEST

        self._acknowledge_publisher.publish(msg)

    def change_state_and_wait(self, state, timeout=None):
        """ Changes the state and waits for all the clients to change.

        :param :state A state to transition to.
        :param :timeout A timeout in seconds to complete the state transition.
        """
        timeout = rospy.Duration(timeout) if timeout else rospy.Duration()
        msg = RobotModeMsg()
        msg.mode = state
        msg.nodeName = self._name
        msg.type = RobotModeMsg.TYPE_REQUEST
        goal = RobotModeGoal(modeMsg=msg)

        self._state_changer.wait_for_server()
        self._state_changer.send_goal(goal)

        return self._state_changer.wait_for_result(timeout=timeout)

    def server_state_information(self, msg):
        if not self._silent:
            loginfo("[%s] Received new information from state server", self._name)

        if msg.type == RobotModeMsg.TYPE_TRANSITION:
            self.start_transition(msg.mode)
        elif msg.type == RobotModeMsg.TYPE_START:
            if not self._silent:
                loginfo("[%s] System Transitioned, starting work", self._name)
        else:
            logerr("[%s] StateClient received a new state command, that is not understandable", self._name)
