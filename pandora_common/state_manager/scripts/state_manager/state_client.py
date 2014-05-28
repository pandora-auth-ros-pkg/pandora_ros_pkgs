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
# Author: Chris Zalidis <zalidis@gmail.com>

import rospy
from state_manager_communications.msg import robotModeMsg
from state_manager_communications.srv import registerNodeSrv, registerNodeSrvRequest

class StateClient(object):

    def __init__(self, do_register=True):
        self._name = rospy.get_name()
        self._acknowledge_publisher = rospy.Publisher('/robot/state/server', robotModeMsg, latch=True, queue_size=5)
        self._state_subscriber = rospy.Subscriber('/robot/state/clients', robotModeMsg, self.server_state_information, queue_size=10)
        if do_register:
            self.client_register()
  
    def client_register(self):
        rospy.wait_for_service('/robot/state/register')
        self._register_service_client = rospy.ServiceProxy('/robot/state/register', registerNodeSrv)
    
        req = registerNodeSrvRequest();
        req.nodeName = self._name
        req.type = registerNodeSrvRequest.TYPE_STARTED
        try:
            self._register_service_client(req)
        except rospy.ServiceException:
            rospy.logerr("[%s] Failed to register node. Retrying...", self._name)
  
    def client_initialize(self):
        rospy.wait_for_service('/robot/state/register')
        self._register_service_client = rospy.ServiceProxy('/robot/state/register', registerNodeSrv)
    
        req = registerNodeSrvRequest();
        req.nodeName = self._name
        req.type = registerNodeSrvRequest.TYPE_INITIALIZED
        try:
            self._register_service_client(req)
        except rospy.ServiceException:
            rospy.logerr("[%s] Failed to register node. Retrying...", self._name)
  
    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self.transition_complete(state)
  
    def transition_complete(self, state):
        rospy.loginfo("[%s] Node Transition to state %i Completed", self._name, state)
        msg = robotModeMsg()
        msg.nodeName = self._name
        msg.mode = state
        msg.type = robotModeMsg.TYPE_ACK
    
        self._acknowledge_publisher.publish(msg)
  
    def complete_transition(self):
        rospy.loginfo("[%s] System Transitioned, starting work", self._name)
  
    def transition_to_state(self, state):
        rospy.loginfo("[%s] Requesting transition to state %i", self._name, state)
        msg = robotModeMsg()
        msg.nodeName = self._name
        msg.mode = state
        msg.type = robotModeMsg.TYPE_REQUEST
    
        self._acknowledge_publisher.publish(msg)
  
    def server_state_information(self, msg):
        rospy.loginfo("[%s] Received new information from state server", self._name)
  
        if msg.type == robotModeMsg.TYPE_TRANSITION:
            self.start_transition(msg.mode)
        elif msg.type == robotModeMsg.TYPE_START:
            self.complete_transition()
        else:
            rospy.logerr("[%s] StateClient received a new state command, that is not understandable", self._name)
  
