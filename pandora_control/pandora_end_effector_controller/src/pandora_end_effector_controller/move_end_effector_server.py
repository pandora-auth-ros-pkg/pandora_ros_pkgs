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

from pandora_end_effector_controller.msg import MoveEndEffectorAction, MoveEndEffectorGoal
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal
from pandora_linear_movement_controller.msg import MoveLinearAction, MoveLinearGoal
from topics import move_end_effector_controller_topic, move_kinect_topic, \
    move_head_topic, move_linear_topic
from client_factory import ClientFactory
from client_list import CLIENTS

class MoveEndEffectorServer(object):

  def __init__(self):
    self.factory = ClientFactory()
    self.current_clients = list()
    self.current_goal = MoveEndEffectorGoal()


    self.server = Server(move_end_effector_controller_topic, 
                        MoveEndEffectorAction, self.execute_cb,
                        False)
    self.server.register_preempt_callback(self.preempt_cb)
    self.server.start()
    self.create_clients()
    self.wait_for_servers()

  def create_clients(self):
    ''' Imports all clients and creates a list of them '''

    for client in CLIENTS:
      self.current_clients.append(self.factory.make_client(client))

  def wait_for_servers(self):
    ''' Waits for every client to connect with their server '''

    for client in self.current_clients:
      client.wait_server()

  def execute_cb(self, goal):
    ''' Callback triggered by the arrival of a goal '''

    self.current_goal = goal
    self.fill_goals()
    self.send_goals()
    self.wait_for_result()
    self.checkGoalState()

  def preempt_cb(self):
    ''' Preempting all goals '''

    for client in self.current_clients:
      client.preempt_if_active()

  def fill_goals(self):
    ''' Filling goals into every client respectively '''
    
    for client in self.current_clients:
      client.fill_goal(self.current_goal)

  def send_goals(self):
    ''' Sending goals to every client respectively '''

    for client in self.current_clients:
      client.send_goal()

  def wait_for_result(self):

    for client in self.current_clients:
      client.wait_result()

  def success(self):

    self.server.set_succeeded()

  def abort(self):

    self.server.set_aborted()

  def preempt(self):

    self.server.set_preempted()

  def check_succeeded(self):
    ''' Checks if the final state of the goal must be set succeeded '''

    must_succeed = True

    for client in self.current_clients:
      must_succeed = must_succeed and client.has_succeeded()

    return must_succeed

  def check_aborted(self):
    ''' Checks if the final state of the goal must be set aborted '''

    must_abort = False

    for  client in self.current_clients:
      must_abort = must_abort or client.has_aborted()

    return must_abort

  def check_preempted(self):
    ''' Checks if the final state of the goal must be set preempted '''

    must_preempt = False

    for  client in self.current_clients:
      must_preempt = must_preempt or client.has_preempted()

    return must_preempt

  def check_recalled(self):
    ''' Checks if the final state of the goal must be set preempted '''

    must_recall = False

    for  client in self.current_clients:
      must_recall = must_recall or client.has_been_recalled()

    return must_recall

  def checkGoalState(self):
    ''' Checking final state of goal '''

    if(self.check_succeeded()):
      self.success()
    elif(self.check_aborted()):
      self.abort()
    elif(self.check_recalled()):
      for client in self.current_clients:
        rospy.loginfo(str(client.client.get_state()))
      self.abort()
    elif(self.check_preempted()):
      self.preempt()
    else:
      for client in self.current_clients:
        rospy.loginfo(str(client.client.get_state()))
      rospy.loginfo('Unexpected State')
      self.abort()