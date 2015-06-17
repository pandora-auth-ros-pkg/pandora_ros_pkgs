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
# Author: Voulgarakis George

import roslib
roslib.load_manifest('pandora_end_effector_controller')
import rospy

from smach import State
from smach_ros import SimpleActionState
from pandora_end_effector_controller.msg import MoveEndEffectorAction, MoveEndEffectorGoal
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal
from pandora_linear_movement_controller.msg import MoveLinearAction, MoveLinearGoal
from topics import move_end_effector_controller_topic, move_kinect_topic, \
    move_head_topic, linear_movement_topic


class EndEffectorcontrollerState(State):

    def __init__(self):
        State.__init__(self, outcomes=['TEST_PARK_TRACK', 'SCAN'],
                       input_keys=['move_end_effector_msg'],
                       output_keys=['move_end_effector_msg'])

    def execute(self, userdata):
        if userdata.move_end_effector_msg.command == \
                MoveEndEffectorGoal.SCAN:
            return 'SCAN'
        else:
            return 'TEST_PARK_TRACK'


class KinectOrientationState(SimpleActionState):

    def __init__(self):
        SimpleActionState.__init__(self, move_kinect_topic,
                                   MoveSensorAction,
                                   goal_cb=self.goal_cb,
                                   outcomes=['succeeded',
                                             'aborted',
                                             'preempted'],
                                   input_keys=['move_end_effector_msg'],
                                   output_keys=['move_end_effector_msg'])

    def goal_cb(self, userdata, goal):
        goal = MoveSensorGoal()
        goal.command = userdata.move_end_effector_msg.command
        goal.point_of_interest = \
            userdata.move_end_effector_msg.point_of_interest
        return goal


# class HeadOrientationState(SimpleActionState):

#     def __init__(self):
#         SimpleActionState.__init__(self, move_head_topic,
#                                    MoveSensorAction,
#                                    goal_cb=self.goal_cb,
#                                    outcomes=['succeeded',
#                                              'aborted',
#                                              'preempted'],
#                                    input_keys=['move_end_effector_msg'],
#                                    output_keys=['move_end_effector_msg'])

#     def goal_cb(self, userdata, goal):
#         goal = MoveSensorGoal()
#         goal.command = userdata.move_end_effector_msg.command
#         goal.point_of_interest = \
#             userdata.move_end_effector_msg.point_of_interest
#         return goal


# class LinearMovementState(SimpleActionState):

#     def __init__(self):
#         SimpleActionState.__init__(self, linear_movement_topic,
#                                    MoveLinearAction,
#                                    goal_cb=self.goal_cb,
#                                    outcomes=['succeeded',
#                                              'aborted',
#                                              'preempted'],
#                                    input_keys=['move_end_effector_msg'],
#                                    output_keys=['move_end_effector_msg'])

    # def goal_cb(self, userdata, goal):
    #     goal = MoveLinearGoal()
    #     goal.command = userdata.move_end_effector_msg.command
    #     goal.point_of_interest = \
    #         userdata.move_end_effector_msg.point_of_interest
    #     goal.center_point = userdata.move_end_effector_msg.center_point
    #     return goal
