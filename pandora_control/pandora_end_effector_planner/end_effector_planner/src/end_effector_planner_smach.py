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
roslib.load_manifest('pandora_end_effector_planner')
roslib.load_manifest('smach')
roslib.load_manifest('smach_ros')
import rospy
import threading
import smach_ros

from smach import StateMachine, Concurrence
from states import EndEffectorPlannerState, KinectOrientationState, \
    HeadOrientationState, LinearMovementState
from pandora_end_effector_planner.msg import MoveEndEffectorAction, \
    MoveEndEffectorGoal
from topics import move_end_effector_planner_topic


def main():

    rospy.init_node('end_effector_planner_smach')

    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'],
                      input_keys=['move_end_effector_msg'])

    with sm:

        sm.userdata.lower_linear = \
            MoveEndEffectorGoal(command=MoveEndEffectorGoal.PARK)

        StateMachine.add(
            'END_EFFECTOR_PLANNER',
            EndEffectorPlannerState(),
            transitions={
                'TEST_PARK_TRACK': 'TEST_PARK_TRACK',
                'SCAN': 'LOWER_LINEAR'
            },
            remapping={'move_end_effector_msg': 'move_end_effector_msg'}
        )

        test_park_track_cc = Concurrence(
            outcomes=[
                'succeeded',
                'aborted',
                'preempted'
            ],
            default_outcome='preempted',
            input_keys=['move_end_effector_msg'],
            output_keys=['move_end_effector_msg'],
            outcome_map={
                'succeeded': {'KINECT_ORIENTATION': 'succeeded',
                              'HEAD_ORIENTATION': 'succeeded',
                              'LINEAR_MOVEMENT': 'succeeded'},
                'aborted': {'LINEAR_MOVEMENT': 'aborted'},
                'preempted': {'KINECT_ORIENTATION': 'preempted',
                              'HEAD_ORIENTATION': 'preempted',
                              'LINEAR_MOVEMENT': 'preempted'}
            },
            child_termination_cb=termination_cb
        )

        with test_park_track_cc:
            Concurrence.add('KINECT_ORIENTATION', KinectOrientationState(),
                            remapping={'move_end_effector_msg':
                                       'move_end_effector_msg'})
            Concurrence.add('HEAD_ORIENTATION', HeadOrientationState(),
                            remapping={'move_end_effector_msg':
                                       'move_end_effector_msg'})
            Concurrence.add('LINEAR_MOVEMENT', LinearMovementState(),
                            remapping={'move_end_effector_msg':
                                       'move_end_effector_msg'})

        StateMachine.add(
            'TEST_PARK_TRACK',
            test_park_track_cc,
            transitions={
                'succeeded': 'succeeded',
                'aborted': 'aborted',
                'preempted': 'preempted'
            },
            remapping={'move_end_effector_msg': 'move_end_effector_msg'}
        )

        StateMachine.add(
            'LOWER_LINEAR',
            LinearMovementState(),
            transitions={
                'succeeded': 'KINECT_ORIENTATION',
                'aborted': 'aborted',
                'preempted': 'preempted'
            },
            remapping={'move_end_effector_msg': 'lower_linear'}
        )

        StateMachine.add(
            'KINECT_ORIENTATION',
            KinectOrientationState(),
            transitions={
                'preempted': 'preempted'
            },
            remapping={'move_end_effector_msg': 'move_end_effector_msg'}
        )

    asw = smach_ros.ActionServerWrapper(
        move_end_effector_planner_topic,
        MoveEndEffectorAction,
        wrapped_container=sm,
        succeeded_outcomes=['succeeded'],
        aborted_outcomes=['aborted'],
        preempted_outcomes=['preempted'],
        goal_key='move_end_effector_msg'
    )

    sis = smach_ros.IntrospectionServer('eef_fsm_introspection', sm, '/EEF_FSM')
    sis.start()

    smach_thread = threading.Thread(target = asw.run_server)
    smach_thread.start()

    rospy.spin()


def termination_cb(outcome_map):
    if outcome_map['LINEAR_MOVEMENT'] == 'aborted':
        return True
    else:
        return False

if __name__ == '__main__':
    main()
