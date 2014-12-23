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
# Author: Nikolaos Tsipas

import rospy
from pandora_audio_msgs.msg import AudioData
from pandora_common_msgs.msg import GeneralAlertMsg
from state_manager_msgs.msg import RobotModeMsg
import std_msgs.msg
import math
import numpy as np
import state_manager


class KinectAudioProcessing(state_manager.state_client.StateClient):

    def __init__(self):

        state_manager.state_client.StateClient.__init__(self)

        self.window_size = rospy.get_param("window_size")
        self.noise_floor_buffer_length = rospy.get_param("noise_floor_buffer_length") # 64 windows (~8 seconds)
        self.source_loc_buffer_length = rospy.get_param("source_loc_buffer_length") # 16 windows (~2 second)
        self.similarity_distance = rospy.get_param("similarity_distance")  # similarity distance in radians (~30 degrees)
        self.source_loc_buffer = []
        self.noise_floor_buffer = []

        self.robot_state = RobotModeMsg.MODE_OFF

        self.pub = rospy.Publisher(rospy.get_param("published_topic_names/sound_source_localisation"), GeneralAlertMsg,
                                   queue_size=10)
        self.sub = rospy.Subscriber(rospy.get_param("subscribed_topic_names/audio_stream"), AudioData, self.callback,
                                    queue_size=1)
        self.sub.unregister()

        self.client_initialize()

    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self.robot_state = state
        self.transition_complete(state)

    def complete_transition(self):
        rospy.loginfo("[%s] System Transitioned, starting work", self._name)
        if self.robot_state == RobotModeMsg.MODE_SENSOR_HOLD or self.robot_state == RobotModeMsg.MODE_SENSOR_TEST:
            self.sub.__init__(rospy.get_param("subscribed_topic_names/audio_stream"), AudioData, self.callback,
                              queue_size=1)
        else:
            self.sub.unregister()

    def rms(self, window_data):
        return math.sqrt((window_data ** 2).sum() / float(len(window_data)))

    def calculate_horizontal_angle(self, rms_c1, rms_c2, rms_c3, rms_c4):

        if rms_c1 < np.median(self.noise_floor_buffer):
            return 999

        dif13 = rms_c1 - rms_c3
        dif24 = rms_c2 - rms_c4

        angle = 999

        if dif13 > 0 and dif24 > 0:
            angle = math.atan2(dif13, dif24)
        elif dif13 > 0 and dif24 < 0:
            angle = math.pi/2 + math.atan2(-dif24, dif13)
        elif dif13 < 0 and dif24 < 0:
            angle = math.pi + math.atan2(-dif13, -dif24)
        elif dif13 < 0 and dif24 > 0:
            angle = 1.5 * math.pi + math.atan2(dif24, -dif13)

        return angle


    def analyse_buffered_data(self, bam):
        similar = 0
        for i in range(0, len(bam)-1):  # excluding the last element which is the most recent
            if abs(bam[-1] - bam[i]) <= self.similarity_distance:
                similar += 1

        if len(bam) <= 1:
            return [bam[-1], 1]

        probability = similar / float(len(bam)-1)
        return [bam[-1], probability]


    def callback(self, data):

        rms_c1 = self.rms(np.array(data.channel1))
        rms_c2 = self.rms(np.array(data.channel2))
        rms_c3 = self.rms(np.array(data.channel3))
        rms_c4 = self.rms(np.array(data.channel4))
        self.noise_floor_buffer.append(rms_c1)
        if len(self.noise_floor_buffer) > self.noise_floor_buffer_length:
            self.noise_floor_buffer.pop(0)

        self.source_loc_buffer.append(self.calculate_horizontal_angle(rms_c1, rms_c2, rms_c3, rms_c4))
        if len(self.source_loc_buffer) > self.source_loc_buffer_length:
            self.source_loc_buffer.pop(0)
        angle, probability = self.analyse_buffered_data(self.source_loc_buffer)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        self.pub.publish(h, angle, 0, probability)


if __name__ == '__main__':
    rospy.init_node('kinect_sound_source_localisation', anonymous=True)
    try:
        kinect_audio_processing = KinectAudioProcessing()
    except rospy.ROSInterruptException: pass
    rospy.spin()

