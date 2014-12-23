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
import alsaaudio
import time
import struct
import numpy
from pandora_audio_msgs.msg import AudioData


class KinectAudioCapture():

    def __init__(self):
        self.sample_rate = rospy.get_param("sample_rate")
        self.window_size = rospy.get_param("window_size")
        self.bit_depth = rospy.get_param("bit_depth")
        self.capture_channels = rospy.get_param("capture_channels")
        self.buffer_size = rospy.get_param("buffer_size")
        self.record_buffers = rospy.get_param("record_buffers")

        pub = rospy.Publisher(rospy.get_param("published_topic_names/audio_stream"), AudioData, queue_size=10)
        inp = self.get_audio_input()
        while not rospy.is_shutdown():
            wb = self.get_raw_data(inp)
            pub.publish(wb[0], wb[1], wb[2], wb[3])

    def get_audio_input(self):
        card = 'sysdefault:CARD=Audio'
        audio_input = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NORMAL, card)
        audio_input.setchannels(self.capture_channels )
        audio_input.setrate(self.sample_rate )
        if self.bit_depth == 16:
            audio_input.setformat(alsaaudio.PCM_FORMAT_S16_LE)
        else:
            raise Exception("Not supported bit depth")
        audio_input.setperiodsize(self.buffer_size)
        return audio_input

    def get_raw_data(self, inp):
        raw_data = []
        for k in range(0, self.record_buffers):
            length, data = inp.read()
            raw_data.append(data)
            time.sleep(.001)

        window_buffers = [[], [], [], []]
        for data in raw_data:
            all_channels_buffer = struct.unpack("<" + str(self.capture_channels * self.buffer_size) + 'h', data)
            for i in range(0, self.capture_channels):
                window_buffers[i].append(all_channels_buffer[i::self.capture_channels])

        for i in range(0, self.capture_channels):
            window_buffers[i] = numpy.hstack(window_buffers[i])
        return window_buffers

if __name__ == '__main__':
    rospy.init_node('kinect_capture', anonymous=True)
    try:
        kinect_audio_capture = KinectAudioCapture()
    except rospy.ROSInterruptException: pass
