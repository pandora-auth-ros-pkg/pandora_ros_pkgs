#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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

import roslib; roslib.load_manifest('pandora_teleop')
import rospy
import sys
import thread
from time import sleep
from geometry_msgs.msg import Twist

class MotorsJoyop:

  linear = 0
  angular = 0
  pub = rospy.Publisher("cmd_vel", Twist)

  def __init__(self,linear_coeff,angular_coeff):
    thread.start_new_thread(self.setSpeeds, (linear_coeff,angular_coeff))
    self.readJoystick()

  def readJoystick(self):
    # Open the js0 device as if it were a file in read mode.
    pipe = open('/dev/input/js0', 'r')

    # Create an empty list to store read characters.
    msg = []

    # Loop forever.
    while 1:

      # For each character read from the /dev/input/js0 pipe...
      for char in pipe.read(1):

        # append the integer representation of the unicode character read to the msg list.
        msg += [ord(char)]

        # If the length of the msg list is 8...
        if len(msg) == 8:

          # Button event if 6th byte is 1
          if msg[6] == 1:
            if msg[4]  == 1:
                print 'button', msg[7], 'down'
            else:
                print 'button', msg[7], 'up'
          # Axis event if 6th byte is 2
          elif msg[6] == 2:
            # Axis 3
            if msg[7] == 2:
              if int(msg[5]) > 127:
                self.linear = float(abs(255 - int(msg[5])))/128
              else:
                self.linear = float(0 - int(msg[5]))/128
            if msg[7] == 3:
              if int(msg[5]) > 127:
                self.angular = float(abs(255 - int(msg[5])))/128
              else:
                self.angular = float(0 - int(msg[5]))/128
          # Reset msg as an empty list.
          msg = []


  def setSpeeds(self,linear_coeff,angular_coeff):

    while True:
      print '--> linear: ', self.linear, ' , angular: ', self.angular


      #~ while not rospy.is_shutdown():
      msg = Twist()
      msg.linear.x = self.linear*linear_coeff
      msg.angular.z = self.angular*angular_coeff
      self.pub.publish(msg)
      rospy.sleep(0.2)


if __name__ == '__main__':
  rospy.init_node('set_vehicle_speed_wrapper')
  joy = MotorsJoyop(linear_coeff=0.55, angular_coeff=1.2)
