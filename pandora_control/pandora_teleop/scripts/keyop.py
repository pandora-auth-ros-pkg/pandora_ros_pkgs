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
# Author: George Kouros


import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty
import thread
from numpy import clip
from sys import argv

control_keys = {
  'up' : '\x41',
  'down' : '\x42',
  'right' : '\x43',
  'left' : '\x44',
  'space' : '\x20'}
control_bindings = {
  '\x41' : ( 0.1 , 0.0),
  '\x42' : (-0.1 , 0.0),
  '\x43' : ( 0.0 ,-0.1),
  '\x44' : ( 0.0 , 0.1),
  '\x20' : ( 0.0 , 0,0)}

mode_keys = {
  'k' : '\6B',  # kinect - xtion pan n' tilt mode key
  'l' : '\6C',  # linear actuator mode key
  'm' : '\6D',  # motors mode key
  'p' : '\70'}  # pi-cam pan n' tilt mode key
modes = {
  'k' : 'xtion',
  'l' : 'lac',
  'm' : 'motors',
  'p' : 'picam'}


class Keyop:

  def __init__(self, max_lin_vel=0.5, max_ang_vel=0.8):
    self.lin_vel_range = [-float(max_lin_vel), float(max_lin_vel)]
    self.ang_vel_range = [-float(max_ang_vel), float(max_ang_vel)]
    self.lac_range = [0.0, 0.14]
    self.xtion_yaw_range = [-0.7, 0.7]
    self.xtion_pitch_range = [-0.45, 0.75]
    self.picam_yaw_range = [-0.7, 0.7]
    self.picam_pitch_range = [-0.6, 0.75]

    self.lin_vel = 0       # motors linear velocity
    self.ang_vel = 0       # motors angular velocity
    self.lac_position = 0  # linear actuator vertical position
    self.xtion_yaw = 0     # xtion yaw
    self.xtion_pitch = 0   # xtion pitch
    self.picam_yaw = 0     # picam yaw
    self.picam_pitch = 0   # picam pitch

    self.motors_pub = rospy.Publisher('cmd_vel', Twist)
    self.lac_pub = rospy.Publisher('linear_actuator/command', Float64)
    self.xtion_yaw_pub = rospy.Publisher('kinect_yaw_controller/command', Float64)
    self.xtion_pitch_pub = rospy.Publisher('kinect_pitch_controller/command', Float64)
    self.picam_yaw_pub = rospy.Publisher('camera_effector/pan_command', Float64)
    self.picam_pitch_pub = rospy.Publisher('camera_effector/tilt_command', Float64)

    self.mode = 'motors'  # initialize mode to motors mode

    rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
    self.key_loop()


  def pub_callback(self, event):
   motors_msg = Twist()         # motors velocity msg
   lac_msg = Float64()          # linear actuator msg
   xtion_yaw_msg = Float64()    # xtion yaw msg
   xtion_pitch_msg = Float64()  # xtion pitch msg
   picam_yaw_msg = Float64()    # picam yaw msg
   picam_pitch_msg = Float64()  # picam pitch msg

   if self.mode == 'motors':
     motors_msg.linear.x = self.lin_vel
     motors_msg.angular.z = self.ang_vel
     self.motors_pub.publish(motors_msg)
   elif self.mode == 'lac':
     lac_msg.data = self.lac_position
     self.lac_pub.publish(lac_msg)
   elif self.mode == 'xtion':
     xtion_yaw_msg.data = self.xtion_yaw
     xtion_pitch_msg.data = self.xtion_pitch
     self.xtion_yaw_pub.publish(xtion_yaw_msg)
     self.xtion_pitch_pub.publish(xtion_pitch_msg)
   elif self.mode == 'picam':
     picam_yaw_msg.data = self.picam_yaw
     picam_pitch_msg.data = self.picam_pitch
     self.picam_yaw_pub.publish(picam_yaw_msg)
     self.picam_pitch_pub.publish(picam_pitch_msg)

  def print_state(self, erase):
    if erase:
      rospy.loginfo("\x1b[3A")

    rospy.loginfo("\033[32;1mTeleop Mode: %s   \033[0m", self.mode)

    rospy.loginfo("\x1b[1M\x1b[1A")

    if self.mode == 'motors':
      rospy.loginfo("\033[33;1mMotors: linear_speed: %0.1f - angular_speed: %0.1f\033[0m", self.lin_vel, self.ang_vel)
    elif self.mode == 'lac':
      rospy.loginfo("\033[33;1mLinear Actuator: Position: %0.2f\033[0m", self.lac_position)
    elif self.mode == 'xtion':
      rospy.loginfo("\033[33;1mXtion: pitch: %0.2f - yaw: %0.2f\033[0m", self.xtion_pitch, self.xtion_yaw)
    elif self.mode == 'picam':
      rospy.loginfo("\033[33;1mPicam: pitch: %0.2f - yaw: %0.2f\033[0m", self.picam_pitch, self.picam_yaw)

  def get_key(self):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    return key


  def key_loop(self):
    rospy.loginfo("Use the arrow keys to adjust speed or position and space to halt/reset")
    rospy.loginfo("Press 'ctr-c' or 'q' to exit")
    self.print_state(0)
    self.settings = termios.tcgetattr(sys.stdin)

    while 1:
      key = self.get_key()
      if key in control_bindings.keys():
        if self.mode == 'motors':
          if key == control_keys['space']:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
          else:
            self.lin_vel = self.lin_vel + control_bindings[key][0]
            self.ang_vel = self.ang_vel + control_bindings[key][1]
          self.lin_vel = clip(self.lin_vel, self.lin_vel_range[0], self.lin_vel_range[1])
          self.ang_vel = clip(self.ang_vel, self.ang_vel_range[0], self.ang_vel_range[1])
        elif self.mode == 'lac':
          if key == control_keys['space']:
            self.lac_position = 0
          else:
            self.lac_position = self.lac_position + control_bindings[key][0] / 10
          self.lac_position = clip(self.lac_position, self.lac_range[0], self.lac_range[1])
        elif self.mode == 'xtion':
          if key == control_keys['space']:
            self.xtion_pitch = 0
            self.xtion_yaw = 0
          else:
            self.xtion_pitch = self.xtion_pitch + control_bindings[key][0] / 2
            self.xtion_yaw = self.xtion_yaw + control_bindings[key][1] / 2
          self.xtion_pitch = clip(self.xtion_pitch, self.xtion_yaw_range[0], self.xtion_yaw_range[1])
          self.xtion_yaw = clip(self.xtion_yaw, self.xtion_pitch_range[0], self.xtion_pitch_range[1])
        elif self.mode == "picam":
          if key == control_keys['space']:
            self.picam_pitch = 0
            self.picam_yaw = 0
          else:
            self.picam_pitch = self.picam_pitch + control_bindings[key][0] / 2
            self.picam_yaw = self.picam_yaw + control_bindings[key][1] / 2
          self.picam_yaw = clip(self.picam_yaw, self.picam_yaw_range[0], self.picam_yaw_range[1])
          self.picam_pitch = clip(self.picam_pitch, self.picam_pitch_range[0], self.picam_pitch_range[1])

        self.print_state(1)

      elif key in mode_keys:
        self.lin_vel = 0; self.ang_vel = 0
        motors_msg = Twist()
        motors_msg.linear.x = 0; motors_msg.angular.z = 0
        self.motors_pub.publish(motors_msg)
        self.mode = modes[key]
        self.print_state(1)
        continue
      elif key == '\x03' or key == '\x71':  # ctr-c or q
        break
      else:
        continue

    self.finalize()

  def finalize(self):
    rospy.loginfo('Halting motors and exiting...')
    self.settings = termios.tcgetattr(sys.stdin)
    self.mode = 'quit'
    motors_msg = Twist(); motors_msg.linear.x = 0; motors_msg.angular.z = 0
    self.motors_pub.publish(motors_msg)
    sys.exit()

if __name__ == "__main__":
  rospy.init_node('keyop_node')
  rospy.loginfo("Keyboard Teleoperation Node Initialized")
  rospy.loginfo("Change mode with m(motors), k(xtion), l(lac), p(picam)")
  args = argv[1:]
  if len(args) == 2:
    rospy.loginfo("Setting motors max linear velocity to %s and angular velocity to %s", args[0], args[1])
    keyop = Keyop(args[0], args[1])
  else:
    rospy.loginfo("Using default motors max linear(0.5m/s) and angular velocity(0.8m/s)")
    keyop = Keyop()
