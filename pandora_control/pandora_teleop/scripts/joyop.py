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


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from numpy import clip
from subprocess import call
from sys import argv

class Joyop:

  def __init__(self, motors_lin_vel_scale=0.5, motors_ang_vel_scale=0.8):
    self.motors_lin_vel_scale = motors_lin_vel_scale
    self.motors_ang_vel_scale = motors_ang_vel_scale
    self.lac_scale = 14.0
    self.xtion_yaw_range = [-0.7, 0.7]
    self.xtion_pitch_range = [-0.45, 0.75]
    self.picam_yaw_range = [-0.7, 0.7]
    self.picam_pitch_range = [-0.6, 0.75]

    self.motors_lin_vel = 0
    self.motors_ang_vel = 0
    self.angular_ang_vel = 0
    self.lac_position = 0
    self.xtion_yaw = 0
    self.xtion_pitch = 0
    self.picam_yaw = 0
    self.picam_pitch = 0

    self.motors_vel_pub = rospy.Publisher('/cmd_vel', Twist)
    self.lac_position_pub = rospy.Publisher('/linear_actuator/command', Float64)
    self.xtion_yaw_pub = rospy.Publisher('/kinect_yaw_controller/command', Float64)
    self.xtion_pitch_pub = rospy.Publisher('/kinect_pitch_controller/command', Float64)
    self.picam_yaw_pub = rospy.Publisher('/camera_effector/pan_command', Float64)
    self.picam_pitch_pub = rospy.Publisher('/camera_effector/tilt_command', Float64)
    joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

    rospy.Timer(rospy.Duration(0.1), self.launch_joy_node, oneshot=True)
    rospy.sleep(rospy.Duration(3))
    rospy.Timer(rospy.Duration(0.1), self.pub_callback, oneshot=False)
    rospy.loginfo("\n\n\n\n")
    rospy.spin()

  def launch_joy_node(self, event):
    call(["rosrun", "joy", "joy_node"])  # launch joystick node

  def print_state(self):
    rospy.loginfo("\x1b[1M\x1b[5A\x1b[5M")  # erase previous prints
    rospy.loginfo("\033[33;1m[motors] lin_vel: %0.1f - ang_vel: %0.1f\033[0m", self.motors_lin_vel, self.motors_ang_vel)
    rospy.loginfo("\033[33;1m[linear_actuator] position: %0.1f\033[0m", self.lac_position)
    rospy.loginfo("\033[33;1m[xtion] yaw: %0.2f - pitch: %0.2f\033[0m", self.xtion_yaw, self.xtion_pitch)
    rospy.loginfo("\033[33;1m[picam] yaw: %0.2f - pitch: %0.2f\033[0m", self.picam_yaw, self.picam_pitch)

  def joy_callback(self, joy_msg):
    self.motors_lin_vel = joy_msg.axes[2] * self.motors_lin_vel_scale
    self.motors_ang_vel = joy_msg.axes[3] * self.motors_ang_vel_scale

    if joy_msg.buttons[0] == 1:
      self.lac_position = self.lac_scale * joy_msg.axes[1] * (joy_msg.axes[1] > 0)
    if joy_msg.buttons[1] == 1:
      self.xtion_yaw = clip(joy_msg.axes[0], self.xtion_yaw_range[0], self.xtion_yaw_range[1])
      self.xtion_pitch = clip(joy_msg.axes[1], self.xtion_pitch_range[0], self.xtion_pitch_range[1])
    if joy_msg.buttons[2] == 1:
      self.picam_yaw = clip(joy_msg.axes[0], self.picam_yaw_range[0], self.picam_yaw_range[1])
      self.picam_pitch = clip(joy_msg.axes[1], self.picam_pitch_range[0], self.picam_pitch_range[1])


  def pub_callback(self, event):
    motors_vel_msg = Twist()
    lac_position_msg = Float64()
    xtion_yaw_msg = Float64()
    xtion_pitch_msg = Float64()
    picam_yaw_msg = Float64()
    picam_pitch_msg = Float64()

    motors_vel_msg.linear.x = self.motors_lin_vel
    motors_vel_msg.angular.z = self.motors_ang_vel
    lac_position_msg.data = self.lac_position
    xtion_yaw_msg.data = self.xtion_yaw
    xtion_pitch_msg.data = self.xtion_pitch
    picam_yaw_msg.data = self.picam_yaw
    picam_pitch_msg.data = self.picam_pitch

    self.motors_vel_pub.publish(motors_vel_msg)
    self.lac_position_pub.publish(lac_position_msg)
    self.xtion_yaw_pub.publish(xtion_yaw_msg)
    self.xtion_pitch_pub.publish(xtion_pitch_msg)
    self.picam_yaw_pub.publish(picam_yaw_msg)
    self.picam_pitch_pub.publish(picam_pitch_msg)

    self.print_state()


if __name__ == "__main__":
  rospy.init_node('joyop_node')
  rospy.loginfo("Joyop Teleoperation Node Initialized")
  rospy.loginfo("RS[motors], 1+LS[lac], 2+LS[xtion], 3+LS[picam]")

  args = argv[1:]
  if len(args) == 1:
    rospy.loginfo("Setting max linear velocity to %s and angular velocity to %s", args[0], args[1])
    joyop = Joyop(args[0], args[1])
  else:
    rospy.loginfo("Using default max linear(0.5m/s) and angular velocity(0.8m/s)")
    joyop = Joyop()

