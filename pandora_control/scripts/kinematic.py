#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_control')
import rospy
import actionlib
import sys
import serial
from time import sleep
from geometry_msgs.msg import Twist

class Kinematic:
  
  pub = None
  serial_left = serial.Serial('/dev/ttyUSB0', 9600) 
  serial_right = serial.Serial('/dev/ttyUSB1', 9600) 
  wheel_separation = 0.4392
  wheel_diameter = 0.0889
  rpm_max = 100
  
  def __init__(self):
    pub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
  
  def twist_callback(self, msg):
    
    wheel_speed_left = msg.linear.x - msg.angular.z * self.wheel_separation / 2.0
    wheel_speed_right = msg.linear.x + msg.angular.z * self.wheel_separation / 2.0
    
    rps_left = wheel_speed_left/self.wheel_diameter
    rps_right = wheel_speed_right/self.wheel_diameter
    
    rpm_left  = rps_left/(self.rpm_max*2*3.14/60) 
    rpm_right  = rps_right/(self.rpm_max*2*3.14/60)
    
    print int(rpm_left*255), int(rpm_right*255)
    self.serial_left.write(str(int(rpm_left*255)))
    self.serial_right.write(str(int(rpm_left*255)))


if __name__ == '__main__':
  rospy.init_node('kinematic')
  joy = Kinematic()
  rospy.spin()
