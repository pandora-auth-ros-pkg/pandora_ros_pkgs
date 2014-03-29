#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_control')
import rospy
import actionlib
import sys
import serial
from geometry_msgs.msg import Twist


class Kinematic:
  
  pub = None
  serial_port = None
  wheel_separation = 0.4392
  wheel_diameter = 0.0889
  rpm_max = 100
  
  def __init__(self):
    pub = rospy.Subscriber("cmd_vel", Twist, self.twist_callback)
    serial_port = serial.Serial('/dev/ttyUSB0', 9600)
  
  def twist_callback(self, msg):
    
    wheel_speed_left = msg.linear.x - msg.angular.z * self.wheel_separation / 2.0
    wheel_speed_right = msg.linear.x + msg.angular.z * self.wheel_separation / 2.0
    
    rps_left = wheel_speed_left/self.wheel_diameter
    rps_right = wheel_speed_right/self.wheel_diameter
    
    rpm_left  = rps_left/(self.rpm_max*2*3.14/60) 
    rpm_right  = rps_right/(self.rpm_max*2*3.14/60)
    
    command = "$L" + '{:+04d}'.format(int(rpm_left*255)) + "R" + '{:+04d}'.format(int(rpm_right*255))
    self.serial_port.write(command)
  
  
if __name__ == '__main__':
  rospy.init_node('kinematic')
  kinematic = Kinematic()
  rospy.spin()




  
  
