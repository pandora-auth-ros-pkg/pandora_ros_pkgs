#!/usr/bin/env python

import roslib; roslib.load_manifest('pandora_control')
import rospy
import actionlib
import sys
import thread
from time import sleep
from geometry_msgs.msg import Twist

class Joystick:

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
            if msg[7] == 3:
              if int(msg[5]) > 127:
                self.linear = float(abs(255 - int(msg[5])))/128
              else:
                self.linear = float(0 - int(msg[5]))/128
            if msg[7] == 2:
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
  joy = Joystick(linear_coeff=0.6, angular_coeff=0.8)
