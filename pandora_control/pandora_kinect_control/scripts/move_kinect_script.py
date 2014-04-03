#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
from pandora_kinect_control.msg import *

if __name__ == '__main__':
  rospy.init_node('move_kinect_client_py')
  client = actionlib.SimpleActionClient('move_kinect_action', MoveKinectAction)
  client.wait_for_server()
  goal = MoveKinectGoal()
  goal.command = int(sys.argv[1])
  client.send_goal(goal)
