#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
from pandora_end_effector_planner.msg import *

if __name__ == '__main__':
  rospy.init_node('move_kinect_client_py')
  client = actionlib.SimpleActionClient('move_kinect_action', MoveSensorAction)
  client.wait_for_server()
  goal = MoveSensorGoal()
  goal.command = int(sys.argv[1])
  if goal.command == 2:
    goal.point_of_interest = sys.argv[2]
  client.send_goal(goal)
