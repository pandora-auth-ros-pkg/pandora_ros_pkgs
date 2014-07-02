#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
from pandora_end_effector_planner.msg import *

if __name__ == '__main__':
  rospy.init_node('move_eef_client_py')
  client = actionlib.SimpleActionClient('control/move_end_effector_planner_action', MoveEndEffectorAction)
  client.wait_for_server()
  goal = MoveEndEffectorGoal()
  goal.command = int(sys.argv[1])
  if goal.command == 2 or goal.command == 3:
    goal.point_of_interest = sys.argv[2]
    goal.center_point = sys.argv[3]
  client.send_goal(goal)
