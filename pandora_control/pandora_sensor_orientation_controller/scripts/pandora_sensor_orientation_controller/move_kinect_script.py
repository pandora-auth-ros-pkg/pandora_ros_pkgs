#! /usr/bin/env python

import roslib
import rospy
import sys
import actionlib
from pandora_sensor_orientation_controller.msg import MoveSensorAction, MoveSensorGoal

if __name__ == '__main__':
    rospy.init_node('move_kinect_client_py')
    client = actionlib.SimpleActionClient('control/move_kinect_action', MoveSensorAction)
    client.wait_for_server()
    goal = MoveSensorGoal()
    goal.command = int(sys.argv[1])
    if goal.command == 2 or goal.command == 3:
        goal.point_of_interest = sys.argv[2]
    client.send_goal(goal)
