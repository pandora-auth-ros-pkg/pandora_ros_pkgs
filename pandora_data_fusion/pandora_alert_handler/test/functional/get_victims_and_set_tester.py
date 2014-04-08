#!/usr/bin/env python
import roslib; 
roslib.load_manifest('data_fusion')
import rospy
import actionlib
from data_fusion_communications.msg import *

if __name__ == '__main__':
    rospy.init_node('verify_victim_test')
    get_victims_client = actionlib.SimpleActionClient('/data_fusion/get_victims', GetVictimsAction)
 
    stuck_goal = GetVictimsGoal()
    get_victims_client.wait_for_server()
    print "Sending get victims goal"
    get_victims_client.send_goal(stuck_goal)
    get_victims_client.wait_for_result()
    print get_victims_client.get_goal_status_text()
    print "Got result. Victims = " + str( get_victims_client.get_result() )
	
    rospy.spin()
	
