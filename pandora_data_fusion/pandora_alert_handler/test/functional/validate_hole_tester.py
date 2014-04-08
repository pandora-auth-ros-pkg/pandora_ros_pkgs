#!/usr/bin/env python
import roslib; 
roslib.load_manifest('data_fusion')
import rospy
import actionlib
from data_fusion_communications.msg import *

if __name__ == '__main__':
    rospy.init_node('delete_current_victim_test')
    get_victims_client = actionlib.SimpleActionClient('/data_fusion/validate_current_hole', ValidateCurrentHoleAction)
 
    stuck_goal = ValidateCurrentHoleGoal(valid=True)
    get_victims_client.wait_for_server()
    print "Sending ValidateCurrentHoleGoal goal"
    get_victims_client.send_goal(stuck_goal)
    get_victims_client.wait_for_result()
    print "Got result"	
    print get_victims_client.get_goal_status_text()
    rospy.spin()
	
