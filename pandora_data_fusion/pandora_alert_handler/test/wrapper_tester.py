#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
from  slam_communications.msg import *


if __name__ == '__main__':
	rospy.init_node('wrapper_tester')
	
	testerClient = actionlib.SimpleActionClient('/slam/slamMap', slamMapAction)
	
	print 'Waiting for server'
	testerClient.wait_for_server()
	
	
	
	while 1:
		
		print 'entered loop'
		
		goal = slamMapActionGoal()
	
		testerClient.send_goal(goal)

		print 'waiting for result'
		testerClient.wait_for_result()
		
		print 'received ' + str(testerClient.get_result())
		
		rospy.sleep(2)
	

