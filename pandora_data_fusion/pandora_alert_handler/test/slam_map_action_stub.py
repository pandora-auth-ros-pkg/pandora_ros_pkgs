#~ actionStub class listens to map message at slamMapActionResult topic and provides
#~ action returning the last map received


#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
from  slam_communications.msg import *


class actionStub:
	
	def __init__(self):
		
		rospy.Subscriber("/slam/slamMapMsg", slamMapActionResult, self.mapMessageCallback)
		
		self._actionStubServer = actionlib.SimpleActionServer('/slam/slamMap', slamMapAction, self.executeCb, False)
		self._actionStubServer.start()
		
		self._latestMap = None
		self._sleepRate = rospy.Rate(100)
		

	def executeCb(self,goal):
		while not self._latestMap:
			self._sleepRate.sleep()
		self._actionStubServer.set_succeeded(self._latestMap.result)

		
	def mapMessageCallback(self,result):
		self._latestMap = result



if __name__ == '__main__':
	rospy.init_node('action_stub')
	try:
		actionStub()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

