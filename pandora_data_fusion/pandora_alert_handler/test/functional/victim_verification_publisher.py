#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
import sys
from  data_fusion_communications.msg import  VictimVerificationMsg


if __name__ == '__main__':
	
	rospy.init_node('hazmat_alert_publisher')
	
	pub = rospy.Publisher('/data_fusion/victim_verification', VictimVerificationMsg)
	
	msg = VictimVerificationMsg(sensorIds = [0,4],probability = 1)
			
	for i in range(2):
		 
		pub.publish(msg)
		rospy.sleep(1)
