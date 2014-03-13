#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
import sys
from  vision_communications.msg import  HazmatAlertMsg, HazmatAlertsVectorMsg


if __name__ == '__main__':
	
	rospy.init_node('hazmat_alert_publisher')
	
	pub = rospy.Publisher('/vision/hazmat_alert', HazmatAlertsVectorMsg)
	
	data = HazmatAlertMsg()
	
	#~ data.yaw = 0
	#~ data.pitch = 0
	
	data.yaw = float(sys.argv[1])
	data.pitch = float(sys.argv[2])
	
	data.patternType = int(sys.argv[3])
	
	msg = HazmatAlertsVectorMsg()
	
	msg.header.frame_id = 'headCamera'
	
	msg.header.stamp = rospy.get_rostime()
	
	msg.hazmatAlerts.append(data)
		
	for i in range(2):
		 
		pub.publish(msg)
		rospy.sleep(1)
