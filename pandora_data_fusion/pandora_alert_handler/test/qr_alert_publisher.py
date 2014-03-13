#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
import sys
from  vision_communications.msg import  QRAlertsVectorMsg, QRAlertMsg


if __name__ == '__main__':
	
	rospy.init_node('qr_alert_publisher')
	
	pub = rospy.Publisher('/vision/qr_alert', QRAlertsVectorMsg)
	
	data = QRAlertMsg()
	
	#~ data.yaw = 0
	#~ data.pitch = 0
	data.yaw = float(sys.argv[1])
	data.pitch = float(sys.argv[2])
	
	data.QRcontent = 'just_testing'
	
	msg = QRAlertsVectorMsg()
	
	msg.header.frame_id = 'headCamera'
	
	msg.header.stamp = rospy.get_rostime()
	
	msg.qrAlerts.append(data)
		
	for i in range(2):
		 
		pub.publish(msg)
		rospy.sleep(1)
