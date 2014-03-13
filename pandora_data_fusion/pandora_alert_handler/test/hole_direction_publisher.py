#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import actionlib
import sys
from  vision_communications.msg import  HoleDirectionMsg, HolesDirectionsVectorMsg


if __name__ == '__main__':
	
	rospy.init_node('hole_direction_publisher')
	
	pub = rospy.Publisher(sys.argv[1], HolesDirectionsVectorMsg)
	
	data = HoleDirectionMsg()
	
	data.yaw = float(sys.argv[2])
	data.pitch = float(sys.argv[3])
	data.probability = float(sys.argv[4])
	data.holeId = float(sys.argv[5])
	
	msg = HolesDirectionsVectorMsg()
	
	msg.header.frame_id = 'headCamera'
	
	msg.header.stamp = rospy.get_rostime()
	
	msg.holesDirections.append(data)
		
	for i in range(2):
		 
		pub.publish(msg)
		rospy.sleep(1)
