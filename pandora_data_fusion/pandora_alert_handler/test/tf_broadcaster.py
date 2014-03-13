#!/usr/bin/env python
import roslib; roslib.load_manifest('data_fusion')
import rospy
import tf





if __name__ == '__main__':
	
	rospy.init_node('tf_broadcaster')
	
	br = tf.TransformBroadcaster()
	
	
	while True:
	
		br.sendTransform((0, 0, 0.7), 
						tf.transformations.quaternion_from_euler(0, 0, 0), 
						rospy.Time.now(), 
						"headCamera", 
						"world")
		rospy.sleep(0.1)
	
