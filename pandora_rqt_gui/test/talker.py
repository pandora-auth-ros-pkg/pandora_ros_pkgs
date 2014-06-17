#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

def talker():
    i = 4
    pub = rospy.Publisher('chatter',Int16 , queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
       
        rospy.loginfo("%d",i)
        
        pub.publish(i)
        i = i+4
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
