#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import Int32

def dummy_callback(data):
    rospy.Publisher("/test/answer", Int32).publish(Int32(4))
    return

if __name__ == "__main__":
    rospy.init_node("pluto", sys.argv)
    subscriber = rospy.Subscriber("/test/listen", Int32, dummy_callback)
    rospy.spin()
