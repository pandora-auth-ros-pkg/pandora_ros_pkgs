#!/usr/bin/env python
import sys

import rospy
from std_msgs.msg import Int32


def main():
    rospy.init_node("dummy_node", sys.argv, log_level=rospy.INFO)
    rospy.loginfo("Yolo")
    publisher = rospy.Publisher("/test/answer", Int32)
    def dummy_callback(data):
        rospy.loginfo("Dummy node got : "+str(data.data))
        publisher.publish(Int32(4))
        return
    subscriber = rospy.Subscriber("/test/listen", Int32, dummy_callback)
    rospy.spin()


if __name__ == "__main__":
    main()
