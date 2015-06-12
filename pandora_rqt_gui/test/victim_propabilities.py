#!/usr/bin/env python

from __future__ import print_function

import random
import rospy
from pandora_data_fusion_msgs.msg import GlobalProbabilitiesMsg

global_propabilities_topic = "/data_fusion/signs_of_life"


def talker():
    msg = GlobalProbabilitiesMsg()
    pub = rospy.Publisher(global_propabilities_topic, GlobalProbabilitiesMsg,
                          queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo(" Publisher for Victim Propabilities initialized")
    while not rospy.is_shutdown():
        msg.thermal = random.randint(0, 5)
        msg.co2 = random.randint(0, 5)
        msg.sound = random.randint(0, 5)
        msg.motion = random.randint(0, 5)
        msg.victim = random.randint(0, 5)
        print(msg)
        pub.publish(msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
