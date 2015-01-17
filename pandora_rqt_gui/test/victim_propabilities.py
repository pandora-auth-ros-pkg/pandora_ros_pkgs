#!/usr/bin/env python
import rospy

from pandora_data_fusion_msgs.msg import GlobalProbabilitiesMsg

# float32 thermal
# float32 co2
# float32 sound
# float32 motion
# float32 face

global_propabilities_topic = "/data_fusion/victim_fusion/global_probabilities"


def talker():
    msg = GlobalProbabilitiesMsg()
    msg.thermal = 1
    msg.co2 = 2
    msg.sound = 3
    msg.motion = 4
    msg.face = 5
    pub = rospy.Publisher(global_propabilities_topic, GlobalProbabilitiesMsg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo(" Publisher for Victim Propabilities initialized")
    r = rospy.Rate(1)  # 1 hz
    while not rospy.is_shutdown():

        msg.thermal = 1 + msg.thermal
        msg.co2 = 2 + msg.thermal
        msg.sound = 3 + msg.thermal
        msg.motion = 4 + msg.thermal
        msg.face = 5 + msg.thermal
        pub.publish(msg)

        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
