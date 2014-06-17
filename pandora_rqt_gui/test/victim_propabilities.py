#!/usr/bin/env python
import rospy

from data_fusion_communications.msg import FusionGlobalMsg

#~ float32 mlx
#~ float32 co2
#~ float32 sound
#~ float32 motion
#~ float32 face

def talker():
    msg = FusionGlobalMsg()
    msg.mlx = 1
    msg.co2 = 2
    msg.sound = 3
    msg.motion = 4
    msg.face = 5
    pub = rospy.Publisher("/data_fusion/victim_fusion/global_probabilities",FusionGlobalMsg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo(" Publisher for Victim Propabilities initialized")
    r = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
       
        msg.mlx = 1 +msg.mlx
        msg.co2 = 2 +  msg.mlx
        msg.sound = 3  + msg.mlx
        msg.motion = 4  + msg.mlx
        msg.face = 5 + msg.mlx
        pub.publish(msg)
       
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
