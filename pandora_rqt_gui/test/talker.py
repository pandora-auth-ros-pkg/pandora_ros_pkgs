#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
from pandora_arm_hardware_interface.msg import Co2Msg
from pandora_xmega_hardware_interface.msg import BatteryMsg
from std_msgs.msg import Int32
from pandora_data_fusion_msgs.msg import WorldModelMsg
from pandora_data_fusion_msgs.msg import VictimInfoMsg


battery_topic = "sensors/battery"
sonars_topic = "sensors/range"
world_model_info_topic = 'data_fusion/world_model_info'
robocup_score_topic = 'data_fusion/robocup_score'
co2_topic ="sensors/co2"

def talker():

    i = 4

    pub = rospy.Publisher('chatter',Int16 , queue_size=10)
    pub_batteries = rospy.Publisher(battery_topic, BatteryMsg ,queue_size=10)
    pub_sonars = rospy.Publisher(sonars_topic, Range,queue_size=10)
    pub_co2 = rospy.Publisher(co2_topic, Co2Msg,queue_size=10)
    pub_score = rospy.Publisher(robocup_score_topic, Int32 ,queue_size=10)
    pub_victims = rospy.Publisher(world_model_info_topic ,WorldModelMsg ,queue_size=10)

    rospy.init_node('talker', anonymous = False)
    r = rospy.Rate(0.5) # 2 hz

    batteries_msg = BatteryMsg()
    batteries_msg.voltage.append(float((random.randrange(19,39)))/4)
    batteries_msg.voltage.append((random.randrange(19,39))/4)

    sonars_msg_left = Range()
    sonars_msg_right = Range()
    sonars_msg_left.header.frame_id = "left"
    sonars_msg_right.header.frame_id = "right"
    
    co2_msg = Co2Msg()
    
    victim1 = VictimInfoMsg()
    victim2 = VictimInfoMsg()
    victim3 = VictimInfoMsg()
    
    victim1.valid =True
    victim2.valid =False
    victim3.valid =True
    victimList = [victim1 ,victim2,victim3]
    
    world_msg = WorldModelMsg()
   
    

    while not rospy.is_shutdown():

        rospy.loginfo("%d",i)

        batteries_msg.voltage[0]=(float((random.randrange(0,20))))/4 +19
        batteries_msg.voltage[1]=(float((random.randrange(0,20))))/4 +19

        sonars_msg_left.range = (float((random.randrange(0,20))))/4
        sonars_msg_right.range = (float((random.randrange(0,100))))/4
        
        co2_msg.co2_percentage = float(i)/1000
        
        world_msg.visitedVictims.append(random.choice(victimList))

        pub.publish(i)
        pub_score.publish(i/3)
        pub_batteries.publish(batteries_msg)
        pub_sonars.publish(sonars_msg_left)
        pub_co2.publish(co2_msg)
        pub_victims.publish(world_msg)
        r.sleep()
        pub_sonars.publish(sonars_msg_right)

        i = i+4
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
