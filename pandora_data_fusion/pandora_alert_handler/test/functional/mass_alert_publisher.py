#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_alert_handler')
import rospy
import actionlib
import sys
from  vision_communications.msg import HoleDirectionMsg, HolesDirectionsVectorMsg
from  vision_communications.msg import HazmatAlertMsg, HazmatAlertsVectorMsg
from  vision_communications.msg import QRAlertsVectorMsg, QRAlertMsg
from  pandora_common_msgs.msg import GeneralAlertMsg 

from dynamic_reconfigure.server import Server

from pandora_alert_handler.cfg import MassAlertPublisherConfig

class MassPublisher:
    
    def __init__(self,frame_id):
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg())
        self.hazmat_msg.header.frame_id = frame_id
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.qrAlerts.append(QRAlertMsg())
        self.qr_msg.header.frame_id = frame_id
        
        self.hole_msg = HolesDirectionsVectorMsg()
        self.hole_msg.header.frame_id = frame_id
       
        self.thermal_msg = GeneralAlertMsg()
        self.thermal_msg.header.frame_id = frame_id

        self.face_msg = GeneralAlertMsg()
        self.face_msg.header.frame_id = frame_id

        self.motion_msg = GeneralAlertMsg()
        self.motion_msg.header.frame_id = frame_id

        self.sound_msg = GeneralAlertMsg()
        self.sound_msg.header.frame_id = frame_id

        self.co2_msg = GeneralAlertMsg()
        self.co2_msg.header.frame_id = frame_id

        self.hazmat_pub = rospy.Publisher('/vision/hazmat_alert', HazmatAlertsVectorMsg)
        self.qr_pub = rospy.Publisher('/vision/qr_alert', QRAlertsVectorMsg)
        self.hole_pub = rospy.Publisher('/vision/hole_direction_alert', HolesDirectionsVectorMsg)
        self.thermal_pub = rospy.Publisher('/sensor_processors/thermal_direction_alert', GeneralAlertMsg)
        self.face_pub = rospy.Publisher('/vision/face_direction_alert', GeneralAlertMsg)
        self.motion_pub = rospy.Publisher('/vision/motion_alert', GeneralAlertMsg)
        self.sound_pub = rospy.Publisher('/sensor_processors/sound_direction_alert', GeneralAlertMsg)
        self.co2_pub = rospy.Publisher('/sensor_processors/co2_direction_alert', GeneralAlertMsg)
        self.dyn_reconf_srv = Server(MassAlertPublisherConfig, self.dyn_reconf_callback)

        self.qr_post = False
        self.hazmat_post = False
        self.hole_1_post = False
        self.hole_2_post = False
        self.thermal_post = False
        self.face_post = False
        self.motion_post = False
        self.sound_post = False
        self.co2_post = False
                
        self.publish_stuff()

    def dyn_reconf_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {hole_1_yaw}, {hole_1_pitch}""".format(**config))

        self.hazmat_msg.hazmatAlerts[0]=HazmatAlertMsg(yaw=config.hazmat_yaw,
            pitch=config.hazmat_pitch,patternType=config.hazmat_pattern)
            
        self.qr_msg.qrAlerts[0]=QRAlertMsg(yaw=config.qr_yaw,pitch=config.qr_pitch,
            QRcontent=config.qr_content)
        
        self.hole_msg.holesDirections = []
        
        if config.hole_1_post:
            self.hole_msg.holesDirections.append(HoleDirectionMsg(yaw=config.hole_1_yaw,
                pitch=config.hole_1_pitch,probability=config.hole_1_probability,
                holeId=config.hole_1_id))
            
        if config.hole_2_post:
            self.hole_msg.holesDirections.append(HoleDirectionMsg(yaw=config.hole_2_yaw,
                pitch=config.hole_2_pitch,probability=config.hole_2_probability,
                holeId=config.hole_2_id))
        
        self.thermal_msg.yaw = config.thermal_yaw
        self.thermal_msg.pitch = config.thermal_pitch
        self.thermal_msg.probability = config.thermal_probability
        
        self.face_msg.yaw = config.face_yaw
        self.face_msg.pitch = config.face_pitch
        self.face_msg.probability = config.face_probability
        
        self.motion_msg.yaw = config.motion_yaw
        self.motion_msg.pitch = config.motion_pitch
        self.motion_msg.probability = config.motion_probability
        
        self.sound_msg.yaw = config.sound_yaw
        self.sound_msg.pitch = config.sound_pitch
        self.sound_msg.probability = config.sound_probability
        
        self.co2_msg.yaw = config.co2_yaw
        self.co2_msg.pitch = config.co2_pitch
        self.co2_msg.probability = config.co2_probability
        
        self.qr_post =  config.qr_post
        self.hazmat_post =  config.hazmat_post
        self.hole_1_post =  config.hole_1_post
        self.hole_2_post = config.hole_2_post
        self.thermal_post = config.thermal_post
        self.face_post = config.face_post
        self.motion_post = config.motion_post
        self.sound_post = config.sound_post
        self.co2_post = config.co2_post
                      
        return config
    
    def publish_stuff(self):
        
        while not rospy.is_shutdown():

            if self.hazmat_post:
                self.hazmat_pub.publish(self.hazmat_msg)
                
            if self.qr_post:
                self.qr_pub.publish(self.qr_msg)
                
            if self.hole_1_post or self.hole_2_post:
                self.hole_pub.publish(self.hole_msg)

            if self.thermal_post:
                self.thermal_pub.publish(self.thermal_msg)
            
            if self.face_post:
                self.face_pub.publish(self.face_msg)
            
            if self.motion_post:
                self.motion_pub.publish(self.motion_msg)
            
            if self.sound_post:
                self.sound_pub.publish(self.sound_msg)
            
            if self.co2_post:
                self.co2_pub.publish(self.co2_msg)
            
            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('mass_alert_publisher')
    mass_pub = MassPublisher(sys.argv[1])
