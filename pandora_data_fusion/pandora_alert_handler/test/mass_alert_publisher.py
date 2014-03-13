#!/usr/bin/env python
import roslib; roslib.load_manifest('pandora_alert_handler')
import rospy
import actionlib
import sys
from  vision_communications.msg import  HoleDirectionMsg, HolesDirectionsVectorMsg
from  vision_communications.msg import  HazmatAlertMsg, HazmatAlertsVectorMsg
from  vision_communications.msg import  QRAlertsVectorMsg, QRAlertMsg

from dynamic_reconfigure.server import Server

from pandora_alert_handler.cfg import MassAlertPublisherConfig

#~ def dyn_reconf_callback():
    #~ pass

class MassPublisher:
    

    
    def __init__(self,frame_id):
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg())
        self.hazmat_msg.header.frame_id = frame_id
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.qrAlerts.append(QRAlertMsg())
        self.qr_msg.header.frame_id = frame_id
        
        self.hole_1_msg = HolesDirectionsVectorMsg()
        #~ self.hole_1_msg.holesDirections.append(HoleDirectionMsg())
        #~ self.hole_1_msg.holesDirections.append(HoleDirectionMsg())
        self.hole_1_msg.header.frame_id = frame_id
        
        #~ self.hole_2_msg = HolesDirectionsVectorMsg()
        #~ self.hole_2_msg.holesDirections.append(HoleDirectionMsg())
        #~ self.hole_2_msg.header.frame_id = 'headCamera'
        
        self.hazmat_pub = rospy.Publisher('/vision/hazmat_alert', HazmatAlertsVectorMsg)
        self.qr_pub = rospy.Publisher('/vision/qr_alert', QRAlertsVectorMsg)
        self.hole_pub = rospy.Publisher('/data_fusion/victim_fusion/hole_direction_alert', HolesDirectionsVectorMsg)
        self.dyn_reconf_srv = Server(MassAlertPublisherConfig, self.dyn_reconf_callback)

        self.qr_post = False
        self.hazmat_post = False
        self.hole_1_post = False
        self.hole_2_post = False
                
        self.publish_stuff()
    


    def dyn_reconf_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {hole_1_yaw}, {hole_1_pitch}""".format(**config))

        self.hazmat_msg.hazmatAlerts[0]=HazmatAlertMsg(yaw=config.hazmat_yaw,
            pitch=config.hazmat_pitch,patternType=config.hazmat_pattern)
            
        self.qr_msg.qrAlerts[0]=QRAlertMsg(yaw=config.qr_yaw,pitch=config.qr_pitch,
            QRcontent=config.qr_content)
        
        
        self.hole_1_msg.holesDirections = []
        
        if config.hole_1_post:
            self.hole_1_msg.holesDirections.append(HoleDirectionMsg(yaw=config.hole_1_yaw,
                pitch=config.hole_1_pitch,probability=1,holeId=config.hole_1_id))
            
        if config.hole_2_post:
            self.hole_1_msg.holesDirections.append(HoleDirectionMsg(yaw=config.hole_2_yaw,
                pitch=config.hole_2_pitch,probability=1,holeId=config.hole_2_id))
        
        
        self.qr_post =  config.qr_post
        self.hazmat_post =  config.hazmat_post
        self.hole_1_post =  config.hole_1_post
        self.hole_2_post = config.hole_2_post
                      
        return config
    
    def publish_stuff(self):
        
        while not rospy.is_shutdown():

            if self.hazmat_post:
                self.hazmat_pub.publish(self.hazmat_msg)
                
            if self.qr_post:
                self.qr_pub.publish(self.qr_msg)
                
            if self.hole_1_post or self.hole_2_post:
                self.hole_pub.publish(self.hole_1_msg)
            
            rospy.sleep(0.5)


            
    
if __name__ == '__main__':
    rospy.init_node('mass_alert_publisher')
    mass_pub = MassPublisher(sys.argv[1])
