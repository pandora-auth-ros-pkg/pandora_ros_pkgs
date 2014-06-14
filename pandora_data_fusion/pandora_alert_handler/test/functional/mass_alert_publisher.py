#!/usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2014, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
__author__ = "Afouras Triantafyllos and Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import roslib; roslib.load_manifest('pandora_alert_handler')
import rospy
import actionlib
import sys
from vision_communications.msg import HoleDirectionMsg, HolesDirectionsVectorMsg
from vision_communications.msg import HazmatAlertMsg, HazmatAlertsVectorMsg
from vision_communications.msg import QRAlertsVectorMsg, QRAlertMsg
from vision_communications.msg import LandoltcAlertsVectorMsg, LandoltcAlertMsg
from vision_communications.msg import DataMatrixAlertsVectorMsg, DataMatrixAlertMsg
from pandora_common_msgs.msg import GeneralAlertMsg 

from dynamic_reconfigure.server import Server

from pandora_alert_handler.cfg import MassAlertPublisherConfig

class MassPublisher:
    
    def __init__(self,frame_id):
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg())
        self.hazmat_msg.header.frame_id = frame_id
        
        self.landoltc_msg = LandoltcAlertsVectorMsg()
        self.landoltc_msg.landoltcAlerts.append(LandoltcAlertMsg())
        self.landoltc_msg.header.frame_id = frame_id

        self.dataMatrix_msg = DataMatrixAlertsVectorMsg()
        self.dataMatrix_msg.dataMatrixAlerts.append(DataMatrixAlertMsg())
        self.dataMatrix_msg.header.frame_id = frame_id

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
        self.landoltc_pub = rospy.Publisher('/vision/landoltc_alert', LandoltcAlertsVectorMsg)
        self.qr_pub = rospy.Publisher('/vision/qr_alert', QRAlertsVectorMsg)
        self.hole_pub = rospy.Publisher('/vision/holes_direction', HolesDirectionsVectorMsg)
        self.thermal_pub = rospy.Publisher('/sensor_processing/thermal_direction_alert', GeneralAlertMsg)
        self.face_pub = rospy.Publisher('/vision/face_direction_alert', GeneralAlertMsg)
        self.motion_pub = rospy.Publisher('/vision/motion_alert', GeneralAlertMsg)
        self.sound_pub = rospy.Publisher('/sensor_processing/sound_direction_alert', GeneralAlertMsg)
        self.co2_pub = rospy.Publisher('/sensor_processing/co2_direction_alert', GeneralAlertMsg)
        self.dataMatrix_pub = rospy.Publisher('/vision/dataMatrix_alert', DataMatrixAlertsVectorMsg)
        self.dyn_reconf_srv = Server(MassAlertPublisherConfig, self.dyn_reconf_callback)

        self.qr_post = False
        self.hazmat_post = False
        self.landoltc_post = False
        self.dataMatrix_post = False
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
            
        self.landoltc_msg.landoltcAlerts[0]=LandoltcAlertMsg(yaw=config.landoltc_yaw,
            pitch=config.landoltc_pitch,angles=[1, 0, -0.25])
            
        self.dataMatrix_msg.dataMatrixAlerts[0]=DataMatrixAlertMsg(yaw=config.dataMatrix_yaw,
            pitch=config.dataMatrix_pitch,datamatrixContent=config.dataMatrix_content)
            
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
        self.landoltc_post =  config.landoltc_post
        self.dataMatrix_post =  config.dataMatrix_post
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
                
            if self.landoltc_post:
                self.landoltc_pub.publish(self.landoltc_msg)
                
            if self.dataMatrix_post:
                self.dataMatrix_pub.publish(self.dataMatrix_msg)
                
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
