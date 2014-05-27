#!usr/bin/env python
import os
from collections import deque

import roslib; roslib.load_manifest('pandora_alert_handler')
import rospy

from vision_communications.msg import HoleDirectionMsg
from vision_communications.msg import HolesDirectionsVectorMsg
from vision_communications.msg import HazmatAlertMsg
from vision_communications.msg import HazmatAlertsVectorMsg
from vision_communications.msg import QRAlertsVectorMsg
from vision_communications.msg import QRAlertMsg
from vision_communications.msg import LandoltcAlertsVectorMsg
from vision_communications.msg import LandoltcAlertMsg
from vision_communications.msg import DataMatrixAlertsVectorMsg
from vision_communications.msg import DataMatrixAlertMsg
from pandora_common_msgs.msg import GeneralAlertMsg

class BadBossOrderFile(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class AlertDeliveryBoy:

    def __init__(self):

        self.frame_id = 'headCamera'

        self.orderWaitingList = deque([])
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.header.frame_id = self.frame_id
        
        self.landoltc_msg = LandoltcAlertsVectorMsg()
        self.landoltc_msg.header.frame_id = self.frame_id
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.header.frame_id = self.frame_id
        
        self.dataMatrix_msg = DataMatrixAlertsVectorMsg()
        self.dataMatrix_msg.header.frame_id = self.frame_id
        
        self.hole_msg = HolesDirectionsVectorMsg()
        self.hole_msg.header.frame_id = self.frame_id

        self.general_msg = GeneralAlertMsg()
        self.general_msg.header.frame_id = self.frame_id
        
        self.hazmatDeliveryAddress = '/vision/hazmat_alert'
        self.hazmat_pub = rospy.Publisher(self.hazmatDeliveryAddress, 
                                          HazmatAlertsVectorMsg)
        self.landoltcDeliveryAddress = '/vision/landoltc_alert'
        self.landoltc_pub = rospy.Publisher(self.landoltcDeliveryAddress, 
                                          LandoltcAlertsVectorMsg)
        self.qrDeliveryAddress = '/vision/qr_alert'
        self.qr_pub = rospy.Publisher(self.qrDeliveryAddress, 
                                      QRAlertsVectorMsg)
        self.holeDeliveryAddress = '/vision/hole_direction_alert'
        self.hole_pub = rospy.Publisher(self.holeDeliveryAddress, 
                                        HolesDirectionsVectorMsg)
        self.dataMatrixDeliveryAddress = '/vision/dataMatrix_alert'
        self.dataMatrix_pub = rospy.Publisher(self.dataMatrixDeliveryAddress, 
                                      DataMatrixAlertsVectorMsg)
        self.thermalDeliveryAddress = '/sensor_processors/thermal_direction_alert'
        self.thermal_pub = rospy.Publisher(self.thermalDeliveryAddress,
                                       GeneralAlertMsg)
        self.faceDeliveryAddress = '/vision/face_direction_alert'
        self.face_pub = rospy.Publisher(self.faceDeliveryAddress,
                                       GeneralAlertMsg)
        self.soundDeliveryAddress = '/sensor_processors/sound_direction_alert'
        self.sound_pub = rospy.Publisher(self.soundDeliveryAddress,
                                       GeneralAlertMsg)
        self.motionDeliveryAddress = '/vision/motion_alert'
        self.motion_pub = rospy.Publisher(self.motionDeliveryAddress,
                                       GeneralAlertMsg)
        self.co2DeliveryAddress = '/sensor_processors/co2_direction_alert'
        self.co2_pub = rospy.Publisher(self.co2DeliveryAddress,
                                       GeneralAlertMsg)

    def deliverHoleOrder(self, orderYaw, 
                        orderPitch, orderId, orderProbability = 1):

        self.hole_msg.header.stamp = rospy.get_rostime()
        self.hole_msg.holesDirections = []
        self.hole_msg.holesDirections.append(HoleDirectionMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            probability = orderProbability,
                                            holeId = orderId))
        self.hole_pub.publish(self.hole_msg)
        rospy.sleep(0.1)

    def deliverHazmatOrder(self, orderYaw, 
                          orderPitch, orderPattern):

        self.hazmat_msg.header.stamp = rospy.get_rostime()
        self.hazmat_msg.hazmatAlerts = []
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            patternType = orderPattern))
        self.hazmat_pub.publish(self.hazmat_msg)
        rospy.sleep(0.1)

    def deliverLandoltcOrder(self, orderYaw, 
                          orderPitch, orderAngles):

        self.landoltc_msg.header.stamp = rospy.get_rostime()
        self.landoltc_msg.landoltcAlerts = []
        self.landoltc_msg.landoltcAlerts.append(LandoltcAlertMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            angles = orderAngles))
        self.landoltc_pub.publish(self.landoltc_msg)
        rospy.sleep(0.1)

    def deliverQrOrder(self, orderYaw, 
                      orderPitch, orderContent):

        self.qr_msg.header.stamp = rospy.get_rostime()
        self.qr_msg.qrAlerts = []
        self.qr_msg.qrAlerts.append(QRAlertMsg(yaw = orderYaw,
                                              pitch = orderPitch,
                                              QRcontent = orderContent))
        self.qr_pub.publish(self.qr_msg)
        rospy.sleep(0.1)

    def deliverThermalOrder(self, orderYaw, orderPitch, orderProbability):

        self.general_msg.header.stamp = rospy.get_rostime()
        self.general_msg.yaw = orderYaw
        self.general_msg.pitch = orderPitch
        self.general_msg.probability = orderProbability
        self.thermal_pub.publish(self.general_msg)
        rospy.sleep(0.1)

    def deliverFaceOrder(self, orderYaw, orderPitch, orderProbability):

        self.general_msg.header.stamp = rospy.get_rostime()
        self.general_msg.yaw = orderYaw
        self.general_msg.pitch = orderPitch
        self.general_msg.probability = orderProbability
        self.face_pub.publish(self.general_msg)
        rospy.sleep(0.1)

    def deliverMotionOrder(self, orderYaw, orderPitch, orderProbability):

        self.general_msg.header.stamp = rospy.get_rostime()
        self.general_msg.yaw = orderYaw
        self.general_msg.pitch = orderPitch
        self.general_msg.probability = orderProbability
        self.motion_pub.publish(self.general_msg)
        rospy.sleep(0.1)

    def deliverSoundOrder(self, orderYaw, orderPitch, orderProbability):

        self.general_msg.header.stamp = rospy.get_rostime()
        self.general_msg.yaw = orderYaw
        self.general_msg.pitch = orderPitch
        self.general_msg.probability = orderProbability
        self.sound_pub.publish(self.general_msg)
        rospy.sleep(0.1)

    def deliverCo2Order(self, orderYaw, orderPitch, orderProbability):

        self.general_msg.header.stamp = rospy.get_rostime()
        self.general_msg.yaw = orderYaw
        self.general_msg.pitch = orderPitch
        self.general_msg.probability = orderProbability
        self.co2_pub.publish(self.general_msg)
        rospy.sleep(0.1)

    def deliverDataMatrixOrder(self, orderYaw, 
                      orderPitch, orderContent):

        self.dataMatrix_msg.header.stamp = rospy.get_rostime()
        self.dataMatrix_msg.dataMatrixAlerts = []
        self.dataMatrix_msg.dataMatrixAlerts.append(dataMatrixAlertMsg(yaw = orderYaw,
                                              pitch = orderPitch,
                                              datamatrixContent = orderContent))
        self.dataMatrix_pub.publish(self.dataMatrix_msg)
        rospy.sleep(0.1)

    def deliverNextOrder(self):
        
        if len(self.orderWaitingList) == 0:
            raise BadBossOrderFile("No orders left.")
        nextOrder = self.orderWaitingList.popleft()
        if nextOrder[1] == 'qr':
            self.qr_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hazmat':
            self.hazmat_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'landoltc':
            self.landoltc_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'datamatrix':
            self.dataMatrix_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hole':
            self.hole_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'thermal':
            self.thermal_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'face':
            self.face_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'motion':
            self.motion_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'sound':
            self.sound_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'co2':
            self.co2_pub.publish(nextOrder[0])
        rospy.sleep(0.1)

    def clearOrderList(self):

        self.orderWaitingList = deque([])

    def getOrderListFromBoss(self, boss):
       
        __dir__ = os.path.dirname(os.path.abspath(__file__))
        filepath = os.path.join(__dir__, boss)
        with open(filepath, 'r') as bossList:
            for order in bossList:
                a = order.split()
                if a[0] == 'qr:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    qr_msg = QRAlertsVectorMsg()
                    qr_msg.header.frame_id = self.frame_id
                    qr_msg.qrAlerts.append(QRAlertMsg(
                                           yaw = float(a[1]),
                                           pitch = float(a[2]),
                                           QRcontent = a[3]))
                    self.orderWaitingList.append((qr_msg, 'qr'))

                elif a[0] == 'hazmat:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    hazmat_msg = HazmatAlertsVectorMsg()
                    hazmat_msg.header.frame_id = self.frame_id
                    hazmat_msg.hazmatAlerts.append(HazmatAlertMsg(
                                                   yaw = float(a[1]),
                                                   pitch = float(a[2]),
                                                   patternType = int(a[3])))
                    self.orderWaitingList.append((hazmat_msg, 'hazmat'))

                elif a[0] == 'landoltc:':
                    if len(a) != 3:
                        raise BadBossOrderFile("Not right argument numbers.")
                    landoltc_msg = LandoltcAlertsVectorMsg()
                    landoltc_msg.header.frame_id = self.frame_id
                    landoltc_msg.landoltcAlerts.append(LandoltcAlertMsg(
                                                   yaw = float(a[1]),
                                                   pitch = float(a[2]),
                                                   angles = [1, 0, -0.25]))
                    self.orderWaitingList.append((landoltc_msg, 'landoltc'))

                elif a[0] == 'datamatrix:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    dataMatrix_msg = DataMatrixAlertsVectorMsg()
                    dataMatrix_msg.header.frame_id = self.frame_id
                    dataMatrix_msg.dataMatrixAlerts.append(DataMatrixAlertMsg(
                                                   yaw = float(a[1]),
                                                   pitch = float(a[2]),
                                                   datamatrixContent = a[3]))
                    self.orderWaitingList.append((dataMatrix_msg, 'datamatrix'))

                elif a[0] == 'hole:':
                    if len(a) != 4 and len(a) != 5:
                        raise BadBossOrderFile("Not right argument numbers.")
                    hole_msg = HolesDirectionsVectorMsg()
                    hole_msg.header.frame_id = self.frame_id
                    if len(a) == 5:
                        hole_msg.holesDirections.append(HoleDirectionMsg(
                                                        yaw = float(a[1]),
                                                        pitch = float(a[2]),
                                                        probability = float(a[4]),
                                                        holeId = int(a[3])))
                    elif len(a) == 4:
                         hole_msg.holesDirections.append(HoleDirectionMsg(
                                                        yaw = float(a[1]),
                                                        pitch = float(a[2]),
                                                        probability = 1.,
                                                        holeId = int(a[3])))
                    self.orderWaitingList.append((hole_msg, 'hole'))

                elif a[0] == 'thermal:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    thermal_msg = GeneralAlertMsg()
                    thermal_msg.header.frame_id = self.frame_id
                    thermal_msg.yaw = float(a[1])
                    thermal_msg.pitch = float(a[2])
                    thermal_msg.probability = float(a[3])
                    self.orderWaitingList.append((thermal_msg, 'thermal'))
                
                elif a[0] == 'face:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    face_msg = GeneralAlertMsg()
                    face_msg.header.frame_id = self.frame_id
                    face_msg.yaw = float(a[1])
                    face_msg.pitch = float(a[2])
                    face_msg.probability = float(a[3])
                    self.orderWaitingList.append((face_msg, 'face'))
                
                elif a[0] == 'motion:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    motion_msg = GeneralAlertMsg()
                    motion_msg.header.frame_id = self.frame_id
                    motion_msg.yaw = float(a[1])
                    motion_msg.pitch = float(a[2])
                    motion_msg.probability = float(a[3])
                    self.orderWaitingList.append((motion_msg, 'motion'))
                
                elif a[0] == 'sound:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    sound_msg = GeneralAlertMsg()
                    sound_msg.header.frame_id = self.frame_id
                    sound_msg.yaw = float(a[1])
                    sound_msg.pitch = float(a[2])
                    sound_msg.probability = float(a[3])
                    self.orderWaitingList.append((sound_msg, 'sound'))
                
                elif a[0] == 'co2:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    co2_msg = GeneralAlertMsg()
                    co2_msg.header.frame_id = self.frame_id
                    co2_msg.yaw = float(a[1])
                    co2_msg.pitch = float(a[2])
                    co2_msg.probability = float(a[3])
                    self.orderWaitingList.append((co2_msg, 'co2'))
                
                else:
                    raise BadBossOrderFile("Not recognized object type.")

    def getOrderListFromBag(self):
        
        pass
