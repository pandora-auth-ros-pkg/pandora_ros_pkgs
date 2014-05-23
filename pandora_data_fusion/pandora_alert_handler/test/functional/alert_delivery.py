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
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.header.frame_id = self.frame_id
        
        self.hole_msg = HolesDirectionsVectorMsg()
        self.hole_msg.header.frame_id = self.frame_id

        self.thermal_msg = GeneralAlertMsg()
        self.thermal_msg.header.frame_id = self.frame_id
        
        self.hazmatDeliveryAddress = '/vision/hazmat_alert'
        self.hazmat_pub = rospy.Publisher(self.hazmatDeliveryAddress, 
                                          HazmatAlertsVectorMsg)
        self.qrDeliveryAddress = '/vision/qr_alert'
        self.qr_pub = rospy.Publisher(self.qrDeliveryAddress, 
                                      QRAlertsVectorMsg)
        self.holeDeliveryAddress = '/vision/hole_direction_alert'
        self.hole_pub = rospy.Publisher(self.holeDeliveryAddress, 
                                        HolesDirectionsVectorMsg)
        self.thermalDeliveryAddress = '/sensor_processors/thermal_direction_alert'
        self.thermal_pub = rospy.Publisher(self.thermalDeliveryAddress,
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

    def deliverHazmatOrder(self, orderYaw, 
                          orderPitch, orderPattern):

        self.hazmat_msg.header.stamp = rospy.get_rostime()
        self.hazmat_msg.hazmatAlerts = []
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            patternType = orderPattern))
        self.hazmat_pub.publish(self.hazmat_msg)

    def deliverQrOrder(self, orderYaw, 
                      orderPitch, orderContent):

        self.qr_msg.header.stamp = rospy.get_rostime()
        self.qr_msg.qrAlerts = []
        self.qr_msg.qrAlerts.append(QRAlertMsg(yaw = orderYaw,
                                              pitch = orderPitch,
                                              QRcontent = orderContent))
        self.qr_pub.publish(self.qr_msg)

    def deliverThermalOrder(self, orderYaw, orderPitch, orderProbability):

        self.thermal_msg.header.stamp = rospy.get_rostime()
        self.thermal_msg.yaw = orderYaw
        self.thermal_msg.pitch = orderPitch
        self.thermal_msg.probability = orderProbability
        self.thermal_pub.publish(self.thermal_msg)

    def deliverNextOrder(self):
        
        if len(self.orderWaitingList) == 0:
            raise BadBossOrderFile("No orders left.")
        nextOrder = self.orderWaitingList.popleft()
        if nextOrder[1] == 'qr':
            self.qr_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hazmat':
            self.hazmat_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hole':
            self.hole_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'thermal':
            self.thermal_pub.publish(nextOrder[0])

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
                    thermal_msg = ThermalDirectionAlertMsg()
                    thermal_msg.header.frame_id = self.frame_id
                    thermal_msg.yaw = float(a[1])
                    thermal_msg.pitch = float(a[2])
                    thermal_msg.probability = float(a[3])
                    self.orderWaitingList.append((thermal_msg, 'thermal'))
                
                else:
                    raise BadBossOrderFile("Not recognized object type.")

    def getOrderListFromBag(self):
        
        pass
