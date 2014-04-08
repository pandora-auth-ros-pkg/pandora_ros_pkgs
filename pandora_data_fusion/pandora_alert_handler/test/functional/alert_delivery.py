#!usr/bin/env python

import rospy

from vision_communications.msg import HoleDirectionMsg
from vision_communications.msg import HolesDirectionsVectorMsg
from vision_communications.msg import HazmatAlertMsg
from vision_communications.msg import HazmatAlertsVectorMsg
from vision_communications.msg import QRAlertsVectorMsg
from vision_communications.msg import QRAlertMsg
from data_fusion_communications.msg import ThermalDirectionAlertMsg

class BadBossOrderFile(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class AlertDeliveryBoy:

    def __init__(self):

        self.frame_id = 'headCamera'

        self.orderWaitingList = []
        
        self.hazmat_msg = HazmatAlertsVectorMsg()
        self.hazmat_msg.header.frame_id = self.frame_id
        
        self.qr_msg = QRAlertsVectorMsg()
        self.qr_msg.header.frame_id = self.frame_id
        
        self.hole_msg = HolesDirectionsVectorMsg()
        self.hole_msg.header.frame_id = self.frame_id

        self.tpa_msg = ThermalDirectionAlertMsg()
        self.tpa_msg.header.frame_id = self.frame_id
        
        self.hazmatDeliveryAddress = '/vision/hazmat_alert'
        self.hazmat_pub = rospy.Publisher(self.hazmatDeliveryAddress, 
                                          HazmatAlertsVectorMsg)
        self.qrDeliveryAddress = '/vision/qr_alert'
        self.qr_pub = rospy.Publisher(self.qrDeliveryAddress, 
                                      QRAlertsVectorMsg)
        self.holeDeliveryAddress = '/data_fusion/victim_fusion/'+\
                                   'hole_direction_alert'
        self.hole_pub = rospy.Publisher(self.holeDeliveryAddress, 
                                        HolesDirectionsVectorMsg)
        self.tpaDeliveryAddress = '/data_fusion/victim_fusion/'+\
                                  'tpa_direction_alert'
        self.tpa_pub = rospy.Publisher(self.tpaDeliveryAddress,
                                       ThermalDirectionAlertMsg)

    def deliverHoleOrder(self, orderYaw, 
                        orderPitch, orderId, orderProbability = 1):

        self.hole_msg.header.stamp = rospy.get_rostime()
        self.hole_msg.holesDirections = []
        self.hole_msg.holesDirections.append(HoleDirectionMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            probability = orderProbability,
                                            holeId = orderId))
        self.hole_pub.publish(self.hole_msg)
        for i in range(2): 
	    self.hole_pub.publish(self.hole_msg)
	    rospy.sleep(1)

    def deliverHazmatOrder(self, orderYaw, 
                          orderPitch, orderPattern):

        self.hazmat_msg.header.stamp = rospy.get_rostime()
        self.hazmat_msg.hazmatAlerts = []
        self.hazmat_msg.hazmatAlerts.append(HazmatAlertMsg(yaw = orderYaw,
                                            pitch = orderPitch,
                                            patternType = orderPattern))
        self.hazmat_pub.publish(self.hazmat_msg)
        for i in range(2): 
	    self.hazmat_pub.publish(self.hazmat_msg)
	    rospy.sleep(1)

    def deliverQrOrder(self, orderYaw, 
                      orderPitch, orderContent):

        self.qr_msg.header.stamp = rospy.get_rostime()
        self.qr_msg.qrAlerts = []
        self.qr_msg.qrAlerts.append(QRAlertMsg(yaw = orderYaw,
                                              pitch = orderPitch,
                                              QRcontent = orderContent))
        self.qr_pub.publish(self.qr_msg)
        for i in range(2): 
	    self.qr_pub.publish(self.qr_msg)
	    rospy.sleep(1)

    def deliverTpaOrder(self, orderYaw, orderPitch, orderProbability):

        self.tpa_msg.header.stamp = rospy.get_rostime()
        self.tpa_msg.yaw = orderYaw
        self.tpa_msg.pitch = orderPitch
        self.tpa_msg.probability = orderProbability
        self.tpa_pub.publish(self.tpa_msg)
        for i in range(2): 
	    self.tpa_pub.publish(self.tpa_msg)
	    rospy.sleep(1)

    def deliverNextOrder(self):
        
        nextOrder = self.orderWaitingList.pop()
        if nextOrder[1] == 'qr':
            self.qr_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hazmat':
            self.hazmat_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'hole':
            self.hole_pub.publish(nextOrder[0])
        elif nextOrder[1] == 'tpa':
            self.tpa_pub.publish(nextOrder[0])

    def clearOrderList(self):

        self.orderWaitingList = []

    def getOrderListFromBoss(self, boss):
        
        with open(boss, 'r') as bossList:
            for order in bossList:
                a = order[:-2].split()
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

                elif a[0] == 'tpa:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    tpa_msg = ThermalDirectionAlertMsg()
                    tpa_msg.header.frame_id = self.frame_id
                    tpa_msg.yaw = float(a[1])
                    tpa_msg.pitch = float(a[2])
                    tpa_msg.probability = float(a[3])
                    self.orderWaitingList.append((tpa_msg, 'tpa'))
                
                else:
                    raise BadBossOrderFile("Not recognized object type.")

    def getOrderListFromBag(self):
        
        pass
