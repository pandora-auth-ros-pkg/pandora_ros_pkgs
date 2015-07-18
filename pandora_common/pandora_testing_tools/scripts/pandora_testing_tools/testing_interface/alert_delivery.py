#!usr/bin/env python
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
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
__author__ = "Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import os
from collections import deque

import roslib; roslib.load_manifest('pandora_testing_tools')
import rospy

from pandora_vision_msgs.msg import ObstacleAlert
from pandora_vision_msgs.msg import ObstacleAlertVector
from pandora_vision_msgs.msg import HoleDirectionAlert
from pandora_vision_msgs.msg import HoleDirectionAlertVector
from pandora_vision_msgs.msg import ThermalAlert
from pandora_vision_msgs.msg import ThermalAlertVector
from pandora_vision_msgs.msg import HazmatAlert
from pandora_vision_msgs.msg import HazmatAlertVector
from pandora_vision_msgs.msg import QRAlert
from pandora_vision_msgs.msg import QRAlertVector
from pandora_vision_msgs.msg import LandoltcAlert
from pandora_vision_msgs.msg import LandoltcAlertVector
from pandora_vision_msgs.msg import DataMatrixAlert
from pandora_vision_msgs.msg import DataMatrixAlertVector
from pandora_audio_msgs.msg import SoundAlert
from pandora_audio_msgs.msg import SoundAlertVector
from pandora_common_msgs.msg import GeneralAlertInfo
from pandora_common_msgs.msg import GeneralAlertVector

class BadBossOrderFile(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class AlertDeliveryBoy():

    def __init__(self, frame_id):

        self.frame_id = frame_id

        self.orderWaitingList = deque([])

        self.hazmat_msg = HazmatAlertVector()
        self.hazmat_msg.header.frame_id = self.frame_id

        self.landoltc_msg = LandoltcAlertVector()
        self.landoltc_msg.header.frame_id = self.frame_id

        self.qr_msg = QRAlertVector()
        self.qr_msg.header.frame_id = self.frame_id

        self.dataMatrix_msg = DataMatrixAlertVector()
        self.dataMatrix_msg.header.frame_id = self.frame_id

        self.hole_msg = HoleDirectionAlertVector()
        self.hole_msg.header.frame_id = self.frame_id

        self.obstacle_msg = ObstacleAlertVector()
        self.obstacle_msg.header.frame_id = self.frame_id

        self.barrel_msg = ObstacleAlertVector()
        self.barrel_msg.header.frame_id = self.frame_id

        self.thermal_msg = ThermalAlertVector()
        self.thermal_msg.header.frame_id = self.frame_id

        self.sound_msg = SoundAlertVector()
        self.sound_msg.header.frame_id = self.frame_id

        self.general_msg = GeneralAlertVector()
        self.general_msg.header.frame_id = self.frame_id

        self.hazmatDeliveryAddress = '/vision/hazmat_alert'
        self.hazmat_pub = rospy.Publisher(self.hazmatDeliveryAddress,
                                          HazmatAlertVector)
        self.landoltcDeliveryAddress = '/vision/landoltc_alert'
        self.landoltc_pub = rospy.Publisher(self.landoltcDeliveryAddress,
                                            LandoltcAlertVector)
        self.qrDeliveryAddress = '/vision/qr_alert'
        self.qr_pub = rospy.Publisher(self.qrDeliveryAddress,
                                      QRAlertVector)
        self.holeDeliveryAddress = '/vision/holes_direction_alert'
        self.hole_pub = rospy.Publisher(self.holeDeliveryAddress,
                                        HoleDirectionAlertVector)
        self.obstacleDeliveryAddress = '/vision/obstacle_alert'
        self.obstacle_pub = rospy.Publisher(self.obstacleDeliveryAddress,
                                            ObstacleAlertVector)
        self.barrel_pub = rospy.Publisher(self.obstacleDeliveryAddress,
                                            ObstacleAlertVector)
        self.dataMatrixDeliveryAddress = '/vision/dataMatrix_alert'
        self.dataMatrix_pub = rospy.Publisher(self.dataMatrixDeliveryAddress,
                                      DataMatrixAlertVector)
        self.thermalDeliveryAddress = '/vision/thermal_direction_alert'
        self.thermal_pub = rospy.Publisher(self.thermalDeliveryAddress,
                                       ThermalAlertVector)
        self.visualVictimDeliveryAddress = '/vision/victim_direction_alert'
        self.visualVictim_pub = rospy.Publisher(self.visualVictimDeliveryAddress,
                                       GeneralAlertVector)
        self.soundDeliveryAddress = '/sound/complete_alert'
        self.sound_pub = rospy.Publisher(self.soundDeliveryAddress,
                                         SoundAlertVector)
        self.motionDeliveryAddress = '/vision/motion_alert'
        self.motion_pub = rospy.Publisher(self.motionDeliveryAddress,
                                       GeneralAlertVector)
        self.co2DeliveryAddress = '/sensor_processing/co2_alert'
        self.co2_pub = rospy.Publisher(self.co2DeliveryAddress,
                                       GeneralAlertVector)


    def makeGeneralAlertInfo(self, yaw, pitch, probability):
        info = GeneralAlertInfo()
        info.yaw = yaw
        info.pitch = pitch
        info.probability = probability
        return info

    def makeGeneralAlertVector(self, yaw, pitch, probability):
        msg = GeneralAlertVector()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = self.frame_id
        msg.alerts = []
        msg.alerts.append(self.makeGeneralAlertInfo(yaw, pitch, probability))
        return msg

    def deliverObstacleOrder(self, pointsYaw, pointsPitch,
            pointsDepth, probability, obs_type, publish=True):

        self.obstacle_msg.header.stamp = rospy.get_rostime()
        self.obstacle_msg.alerts = []
        msg = ObstacleAlert()
        msg.type = obs_type
        msg.probability = probability
        msg.pointsYaw = pointsYaw
        msg.pointsPitch = pointsPitch
        msg.pointsDepth = pointsDepth
        self.obstacle_msg.alerts.append(msg)
        rospy.loginfo(self.obstacle_msg)
        if publish:
            self.obstacle_pub.publish(self.obstacle_msg)
            rospy.sleep(0.1)
        else:
            return self.obstacle_msg

    def deliverBarrelOrder(self, yaw, pitch, probability, publish=True):

        self.barrel_msg.header.stamp = rospy.get_rostime()
        self.barrel_msg.alerts = []
        msg = ObstacleAlert()
        msg.type = 0
        msg.probability = probability
        msg.pointsYaw[0] = yaw
        msg.pointsPitch[0] = pitch
        msg.pointsDepth[0] = 1.0
        self.barrel_msg.alerts.append(msg)
        rospy.loginfo(self.barrel_msg)
        if publish:
            self.obstacle_pub.publish(self.barrel_msg)
            rospy.sleep(0.1)
        else:
            return self.barrel_msg

    def deliverHoleOrder(self, orderYaw,
                        orderPitch, orderProbability=1, orderId=1, publish=True):

        self.hole_msg.header.stamp = rospy.get_rostime()
        self.hole_msg.alerts = []
        msg = HoleDirectionAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, orderProbability),
                holeId=orderId)
        print msg
        self.hole_msg.alerts.append(msg)
        rospy.loginfo(self.hole_msg)
        if publish:
            self.hole_pub.publish(self.hole_msg)
            rospy.sleep(0.1)
        else:
            return self.hole_msg

    def deliverHazmatOrder(self, orderYaw,
                          orderPitch, orderPattern, publish=True):

        self.hazmat_msg.header.stamp = rospy.get_rostime()
        self.hazmat_msg.alerts = []
        msg = HazmatAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, 1),
                patternType=orderPattern)
        self.hazmat_msg.alerts.append(msg)
        if publish:
            self.hazmat_pub.publish(self.hazmat_msg)
            rospy.sleep(0.1)
        else:
            return self.hazmat_msg

    def deliverLandoltcOrder(self, orderYaw,
                             orderPitch, orderAngles, publish=True):

        self.landoltc_msg.header.stamp = rospy.get_rostime()
        self.landoltc_msg.alerts = []
        msg = LandoltcAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, 1),
                angles=orderAngles)
        self.landoltc_msg.alerts.append(msg)
        if publish:
            self.landoltc_pub.publish(self.landoltc_msg)
            rospy.sleep(0.1)
        else:
            return self.landoltc_msg

    def deliverQrOrder(self, orderYaw, orderPitch, orderContent, publish=True):

        self.qr_msg.header.stamp = rospy.get_rostime()
        self.qr_msg.alerts = []
        msg = QRAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, 1),
                QRcontent=orderContent)
        self.qr_msg.alerts.append(msg)
        if publish:
            self.qr_pub.publish(self.qr_msg)
            rospy.sleep(0.1)
        else:
            return self.qr_msg

    def deliverDataMatrixOrder(self, orderYaw, orderPitch, orderContent,
            publish=True):

        self.dataMatrix_msg.header.stamp = rospy.get_rostime()
        self.dataMatrix_msg.alerts = []
        msg = DataMatrixAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, 1),
                datamatrixContent=orderContent)
        self.dataMatrix_msg.alerts.append(msg)
        if publish:
            self.dataMatrix_pub.publish(self.dataMatrix_msg)
            rospy.sleep(0.1)
        else:
            return self.dataMatrix_msg

    def deliverThermalOrder(self, orderYaw, orderPitch, orderProbability,
            publish=True):
        self.thermal_msg.header.stamp = rospy.get_rostime()
        self.thermal_msg.alerts = []
        msg = ThermalAlert(
                info=self.makeGeneralAlertInfo(orderYaw, orderPitch, orderProbability),
                temperature=35.0)
        self.thermal_msg.alerts.append(msg)
        if publish:
            self.thermal_pub.publish(self.thermal_msg)
            rospy.sleep(0.1)
        else:
            return self.thermal_msg

    def deliverSoundOrder(self, orderYaw, orderPitch, orderProbability, orderWord,
            publish=True):
        self.sound_msg.header.stamp = rospy.get_rostime()
        self.sound_msg.alerts = []
        msg = SoundAlert(info=self.makeGeneralAlertInfo(orderYaw, orderPitch, orderProbability),
                         word=orderWord)
        self.sound_msg.alerts.append(msg)
        if publish:
            self.sound_pub.publish(self.sound_msg)
            rospy.sleep(0.1)
        else:
            return msg

    def deliverVisualVictimOrder(self, orderYaw, orderPitch, orderProbability,
            publish=True):
        msg = self.makeGeneralAlertVector(orderYaw, orderPitch, orderProbability)
        if publish:
            self.visualVictim_pub.publish(msg)
            rospy.sleep(0.1)
        else:
            return msg

    def deliverMotionOrder(self, orderYaw, orderPitch, orderProbability,
            publish=True):
        msg = self.makeGeneralAlertVector(orderYaw, orderPitch, orderProbability)
        if publish:
            self.motion_pub.publish(msg)
            rospy.sleep(0.1)
        else:
            return msg

    def deliverCo2Order(self, orderYaw, orderPitch, orderProbability,
            publish=True):
        msg = self.makeGeneralAlertVector(orderYaw, orderPitch, orderProbability)
        if publish:
            self.co2_pub.publish(msg)
            rospy.sleep(0.1)
        else:
            return msg

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
        elif nextOrder[1] == 'visualVictim':
            self.visualVictim_pub.publish(nextOrder[0])
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
                    qr_msg = self.deliverQrOrder(float(a[1]), float(a[2]), a[3])
                    self.orderWaitingList.append((qr_msg, 'qr'))

                elif a[0] == 'hazmat:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    hazmat_msg = self.deliverHazmatOrder(float(a[1]), float(a[2]), int(a[3]))
                    self.orderWaitingList.append((hazmat_msg, 'hazmat'))

                elif a[0] == 'landoltc:':
                    if len(a) != 3:
                        raise BadBossOrderFile("Not right argument numbers.")
                    angles = [1, 0, -0.25]
                    landoltc_msg = self.deliverLandoltcOrder(float(a[1]), float(a[2]), angles)
                    self.orderWaitingList.append((landoltc_msg, 'landoltc'))

                elif a[0] == 'datamatrix:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    dataMatrix_msg = self.deliverDataMatrixOrder(float(a[1]), float(a[2]),
                            a[3])
                    self.orderWaitingList.append((dataMatrix_msg, 'datamatrix'))

                elif a[0] == 'hole:':
                    if len(a) != 4 and len(a) != 5:
                        raise BadBossOrderFile("Not right argument numbers.")
                    hole_msg = HoleDirectionAlertVector()
                    if len(a) == 5:
                        hole_msg = self.deliverHoleOrder(float(a[1]), float(a[2]),
                                float(a[4]), int(a[3]))
                    elif len(a) == 4:
                        hole_msg = self.deliverHoleOrder(float(a[1]), float(a[2]),
                                1., int(a[3]))
                    self.orderWaitingList.append((hole_msg, 'hole'))

                elif a[0] == 'thermal:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    thermal_msg = GeneralAlertInfo()
                    thermal_msg.header.frame_id = self.frame_id
                    thermal_msg.yaw = float(a[1])
                    thermal_msg.pitch = float(a[2])
                    thermal_msg.probability = float(a[3])
                    self.orderWaitingList.append((thermal_msg, 'thermal'))

                elif a[0] == 'visualVictim:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    visualVictim_msg = GeneralAlertInfo()
                    visualVictim_msg.header.frame_id = self.frame_id
                    visualVictim_msg.yaw = float(a[1])
                    visualVictim_msg.pitch = float(a[2])
                    visualVictim_msg.probability = float(a[3])
                    self.orderWaitingList.append((visualVictim_msg, 'visualVictim'))

                elif a[0] == 'motion:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    motion_msg = GeneralAlertInfo()
                    motion_msg.header.frame_id = self.frame_id
                    motion_msg.yaw = float(a[1])
                    motion_msg.pitch = float(a[2])
                    motion_msg.probability = float(a[3])
                    self.orderWaitingList.append((motion_msg, 'motion'))

                elif a[0] == 'sound:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    sound_msg = GeneralAlertInfo()
                    sound_msg.header.frame_id = self.frame_id
                    sound_msg.yaw = float(a[1])
                    sound_msg.pitch = float(a[2])
                    sound_msg.probability = float(a[3])
                    self.orderWaitingList.append((sound_msg, 'sound'))

                elif a[0] == 'co2:':
                    if len(a) != 4:
                        raise BadBossOrderFile("Not right argument numbers.")
                    co2_msg = GeneralAlertInfo()
                    co2_msg.header.frame_id = self.frame_id
                    co2_msg.yaw = float(a[1])
                    co2_msg.pitch = float(a[2])
                    co2_msg.probability = float(a[3])
                    self.orderWaitingList.append((co2_msg, 'co2'))

                else:
                    raise BadBossOrderFile("Not recognized object type.")

    def getOrderListFromBag(self):

        pass
