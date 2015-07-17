#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
import random
import string

from pandora_data_fusion_msgs.msg import (QrInfo, VictimInfo, HazmatInfo,
                                          ObstacleInfo)
from pandora_data_fusion_msgs.srv import GetGeotiff, GetGeotiffResponse


def generate_content(size=6, chars=string.ascii_uppercase):
    return ''.join(random.choice(chars) for _ in range(size))


def random_qr_info(id):
    qr = QrInfo()
    qr.id = id
    qr.qrFrameId = 'map'
    qr.timeFound = rospy.Time.now()
    qr.qrPose.pose.position.x = random.randint(3, 7)
    qr.qrPose.pose.position.y = random.randint(3, 7)
    qr.qrPose.pose.position.z = random.randint(3, 7)
    qr.qrPose.header.frame_id = 'map'
    qr.probability = random.random()
    qr.content = generate_content()

    return qr


def random_victim_info(id):
    victim = VictimInfo()
    victim.id = id
    victim.victimFrameId = 'map'
    victim.timeFound = rospy.Time.now()
    victim.victimPose.pose.position.x = random.randint(2, 10)
    victim.victimPose.pose.position.y = random.randint(2, 10)
    victim.victimPose.pose.position.z = random.randint(2, 10)
    victim.victimPose.header.frame_id = 'map'
    victim.probability = random.random()

    return victim


def random_hazmats_info(id):
    hazmat = HazmatInfo()
    hazmat.id = id
    hazmat.hazmatFrameId = 'map'
    hazmat.timeFound = rospy.Time.now()
    hazmat.hazmatPose.pose.position.x = random.randint(2, 10)
    hazmat.hazmatPose.pose.position.y = random.randint(2, 10)
    hazmat.hazmatPose.pose.position.z = random.randint(2, 10)
    hazmat.hazmatPose.header.frame_id = 'map'
    hazmat.probability = random.random()

    return hazmat


def random_obstacle_info(id):
    obstacle = ObstacleInfo()
    obstacle.id = id
    obstacle.obstacleFrameId = 'map'
    obstacle.timeFound = rospy.Time.now()
    obstacle.obstaclePose.pose.position.x = random.randint(3, 8)
    obstacle.obstaclePose.pose.position.y = random.randint(3, 8)
    obstacle.obstaclePose.pose.position.z = random.randint(3, 8)
    obstacle.obstaclePose.header.frame_id = 'map'
    obstacle.probability = random.random()

    return obstacle


def send_objects(req):
    print("Service called.")

    qrs = [random_qr_info(i + 1) for i in range(6)]
    victims = [random_victim_info(i + 1) for i in range(3)]
    hazmats = [random_hazmats_info(i + 1) for i in range(3)]
    obstacles = [random_obstacle_info(i + 1) for i in range(3)]

    return GetGeotiffResponse(victims=victims,
                              hazmats=hazmats,
                              qrs=qrs,
                              obstacles=obstacles)


def send_objects_with_order(req):
    print("Service with order called.")

    qr1 = QrInfo()
    qr1.qrPose.pose.position.x = 1
    qr1.qrPose.pose.position.y = 1
    qr1.qrPose.header.frame_id = 'map'
    qr1.timeFound = rospy.Time.now() + rospy.Duration(100)
    qr2 = QrInfo()
    qr2.qrPose.pose.position.x = 1.5
    qr2.qrPose.pose.position.y = 1.5
    qr2.qrPose.header.frame_id = 'map'
    qr2.timeFound = rospy.Time.now() + rospy.Duration(95)
    qr3 = QrInfo()
    qr3.qrPose.pose.position.x = 2
    qr3.qrPose.pose.position.y = 2
    qr3.qrPose.header.frame_id = 'map'
    qr3.timeFound = rospy.Time.now() + rospy.Duration(90)

    victim1 = VictimInfo()
    victim1.victimPose.pose.position.x = 2.5
    victim1.victimPose.pose.position.y = 2.5
    victim1.victimPose.header.frame_id = 'map'
    victim1.timeFound = rospy.Time.now() + rospy.Duration(85)
    victim2 = VictimInfo()
    victim2.victimPose.pose.position.x = 3
    victim2.victimPose.pose.position.y = 3
    victim2.victimPose.header.frame_id = 'map'
    victim2.timeFound = rospy.Time.now() + rospy.Duration(80)
    victim3 = VictimInfo()
    victim3.victimPose.pose.position.x = 3.5
    victim3.victimPose.pose.position.y = 3.5
    victim3.victimPose.header.frame_id = 'map'
    victim3.timeFound = rospy.Time.now() + rospy.Duration(75)

    hazmat1 = HazmatInfo()
    hazmat1.hazmatPose.pose.position.x = 4
    hazmat1.hazmatPose.pose.position.y = 4
    hazmat1.timeFound = rospy.Time.now() + rospy.Duration(70)
    hazmat1.hazmatPose.header.frame_id = 'map'
    hazmat2 = HazmatInfo()
    hazmat2.hazmatPose.pose.position.x = 4.5
    hazmat2.hazmatPose.pose.position.y = 4.5
    hazmat2.hazmatPose.header.frame_id = 'map'
    hazmat2.timeFound = rospy.Time.now() + rospy.Duration(65)
    hazmat3 = HazmatInfo()
    hazmat3.hazmatPose.pose.position.x = 5
    hazmat3.hazmatPose.pose.position.y = 5
    hazmat3.hazmatPose.header.frame_id = 'map'
    hazmat3.timeFound = rospy.Time.now() + rospy.Duration(60)

    obstacle1 = ObstacleInfo()
    obstacle1.obstaclePose.pose.position.x = 5.5
    obstacle1.obstaclePose.pose.position.y = 5.5
    obstacle1.obstaclePose.header.frame_id = 'map'
    obstacle1.timeFound = rospy.Time.now() + rospy.Duration(55)
    obstacle2 = ObstacleInfo()
    obstacle2.obstaclePose.pose.position.x = 6
    obstacle2.obstaclePose.pose.position.y = 6
    obstacle2.obstaclePose.header.frame_id = 'map'
    obstacle2.timeFound = rospy.Time.now() + rospy.Duration(50)
    obstacle3 = ObstacleInfo()
    obstacle3.obstaclePose.pose.position.x = 0
    obstacle3.obstaclePose.pose.position.y = 0
    obstacle3.obstaclePose.header.frame_id = 'map'
    obstacle3.timeFound = rospy.Time.now() + rospy.Duration(45)

    qrs = [qr1, qr2, qr3]
    victims = [victim1, victim2, victim3]
    hazmats = [hazmat1, hazmat2, hazmat3]
    obstacles = [obstacle1, obstacle2, obstacle3]

    return GetGeotiffResponse(victims=victims,
                              hazmats=hazmats,
                              qrs=qrs,
                              obstacles=obstacles)


def server():
    rospy.init_node("data_fusion_objects")

    rospy.Service('/data_fusion/get_geotiff', GetGeotiff, send_objects_with_order)
    #rospy.Service('data_fusion_geotiff', GetGeotiff, send_objects)

    print("Service initialized.")
    rospy.spin()

if __name__ == "__main__":
    server()
