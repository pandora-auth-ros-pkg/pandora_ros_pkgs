#!/usr/bin/env python

from __future__ import print_function

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


def server():
    rospy.init_node("data_fusion_objects")
    rospy.Service('data_fusion_geotiff', GetGeotiff, send_objects)

    print("Service initialized.")
    rospy.spin()

if __name__ == "__main__":
    server()
