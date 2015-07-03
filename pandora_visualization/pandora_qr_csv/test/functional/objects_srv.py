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
    qr.timeFound = rospy.Time.now() + rospy.Duration(id * 20)
    qr.qrPose.pose.position.x = random.randint(2, 10)
    qr.qrPose.pose.position.y = random.randint(2, 10)
    qr.qrPose.pose.position.z = random.randint(2, 10)
    qr.probability = random.random()
    qr.content = generate_content()

    return qr


def random_obstacle_info(id):
    obstacle = ObstacleInfo()
    obstacle.id = id
    obstacle.timeFound = rospy.Time.now() + rospy.Duration(id * 60)
    obstacle.obstaclePose.pose.position.x = random.randint(2, 10)
    obstacle.obstaclePose.pose.position.y = random.randint(2, 10)
    obstacle.obstaclePose.pose.position.z = random.randint(2, 10)
    obstacle.probability = random.random()
    obstacle.type = random.randint(1, 3)

    return obstacle


def send_objects(req):
    print("Service called.")

    qrs = [random_qr_info(i + 1) for i in range(6)]
    victims = [VictimInfo() for i in range(3)]
    hazmats = [HazmatInfo() for i in range(3)]
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
