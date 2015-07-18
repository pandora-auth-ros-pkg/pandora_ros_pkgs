#!/usr/bin/env python
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
__author__ = "Afouras Triantafyllos and Tsirigotis Christos"
__maintainer__ = "Tsirigotis Christos"
__email__ = "tsirif@gmail.com"

import roslib
roslib.load_manifest('pandora_alert_handler')
import rospy
import actionlib
import sys
from pandora_testing_tools.testing_interface import alert_delivery

from dynamic_reconfigure.server import Server
from pandora_alert_handler.cfg import MassAlertPublisherConfig


class MassPublisher:

    def __init__(self, frame_id):

        self.alertDeliveryBoy = alert_delivery.AlertDeliveryBoy(frame_id)

        self.dyn_reconf_srv = Server(MassAlertPublisherConfig, self.dyn_reconf_callback)

        self.qr_post = False
        self.obstacle_post = False
        self.barrel_post = False
        self.hazmat_post = False
        self.landoltc_post = False
        self.dataMatrix_post = False
        self.hole_1_post = False
        self.hole_2_post = False
        self.thermal_post = False
        self.visualVictim_post = False
        self.motion_post = False
        self.sound_post = False
        self.co2_post = False

        self.config = None

        self.publish_stuff()

    def dyn_reconf_callback(self, config, level):

        self.config = config
        return config

    def publish_stuff(self):

        while not rospy.is_shutdown():
            if self.config is not None:
                if self.config.hazmat_post:
                    self.alertDeliveryBoy.deliverHazmatOrder(
                        self.config.hazmat_yaw,
                        self.config.hazmat_pitch,
                        self.config.hazmat_pattern)

                if self.config.qr_post:
                    self.alertDeliveryBoy.deliverQrOrder(
                        self.config.qr_yaw,
                        self.config.qr_pitch,
                        self.config.qr_content)

                if self.config.dataMatrix_post:
                    self.alertDeliveryBoy.deliverDataMatrixOrder(
                        self.config.dataMatrix_yaw,
                        self.config.dataMatrix_pitch,
                        self.config.dataMatrix_content)

                if self.config.landoltc_post:
                    self.alertDeliveryBoy.deliverLandoltcOrder(
                        self.config.landoltc_yaw,
                        self.config.landoltc_pitch,
                        orderAngles=[1, 0, 0.25])

                if self.config.motion_post:
                    self.alertDeliveryBoy.deliverMotionOrder(
                        self.config.motion_yaw,
                        self.config.motion_pitch,
                        self.config.motion_probability)

                if self.config.co2_post:
                    self.alertDeliveryBoy.deliverCo2Order(
                        self.config.co2_yaw,
                        self.config.co2_pitch,
                        self.config.co2_probability)

                if self.config.sound_post:
                    self.alertDeliveryBoy.deliverSoundOrder(
                        self.config.sound_yaw,
                        self.config.sound_pitch,
                        self.config.sound_probability,
                        self.config.sound_word)

                if self.config.visualVictim_post:
                    self.alertDeliveryBoy.deliverVisualVictimOrder(
                        self.config.visualVictim_yaw,
                        self.config.visualVictim_pitch,
                        self.config.visualVictim_probability)

                if self.config.thermal_post:
                    self.alertDeliveryBoy.deliverThermalOrder(
                        self.config.thermal_yaw,
                        self.config.thermal_pitch,
                        self.config.thermal_probability)

                if self.config.hole_1_post:
                    self.alertDeliveryBoy.deliverHoleOrder(
                        self.config.hole_1_yaw,
                        self.config.hole_1_pitch,
                        self.config.hole_1_probability,
                        self.config.hole_1_id)

                if self.config.hole_2_post:
                    self.alertDeliveryBoy.deliverHoleOrder(
                        self.config.hole_2_yaw,
                        self.config.hole_2_pitch,
                        self.config.hole_2_probability,
                        self.config.hole_2_id)

                if self.config.obstacle_post:
                    self.alertDeliveryBoy.deliverObstacleOrder(
                        (0.0, self.config.obstacle_2_yaw, 0.0, self.config.obstacle_1_yaw),
                        (0.0, self.config.obstacle_2_pitch, 0.0, self.config.obstacle_1_pitch),
                        (0.0, self.config.obstacle_2_depth, 0.0, self.config.obstacle_1_depth),
                        self.config.obstacle_probability,
                        self.config.obstacle_type)

                if self.config.barrel_post:
                    self.alertDeliveryBoy.deliverBarrelOrder(
                            self.config.barrel_yaw,
                            self.config.barrel_pitch,
                            self.config.barrel_probability)

            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('mass_alert_publisher')
    mass_pub = MassPublisher(sys.argv[1])
