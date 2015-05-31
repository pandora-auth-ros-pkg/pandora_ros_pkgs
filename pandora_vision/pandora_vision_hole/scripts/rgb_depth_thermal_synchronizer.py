#!/usr/bin/env python
#Software License Agreement
__version__ = "0.0.1"
__status__ = "Production"
__license__ = "BSD"
__copyright__ = "Copyright (c) 2015, P.A.N.D.O.R.A. Team. All rights reserved."
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of P.A.N.D.O.R.A. Team nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
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
__author__ = "Triantafyllidis Angelos"
__maintainer__ = "Triantafyllidis Angelos"
__email__ = "aggelostriadafillidis@gmail.com"

PKG  =  'pandora_vision_hole'
NAME  =  'rgb_depth_thermal_synchronizer'

import message_filters
from camera_calibration import approxsync
from sensor_msgs.msg import Image, PointCloud2
import rospy
from pandora_vision_msgs.msg import SynchronizedMsg

# Here the synchronized messages from kinect and flir are sent
# to the synchronizer node as one united message.
def callback(pointCloud, image):

    ns = rospy.get_namespace()

    # Check if topic for synchronized message has been given properly if yes pass it to the variable
    if (rospy.has_param(ns + "/rgb_depth_thermal_synchronizer_node/published_topics/synchronized_topic")):
        synch_topic = rospy.get_param(ns + "/rgb_depth_thermal_synchronizer_node/published_topics/synchronized_topic")

        # Make topic's name absolute
        synch_topic = ns + synch_topic
        #rospy.loginfo("[Rgbdt_synchronizer] is publishing to: %s", synch_topic)
    else:
        rospy.logerr( "No synchronized topic found")
        rospy.signal_shutdown("shutdown RGBDT synchronizer")

    # Publisher of PointCloud2 and Image messages
    synch_pub = rospy.Publisher(synch_topic, SynchronizedMsg)

    # Pack the message to be sent.
    msg = SynchronizedMsg()
    msg.pc = pointCloud
    msg.thermalInfo = image

    # Publish the message to the synchronizer node
    synch_pub.publish(msg)


if __name__=='__main__':

    # Initialization of rgbdt_synchronizer node
    rospy.init_node('rgbdt_synchronizer')
    ns = rospy.get_namespace()

    # Check if kinect topic has been given properly if yes pass it to the variable
    if (rospy.has_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_topic")):
        kinect_topic = rospy.get_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_topic")
        rospy.loginfo("[Rgbdt_synchronizer] is subscribed to: %s", kinect_topic)
    else:
        rospy.logerr( "No point cloud topic found")
        rospy.signal_shutdown("shutdown RGBDT synchronizer")


    # Check if flir topic has been given properly if yes pass it to the variable
    if (rospy.has_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_thermal_topic")):
        flir_topic = rospy.get_param(ns + "/rgb_depth_thermal_synchronizer_node/subscribed_topics/input_thermal_topic")
        rospy.loginfo("[Rgbdt_synchronizer] is subscribed to: %s", flir_topic)
    else:
        rospy.logerr("No flir topic found")
        rospy.signal_shutdown("shutdown RGBDT synchronizer")

    # Subscribers of kinect and flir messages
    kinect_subscriber = message_filters.Subscriber(kinect_topic, PointCloud2)
    flir_subscriber = message_filters.Subscriber(flir_topic, Image)

    # Synchronize kinect and flir topics
    sync = approxsync.ApproximateSynchronizer(0.5, [kinect_subscriber, flir_subscriber], 5000)
    sync.registerCallback(callback)

    rospy.spin()
