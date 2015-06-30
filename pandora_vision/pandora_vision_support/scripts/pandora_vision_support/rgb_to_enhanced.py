#!/usr/bin/env python
# encoding: utf-8
# Software License Agreement
__version__ = "0.0.1"
__status__ = "Development"
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
__authors__ = "Christos Tsirigotis"
__maintainer__ = "Christos Tsirigotis"
__email__ = "tsirif@gmail.com"

import os
import sys

import rosbag

from collections import defaultdict

from sensor_msgs.msg import Image
from pandora_vision_msgs.msg import EnhancedImage
from pandora_vision_msgs.msg import RegionOfInterest


def convert(input_bag_name, topic, annotation_file, output_bag_name="output.bag"):
    """ @brief: Converts a bag which contains a topic of sensor_msgs/Image to a
        bag which contains the respective pandora_vision_msgs/EnhancedImage
        given a annotation file

    @param input_bag_name: bag with sensor_msgs/Image of rgb channel
    @type input_bag_name: str
    @param topic: topic to convert
    @type topic: str
    @param annotation_file: path to annotation file
    @type annotation_file: str
    @param output_bag_name: result's name
    @type output_bag_name: str
    @return: nothing

    """
    with rosbag.Bag(output_bag_name, 'w') as output_bag:
        with rosbag.Bag(input_bag_name, 'r') as input_bag:
            annotation_dict = read_annotation_file(annotation_file)
            i = 0
            for topic_name, image, time in input_bag.read_messages():
                if topic_name == topic:
                    i += 1
                    frame_name = "frame"+str(i)+".png"
                    if frame_name not in annotation_dict:
                        continue
                    enhanced_image = convert_image(image,
                                                   frame_name, annotation_dict)
                    output_bag.write("/enhanced_images", enhanced_image, time)
                else:
                    output_bag.write(topic_name, image, time)


def convert_image(image, frame_name, annotation_dict):
    """ @brief: Converts an image of rgb channel to EnhancedImage given its
        region of interest with the annotation_dict

    @param image: image to be converted
    @type image: sensor_msgs/Image
    @param frame_name: name of this frame
    @rtype frame_name: string
    @param annotation_dict: dictionary containing regions of interest
    @type annotation_dict: defaultdict of tuples
    @return: converted image
    @rtype: pandora_vision_msgs/EnhancedImage

    """
    enhanced_image = EnhancedImage()
    enhanced_image.isDepth = False
    enhanced_image.rgbImage = image
    annotation = annotation_dict[frame_name]
    region_of_interest = annotation_to_roi(annotation)
    enhanced_image.regionsOfInterest.append(region_of_interest)
    enhanced_image.header = image.header
    return enhanced_image


def annotation_to_roi(annotation):
    """ @brief: Convert an annotation entry to ROI msg

    @param annotation: annotation from pandora_rqt_annotator
    @type annotation: tuple
    @return: corresponding message
    @rtype: pandora_vision_msgs.msg.RegionOfInterest

    """
    region_of_interest = RegionOfInterest()
    width = annotation[3] - annotation[1]
    height = annotation[4] - annotation[2]
    center_x = (annotation[1] + annotation[3]) / 2
    center_y = (annotation[2] + annotation[4]) / 2
    region_of_interest.center.x = center_x
    region_of_interest.center.y = center_y
    region_of_interest.height = height
    region_of_interest.width = width
    return region_of_interest


def read_annotation_file(annotation_file):
    """ @brief: Creates a dictionary from the annotation file given

    @param annotation_file: path to annotation file
    @type annotation_file: string
    @return: dictionary of frame names to regions of interest
    @rtype: defaultdict of tuples

    """
    if not os.path.isfile(annotation_file):
        print "ERROR : Incorrect Path for annotator file."
        print annotation_file
        exit(2)
    annotations_dict = defaultdict(tuple)
    with open(annotation_file, "rU") as annotator_file:
        for line in annotator_file:
            if line.startswith("#"):
                continue
            (frame_name, object_type, x_top_left_point, y_top_left_point,
             x_bottom_right_point, y_bottom_right_point) = line.split(",")
            y_bottom_right_point = y_bottom_right_point.replace("\n", "")
            annotations_dict[frame_name] = (object_type,
                                            int(x_top_left_point),
                                            int(y_top_left_point),
                                            int(x_bottom_right_point),
                                            int(y_bottom_right_point))
    return annotations_dict


if __name__ == '__main__':
    ARGS = sys.argv[1:]
    if len(ARGS) < 3:
        print "./rgb_to_enhanced.py [-O output_bag_name] input_bag_name "+\
               "topic_name annotation_file_name"
        exit(1)
    if ARGS[0] == '-O':
        if len(ARGS) < 5:
            print "./rgb_to_enhanced.py [-O output_bag_name] input_bag_name "+\
                  "topic_name annotation_file_name"
            exit(1)
        OUTBAG_NAME = ARGS[1]
        INBAG_NAME = ARGS[2]
        TOPIC = ARGS[3]
        ANNOTATION_NAME = ARGS[4]
        convert(INBAG_NAME, TOPIC, ANNOTATION_NAME, OUTBAG_NAME)
    else:
        INBAG_NAME = ARGS[0]
        TOPIC = ARGS[1]
        ANNOTATION_NAME = ARGS[2]
        convert(INBAG_NAME, TOPIC, ANNOTATION_NAME)
