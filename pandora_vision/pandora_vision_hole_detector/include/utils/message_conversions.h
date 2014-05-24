/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef UTILS_MESSAGE_CONVERSIONS_H
#define UTILS_MESSAGE_CONVERSIONS_H

#include "utils/defines.h"
#include "utils/holes_conveyor.h"
#include "vision_communications/CandidateHolesVectorMsg.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class MessageConversions
    @brief Provides methods for converting images and point clouds
    from and to ROS messages
   **/
  class MessageConversions
  {
    public:

      /**
        @brief Converts a cv::Mat image into a sensor_msgs::Image message
        @param[in] image [const cv::Mat&] The image
        @param[in] encoding [const std::string&] The image message's encoding
        @param[in] msg [const sensor_msgs::Image&] A message needed for
        setting the output message's header by extracting its header
        @return [sensor_msgs::Image] The output image message
       **/
      static sensor_msgs::Image convertImageToMessage(
        const cv::Mat& image, const std::string& encoding,
        const sensor_msgs::Image& msg);

      /**
        @brief Extracts an image from a point cloud message
        @param pointCloud[in] [const PointCloudPtr&]
        The input point cloud message
        @param[in] id [const int&] The enconding of the converted image.
        CV_32FC1 for depth image, CV_8UC3 for rgb image
        @return cv::Mat The output image
       **/
      static cv::Mat convertPointCloudMessageToImage(
        const PointCloudPtr& pointCloud, const int& encoding);

      /**
        @brief Constructs a vision_communications/CandidateHolesVectorMsg
        message
        @param[in] conveyor [HolesConveyor&] A struct containing
        vectors of the holes' keypoints, bounding rectangles' vertices
        and blobs' outlines
        @param[in] image [cv::Mat&] The image to be packed in the message
        @param[out] candidateHolesVectorMsg
        [vision_communications::CandidateHolesVectorMsg*] The output message
        @param[in] encoding [std::string&] The image's encoding
        @param[in] msg [const sensor_msgs::Image&] Needed to extract
        its header and place it as the header of the output message
        @return void
       **/
      static void createCandidateHolesVectorMessage(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        vision_communications::CandidateHolesVectorMsg* candidateHolesVectorMsg,
        const std::string& encoding,
        const sensor_msgs::Image& msg);

      /**
        @brief Constructs a vision_communications/CandidateHolesVectorMsg
        message
        @param[in] conveyor [const HolesConveyor&] A struct containing
        vectors of the holes' keypoints, bounding rectangles' vertices
        and blobs' outlines
        @param[in] image [const sensor_msgs::Image&] The image to be packed
        in the message
        @param[out] candidateHolesVectorMsg
        [vision_communications::CandidateHolesVectorMsg*] The output message
        @param[in] msg [const sensor_msgs::Image&] Needed to extract
        its header and place it as the header of the output message
        @return void
       **/
      static void createCandidateHolesVectorMessage(
        const HolesConveyor& conveyor,
        const sensor_msgs::Image& image,
        vision_communications::CandidateHolesVectorMsg* candidateHolesVectorMsg,
        const sensor_msgs::Image& msg);

      /**
        @brief Constructs a vision_communications/CandidateHolesVectorMsg
        message
        @param[in] conveyor [HolesConveyor&] A struct containing
        vectors of the holes' keypoints, bounding rectangles' vertices
        and blobs' outlines
        @param[out] candidateHolesVector
        [std::vector<vision_communications::CandidateHolesVectorMsg>*]
        The vector containing the conveyor's holes in
        vision_communications::CandidateHolesVectorMsg format
        @return void
       **/
      static void createCandidateHolesVector(
        const HolesConveyor& conveyor,
        std::vector<vision_communications::CandidateHoleMsg>*
        candidateHolesVector);

      /**
        @brief Extracts a cv::Mat image from a ROS image message pointer
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS image
        message pointer
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessage(
        const sensor_msgs::ImageConstPtr& msg, cv::Mat* image,
        const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a ROS image message
        @param[in] msg [const sensor_msgs::Image&] The input ROS image
        message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessage(
        const sensor_msgs::Image& msg, cv::Mat* image,
        const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a custom ROS message  of type
        vision_communications::CandidateHolesVectorMsg
        containing the interpolated depth image
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessageContainer(
        const vision_communications::CandidateHolesVectorMsg& msg,
        cv::Mat* image, const std::string& encoding);

  };

} // namespace pandora_vision

#endif  // UTILS_MESSAGE_CONVERSIONS_H
