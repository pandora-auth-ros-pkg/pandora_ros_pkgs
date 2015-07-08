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

#ifndef PANDORA_VISION_HOLE_RGB_NODE_UTILS_MESSAGE_CONVERSIONS_H
#define PANDORA_VISION_HOLE_RGB_NODE_UTILS_MESSAGE_CONVERSIONS_H

#include <vector>
#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>

#include "pandora_vision_hole/CandidateHolesVectorMsg.h"
#include "rgb_node/utils/defines.h"
#include "rgb_node/utils/outline_discovery.h"
#include "rgb_node/utils/holes_conveyor.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace rgb
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
        const PointCloudConstPtr& pointCloud, int encoding);

      static void toROSDepthMsg(
        const sensor_msgs::PointCloud2& cloud, sensor_msgs::Image& msg);

      /**
        @brief Constructs a pandora_vision_msgs/CandidateHolesVectorMsg
        message
        @param[in] conveyor [HolesConveyor&] A struct containing
        vectors of the holes' keypoints, bounding rectangles' vertices
        and blobs' outlines
        @param[out] candidateHolesVector
        [std::vector<::::pandora_vision_hole::CandidateHolesVectorMsg>*]
        The vector containing the conveyor's holes in
        pandora_vision_msgs::CandidateHolesVectorMsg format
        @return void
       **/
      static void createCandidateHolesVector(
        const HolesConveyor& conveyor,
        std::vector< ::pandora_vision_hole::CandidateHoleMsg >*
        candidateHolesVector);

      /**
        @brief Constructs a pandora_vision_msgs/CandidateHolesVectorMsg
        message
        @param[in] conveyor [HolesConveyor&] A struct containing
        vectors of the holes' keypoints, bounding rectangles' vertices
        and blobs' outlines
        @param[in] image [cv::Mat&] The image to be packed in the message
        @param[out] candidateHolesVectorMsg
        [::::pandora_vision_hole::CandidateHolesVectorMsg*] The output message
        @param[in] encoding [std::string&] The image's encoding
        @param[in] msg [const sensor_msgs::Image&] Needed to extract
        its header and place it as the header of the output message
        @return void
       **/
      static void createCandidateHolesVectorMessage(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const ::pandora_vision_hole::CandidateHolesVectorMsgPtr& candidateHolesVectorMsg,
        const std::string& encoding,
        const sensor_msgs::Image& msg);

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
        ::::pandora_vision_hole::CandidateHolesVectorMsg
        containing the interpolated depth image
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessageContainer(
        const ::pandora_vision_hole::CandidateHolesVectorMsg& msg,
        cv::Mat* image, const std::string& encoding);

      /**
        @brief Recreates the HolesConveyor struct for the candidate holes
        from the pandora_vision_msgs::CandidateHolerMsg message
        @param[in] candidateHolesVector
        [const std::vector<::pandora_vision_hole::CandidateHoleMsg>&]
        The input candidate holes
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[in] inImage [const cv::Mat&] An image used for its size.
        It is needed if the wavelet method is used in the keypoints' extraction,
        in order to obtain the coherent shape of holes' outline points
        @param[in] representationMethod [const int&] The @param inImage
        representation method. 0 for normal mode, 1 for wavelet mode
        @param[in] raycastKeypointPartitions [const int&] The number of rays
        used, if @param representationMethod = 1, in order to recreate a
        blob's outline
        @return void
       **/
      static void fromCandidateHoleMsgToConveyor(
        const std::vector< ::pandora_vision_hole::CandidateHoleMsg >&
        candidateHolesVector,
        HolesConveyor* conveyor,
        const cv::Mat& inImage,
        const int& representationMethod,
        const int& raycastKeypointPartitions);

      /**
        @brief Unpacks the the HolesConveyor struct for the
        candidate holes, the interpolated depth image or the RGB image
        from the ::::pandora_vision_hole::CandidateHolesVectorMsg message
        @param[in] holesMsg
        [pandora_vision_msgs::CandidateHolesVectorMsg&] The input
        candidate holes message obtained through the depth node
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[out] image [cv::Mat*] The output image
        @param[in] representationMethod [const int&] The @param inImage
        representation method. 0 for normal mode, 1 for wavelet mode
        @param[in] encoding [const std::string&] The encoding used for
        @param[in] raycastKeypointPartitions [const int&] The number of rays
        used, if @param representationMethod = 1, in order to recreate a
        blob's outline
        @return void
       **/
      static void unpackMessage(
        const ::pandora_vision_hole::CandidateHolesVectorMsg& holesMsg,
        HolesConveyor* conveyor,
        cv::Mat* image,
        const int& representationMethod,
        const std::string& encoding,
        const int& raycastKeypointPartitions);

      /**
       @brief Convert the Float32MultiArray data to cv::Mat.
       Its cv format  will be CV_8UC1.
       @param[in] inArray [const std_msgs::Float32MultiArray&]
       The input MultiArray
       @return cv::Mat
       **/
      static cv::Mat convertFloat32MultiArrayToMat(
        const std_msgs::Float32MultiArray& inArray);
  };

}  // namespace rgb
}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_RGB_NODE_UTILS_MESSAGE_CONVERSIONS_H
