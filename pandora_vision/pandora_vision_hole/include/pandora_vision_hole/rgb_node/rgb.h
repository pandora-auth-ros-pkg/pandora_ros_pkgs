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
 * Authors: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_RGB_NODE_RGB_H
#define PANDORA_VISION_HOLE_RGB_NODE_RGB_H

#include "pandora_vision_hole/CandidateHolesVectorMsg.h"
#include "utils/message_conversions.h"
#include "utils/histogram.h"
#include "utils/parameters.h"
#include "utils/wavelets.h"
#include "rgb_node/hole_detector.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  /**
    @class Rgb
    @brief Provides functionalities for locating holes via
    analysis of a RGB image
   **/
  class Rgb
  {
    private:
      // The NodeHandle
      ros::NodeHandle nodeHandle_;

      // The ROS subscriber for acquisition of the RGB image through the
      // depth sensor
      ros::Subscriber rgbImageSubscriber_;

      // The name of the topic where the rgb image is acquired from
      std::string rgbImageTopic_;

      // The ROS publisher ofcandidate holes
      ros::Publisher candidateHolesPublisher_;

      // The name of the topic where the candidate holes that the rgb node
      // locates are published to
      std::string candidateHolesTopic_;

      // A vector of histograms for the texture of walls
      std::vector<cv::MatND> wallsHistogram_;

      // The dynamic reconfigure (RGB) parameters' server
      dynamic_reconfigure::Server< ::pandora_vision_hole::rgb_cfgConfig >
        server;

      // The dynamic reconfigure (RGB) parameters' callback
      dynamic_reconfigure::Server< ::pandora_vision_hole::rgb_cfgConfig >::
        CallbackType f;

      /**
        @brief Callback for the rgb image received by the synchronizer node.

        The rgb image message received by the synchronizer node is unpacked
        in a cv::Mat image. Holes are then located inside this image and
        information about them, along with the rgb image, is then sent to the
        hole fusion node
        @param msg [const sensor_msgs::Image&] The rgb image message
        @return void
       **/
      void inputRgbImageCallback(const sensor_msgs::Image& inImage);

      /**
        @brief Acquires topics' names needed to be subscribed to and advertise
        to by the rgb node
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole::rgb_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallback(
        const ::pandora_vision_hole::rgb_cfgConfig& config,
        const uint32_t& level);


    public:
      // The constructor
      Rgb();

      // The destructor
      ~Rgb();
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_RGB_NODE_RGB_H
