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

#ifndef DEPTH_NODE_DEPTH_H
#define DEPTH_NODE_DEPTH_H

#include "depth_node/hole_detector.h"
#include "utils/parameters.h"
#include "utils/message_conversions.h"
#include "utils/wavelets.h"
#include "pandora_vision_msgs/CandidateHolesVectorMsg.h"


/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class Depth
    @brief Provides functionalities for locating holes via
    analysis of a depth image
   **/
  class Depth
  {
    private:
      // The ROS node handle
      ros::NodeHandle nodeHandle_;

      // Subscriber of Kinect point cloud
      ros::Subscriber depthImageSubscriber_;

      // The name of the topic where the depth image is acquired from
      std::string depthImageTopic_;

      // ROS publisher for the candidate holes
      ros::Publisher candidateHolesPublisher_;

      // The name of the topic where the candidate holes that the depth node
      // locates are published to
      std::string candidateHolesTopic_;

      // The dynamic reconfigure (depth) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::depth_cfgConfig>
        server;

      // The dynamic reconfigure (depth) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_detector::depth_cfgConfig>
        ::CallbackType f;

      /**
        @brief Callback for the depth image received by the synchronizer node.

        The depth image message received by the synchronizer node is unpacked
        in a cv::Mat image and stripped of its noise.
        Holes are then located inside this image and information about them,
        along with the denoised image, is then sent to the hole fusion node
        @param msg [const sensor_msgs::Image&] The depth image message
        @return void
       **/
      void inputDepthImageCallback(const sensor_msgs::Image& msg);

      /**
        @brief Acquires topics' names needed to be subscribed to and advertise
        to by the depth node
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole_detector::depth_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallback(
        const pandora_vision_hole_detector::depth_cfgConfig& config,
        const uint32_t& level);


    public:

      /**
        @brief Default constructor. Initiates communications, loads parameters.
        @return void
       **/
      Depth(void);

      /**
        @brief Default destructor
        @return void
       **/
      ~Depth(void);

  };

} // namespace pandora_vision

#endif  // DEPTH_NODE_DEPTH_H
