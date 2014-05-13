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

#ifndef SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H
#define SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H

#include "utils/message_conversions.h"
#include <utils/defines.h>
#include <utils/parameters.h>
#include <std_msgs/Empty.h>

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class RgbDepthSynchronizer
    @brief Responsible for the dissemination of the Depth and RGB images to
    their respective responsible nodes, extracted from a coloured point cloud.
   **/
  class RgbDepthSynchronizer
  {
    private:

      // The ROS node handle
      ros::NodeHandle nodeHandle_;

      // The subscriber to the point cloud topic
      ros::Subscriber pointCloudSubscriber_;

      // The publishers which will advertise the
      // synchronized point cloud, depth and rgb images extracted from the
      // point cloud;
      ros::Publisher synchronizedPointCloudPublisher_;
      ros::Publisher synchronizedDepthImagePublisher_;
      ros::Publisher synchronizedRGBImagePublisher_;

      // A boolean indicating whether the node is publishing through the
      // above two publishers
      bool isLocked_;

      // The subscriber to the topic where the hole_fusion node publishes
      // lock/unlock messages concerning the rgb_depth_synchronizer's
      // behaviour
      ros::Subscriber holeFusionSubscriber_;

      // Records the time for each synchronizer invocation
      double invocationTime_;

      // Mean invocation interval of time
      double meanProcessingTime_;

      // Amount of synchronizer's invocations
      int ticks_;

      /**
        @brief The synchronized callback for the point cloud
        obtained by the depth sensor.
        @param[in] pointCloudMessage [const sensor_msgs::PointCloud2ConstPtr&]
        The input point cloud
        @return void
       **/
      void synchronizedCallback(
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage);

      /**
        @brief The callback for the hole_fusion node request for the
        lock/unlock of the rgb_depth_synchronizer node
        @param[in] lockMsg [const std_msgs::Empty] An empty message
        @return void
       **/
      void holeFusionCallback(const std_msgs::Empty& lockMsg);


    public:

      /**
        @brief The constructor
       **/
      RgbDepthSynchronizer(void);

      /**
        @brief The default constructor
       **/
      ~RgbDepthSynchronizer(void);
  };

} // namespace pandora_vision

#endif  // SYNCHRONIZER_NODE_RGB_DEPTH_SYNCHRONIZER_H
