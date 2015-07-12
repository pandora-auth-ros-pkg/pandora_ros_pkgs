/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 *  "AS IS" AND ANY EXSS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef FRAME_MATCHER_MATCHER_PROCESSOR_H
#define FRAME_MATCHER_MATCHER_PROCESSOR_H

#include <string>

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>

#include "sensor_processor/handler.h"
#include "sensor_processor/processor.h"

#include "frame_matcher/points_on_frame.h"
#include "frame_matcher/roi_transformer.h"
#include "frame_matcher/view_pose_finder.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{
  /**
   * @class MatcherProcessor TODO
   */
  class MatcherProcessor : public sensor_processor::Processor<PointsOnFrame, PointsOnFrame>
  {
   public:
    /**
     * @brief Constructor
     * @param ns [const std::string&] The namespace of this postprocessor's nodeHandle
     * @param handler [sensor_processor::AbstractHandler*] A pointer of the class that
     * handles this postprocessor
     **/
    MatcherProcessor(const std::string& ns, sensor_processor::Handler* handler);
    virtual void
    initialize(const std::string& ns, sensor_processor::Handler* handler);
    MatcherProcessor();

    /**
     * @brief TODO
     * @param input [const pandora_vision_msgs::EnhancedImageConstPtr&] TODO
     * @param output [const PointsOnFramePtr&] TODO
     */
    virtual bool
    process(const PointsOnFrameConstPtr& input,
        const PointsOnFramePtr& output);

   private:
    void updateMap(const nav_msgs::OccupancyGridConstPtr& msg);
    void updateImageTo(const sensor_msgs::ImageConstPtr& msg);

   private:
    ros::NodeHandle nh_;
    ros::NodeHandle processor_nh_;
    std::string processorName_;

    RoiTransformerPtr roiTransformer_;
    ViewPoseFinderPtr viewPoseFinderPtr_;

    ros::Subscriber imageToSubscriber_;
    std::string imageToTopicName_;
    ros::Subscriber mapSubscriber_;
    std::string mapTopicName_;

    nav_msgs::OccupancyGridConstPtr mapConstPtr_;
    sensor_msgs::ImageConstPtr imageToConstPtr_;
  };
}  // namespace frame_matcher
}  // namespace pandora_data_fusion

#endif  // FRAME_MATCHER_MATCHER_PROCESSOR_H
