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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef FRAME_MATCHER_KEYPOINT_TRANSFORMER_H
#define FRAME_MATCHER_KEYPOINT_TRANSFORMER_H

#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>

#include "pandora_vision_common/pandora_vision_utilities/general_alert_converter.h"
#include "pandora_vision_common/poi_stamped.h"
#include "pandora_common_msgs/GeneralAlert.h"

#include "frame_matcher/view_pose_finder.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{

  /**
   * @class KeypointTransformer TODO
   */
  class KeypointTransformer
  {
   public:
    KeypointTransformer(const ros::NodeHandle& nh, const ViewPoseFinderPtr& viewPoseFinderPtr);
    virtual
    ~KeypointTransformer();

    cv::Point2f
    transformKeypoint(const sensor_msgs::Image& imageFrom,
                      const cv::Point2f& pointFrom,
                      const sensor_msgs::Image& imageTo);

   private:
    ros::NodeHandle nh_;

    ViewPoseFinderPtr viewPoseFinderPtr_;
    pandora_vision::GeneralAlertConverter generalAlertConverter_;

    std::string global_frame_;
  };

  typedef boost::scoped_ptr<KeypointTransformer> KeypointTransformerPtr;

}  // namespace frame_matcher
}  // namespace pandora_data_fusion

#endif  // FRAME_MATCHER_KEYPOINT_TRANSFORMER_H
