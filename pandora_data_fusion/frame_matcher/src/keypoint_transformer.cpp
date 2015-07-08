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

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Point.h>

#include "pandora_vision_common/pandora_vision_utilities/general_alert_converter.h"
#include "pandora_vision_common/poi_stamped.h"
#include "pandora_common_msgs/GeneralAlert.h"

#include "frame_matcher/keypoint_transformer.h"
#include "frame_matcher/view_pose_finder.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{

  KeypointTransformer::
  KeypointTransformer(const ros::NodeHandle& nh, const ViewPoseFinderPtr& viewPoseFinderPtr) :
    nh_(nh), viewPoseFinderPtr_(viewPoseFinderPtr)
  {
    nh.param<std::string>("global_frame", global_frame_, "/map");
  }

  KeypointTransformer::
  ~KeypointTransformer() {}

  cv::Point2f
  KeypointTransformer::
  transformKeypoint(
      const sensor_msgs::Image& imageFrom,
      const cv::Point2f& pointFrom,
      const sensor_msgs::Image& imageTo)
  {
    // #1 Calculate yaw and pitch from origin camera frame towards the point we
    // want to transform
    pandora_common_msgs::GeneralAlert originCameraAlert;
    pandora_vision::POIStamped poiFrom;
    poiFrom.header = imageFrom.header;
    poiFrom.point = pointFrom;
    originCameraAlert = generalAlertConverter_.getGeneralAlert(nh_, poiFrom,
        imageFrom.width, imageFrom.height);

    // #2 Calculate position of the point we want to transform in the world
    tf::Transform originCameraFrame = viewPoseFinderPtr_->lookupTransformFromWorld(
        global_frame_, originCameraAlert.header);
    geometry_msgs::Point pointInWorld = viewPoseFinderPtr_->findAlertPosition(
        originCameraAlert.info.yaw, originCameraAlert.info.pitch, originCameraFrame);

    // #3 Calculate yaw and pitch from target camera frame towards the point we
    // want to transform
    pandora_common_msgs::GeneralAlert targetCameraAlert;
    targetCameraAlert.header.frame_id = generalAlertConverter_.findParentFrameId(nh_,
        imageTo.header.frame_id, "/robot_description");
    targetCameraAlert.header.stamp = imageTo.header.stamp;  // change this later
    tf::Transform targetCameraFrame = viewPoseFinderPtr_->lookupTransformFromWorld(
        global_frame_, targetCameraAlert.header);
    viewPoseFinderPtr_->findViewOrientation(pointInWorld, targetCameraFrame,
        &targetCameraAlert.info.yaw, &targetCameraAlert.info.pitch);

    // #4 Calculate point on target camera frame on which we see the same object
    // in the world
    pandora_vision::POI poiOnTargetCamera = generalAlertConverter_.getPOI(nh_,
        targetCameraAlert, imageTo.width, imageTo.height);
    return poiOnTargetCamera.getPoint();
  }

}  // namespace frame_matcher
}  // namespace pandora_data_fusion
