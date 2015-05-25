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
 * Authors: 
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>

#include "sensor_coverage/coverage_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    CoverageChecker::CoverageChecker(const NodeHandlePtr& nh, const std::string& frameName)
      : nh_(nh), frameName_(frameName)
    {}

    boost::shared_ptr<octomap::OcTree> CoverageChecker::map3d_;
    nav_msgs::OccupancyGridPtr CoverageChecker::map2d_;
    double CoverageChecker::OCCUPIED_CELL_THRES = 0.5;
    double CoverageChecker::MAX_HEIGHT = 0;
    double CoverageChecker::FOOTPRINT_WIDTH = 0;
    double CoverageChecker::FOOTPRINT_HEIGHT = 0;

    void CoverageChecker::findCoverage(
        const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      sensorTransform.getBasis().getRPY(sensorRoll_, sensorPitch_, sensorYaw_);
      sensorPosition_ = octomap::pointMsgToOctomap(
          Utils::vector3ToPoint(sensorTransform.getOrigin()));
      baseTransform.getBasis().getRPY(robotRoll_, robotPitch_, robotYaw_);
      robotPosition_ = octomap::pointMsgToOctomap(
          Utils::vector3ToPoint(baseTransform.getOrigin()));
    }

    void CoverageChecker::getParameters()
    {
      if (!nh_->getParam(frameName_+"/sensor_range", SENSOR_RANGE))
      {
        ROS_FATAL("%s sensor range param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam(frameName_+"/sensor_hfov", SENSOR_HFOV))
      {
        ROS_FATAL("%s sensor hfov param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam(frameName_+"/sensor_vfov", SENSOR_VFOV))
      {
        ROS_FATAL("%s sensor vfov param not found", frameName_.c_str());
        ROS_BREAK();
      }
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

