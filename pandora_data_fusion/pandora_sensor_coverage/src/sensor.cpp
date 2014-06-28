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

#include "sensor_coverage/sensor.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    Sensor::Sensor(const NodeHandlePtr& nh,
        const std::string& frameName, const std::string& mapOrigin)
      : nh_(nh), frameName_(frameName)
    {
      if (!nh_->getParam(frameName_+"/produces_surface_coverage", surfaceCoverage_))
      {
        ROS_FATAL("%s produces surface coverage param not found", frameName_.c_str());
        ROS_BREAK();
      }
      spaceChecker_.reset( new SpaceChecker(nh_, frameName_) );
      if (surfaceCoverage_)
      {
        surfaceChecker_.reset( new SurfaceChecker(nh_, frameName_) );
        spaceChecker_->setCoverageMap3d(
            boost::dynamic_pointer_cast<octomap::OcTree>(surfaceChecker_->getCoverageMap3d()));
      }
      sensorWorking_ = false;
      listener_.reset(TfFinder::newTfListener(mapOrigin));
      getParameters();
      coverageUpdater_ = nh_->createTimer(ros::Duration(0.1),
          &Sensor::coverageUpdate, this);
    }

    boost::shared_ptr<octomap::OcTree> Sensor::map3d_ = boost::shared_ptr<octomap::OcTree>();
    nav_msgs::OccupancyGridPtr Sensor::map2d_;
    std::string Sensor::GLOBAL_FRAME;
    std::string Sensor::ROBOT_BASE_FRAME;

    void Sensor::notifyStateChange(int newState)
    {
      switch (newState)
      {
        case state_manager_communications::robotModeMsg::MODE_EXPLORATION:
          sensorWorking_ = EXPLORATION_STATE;
          break;
        case state_manager_communications::robotModeMsg::MODE_IDENTIFICATION:
          sensorWorking_ = IDENTIFICATION_STATE;
          break;
        case state_manager_communications::robotModeMsg::MODE_DF_HOLD:
          sensorWorking_ = HOLD_STATE;
          break;
        case state_manager_communications::robotModeMsg::MODE_ARM_APPROACH:
          sensorWorking_ = HOLD_STATE;
          break;
        default:
          sensorWorking_ = false;
          break;
      }
    }

    void Sensor::coverageUpdate(const ros::TimerEvent& event)
    {
      //  If sensor is not open and working, do not update coverage patch.
      if (!sensorWorking_)
        return;
      if (map2d_->data.size() == 0 || map3d_.get() == NULL)
        return;
      //  If it does, fetch current transformation.
      ros::Time timeNow = ros::Time::now();
      tf::StampedTransform sensorTransform, baseTransform;
      try
      {
        listener_->waitForTransform(
            GLOBAL_FRAME, frameName_, timeNow, ros::Duration(0.5));
        listener_->lookupTransform(
            GLOBAL_FRAME, frameName_, timeNow, sensorTransform);
        listener_->waitForTransform(
            GLOBAL_FRAME, ROBOT_BASE_FRAME, timeNow, ros::Duration(0.5));
        listener_->lookupTransform(
            GLOBAL_FRAME, ROBOT_BASE_FRAME, timeNow, baseTransform);
        //  Update coverage perception.
        //  Publish updated coverage perception.
        if (surfaceCoverage_)
        {
          surfaceChecker_->findCoverage(sensorTransform);
          surfaceChecker_->publishCoverage();
        }
        spaceChecker_->findCoverage(sensorTransform, baseTransform);
        spaceChecker_->publishCoverage();
      }
      catch (TfException ex)
      {
        ROS_WARN_NAMED("SENSOR_COVERAGE",
            "[SENSOR_COVERAGE_SENSOR %d] %s", __LINE__, ex.what());
      }
    }

    void Sensor::getParameters()
    {
      if (!nh_->getParam(frameName_+"/exploration_state", EXPLORATION_STATE))
      {
        ROS_FATAL("%s exploration state param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam(frameName_+"/identification_state", IDENTIFICATION_STATE))
      {
        ROS_FATAL("%s identification state param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam(frameName_+"/hold_state", HOLD_STATE))
      {
        ROS_FATAL("%s hold state param not found", frameName_.c_str());
        ROS_BREAK();
      }
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

