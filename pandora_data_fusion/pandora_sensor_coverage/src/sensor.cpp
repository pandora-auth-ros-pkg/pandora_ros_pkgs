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

    Sensor::Sensor(const NodeHandlePtr& nh, const OctomapPtr& globalMap,
            const std::string& frameName, const std::string& mapOrigin)
      : nh_(nh), globalMap_(globalMap), frameName_(frameName)
    {
      sensorWorking_ = false;

      listener_.reset(TfFinder::newTfListener(mapOrigin));

      std::string topic;

      if (nh_->getParam("published_topic_names/"+frameName_, topic))
      {
        coveragePublisher_ = nh_->advertise<octomap_msgs::Octomap>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s topic name param not found", frameName_.c_str());
        ROS_BREAK();
      }

      getParameters();

      coverageUpdater_ = nh_->createTimer(ros::Duration(0.1),
          &Sensor::coverageUpdate, this);
    }

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
      //  If it does, fetch current transformation.
      try
      {
        ros::Time timeNow = ros::Time::now();
        listener_->waitForTransform(
            "/map", frameName_, timeNow, ros::Duration(0.5));
        listener_->lookupTransform(
            "/map", frameName_, timeNow, tfTransform_);
      }
      catch (TfException ex)
      {
        ROS_WARN_NAMED("SENSOR_COVERAGE",
            "[SENSOR_COVERAGE_SENSOR %d] %s", __LINE__, ex.what());
      }
      //  Update coverage patch.
      patchDrawer();
      //  Publish fresh patch.
      coveragePublisher_.publish(coveragePatch_);
    }

    void Sensor::patchDrawer()
    {
    }

    void Sensor::getParameters()
    {
      if (!nh_->getParam("sensor_range/"+frameName_, SENSOR_RANGE))
      {
        ROS_FATAL("%s sensor range param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam("sensor_hfov/"+frameName_, SENSOR_HFOV))
      {
        ROS_FATAL("%s sensor hfov param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam("sensor_vfov/"+frameName_, SENSOR_VFOV))
      {
        ROS_FATAL("%s sensor vfov param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam("exploration_state/"+frameName_, EXPLORATION_STATE))
      {
        ROS_FATAL("%s exploration state param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam("identification_state/"+frameName_, IDENTIFICATION_STATE))
      {
        ROS_FATAL("%s identification state param not found", frameName_.c_str());
        ROS_BREAK();
      }
      if (!nh_->getParam("hold_state/"+frameName_, HOLD_STATE))
      {
        ROS_FATAL("%s hold state param not found", frameName_.c_str());
        ROS_BREAK();
      }
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

