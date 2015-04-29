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
* Author:  George Kouros
*********************************************************************/

#include "imu_controllers/imu_rpy_controller.h"

namespace pandora_hardware_interface
{
namespace imu
{
  ImuRPYController::ImuRPYController()
  {
  }

  ImuRPYController::~ImuRPYController()
  {
  }

  bool ImuRPYController::init(
    pandora_hardware_interface::imu::ImuRPYInterface* hw,
    ros::NodeHandle& rootNh,
    ros::NodeHandle& controllerNh)
  {
    // get all joint states from the hardware interface
    const std::vector<std::string>& sensorNames = hw->getNames();
    for (unsigned ii = 0; ii < sensorNames.size(); ii++)
      ROS_DEBUG("Got sensor %s", sensorNames[ii].c_str());

    // get publishing period
    if (!controllerNh.getParam("publish_rate", publishRate_))
    {
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    for (unsigned ii = 0; ii < sensorNames.size(); ii++)
    {
      // sensor handle
      sensors_.push_back(hw->getHandle(sensorNames[ii]));

      // Roll, Pitch, Yaw realtime publisher
      RtPublisherPtr rtPub(
        new realtime_tools::RealtimePublisher<pandora_sensor_msgs::ImuRPY>(
          rootNh,
          sensorNames[ii],
          4));
      realtimePubs_.push_back(rtPub);
    }

    // Last published times
    lastPublishTimes_.resize(sensorNames.size());
    return true;
  }

  void ImuRPYController::starting(const ros::Time& time)
  {
    // initialize time
    for (unsigned ii = 0; ii < lastPublishTimes_.size(); ii++){
      lastPublishTimes_[ii] = time;
    }
  }

  void ImuRPYController::update(
    const ros::Time& time,
    const ros::Duration& period)
  {
    // limit rate of publishing
    for (unsigned ii = 0; ii < realtimePubs_.size(); ii++){
      if (publishRate_ > 0.0 &&
        lastPublishTimes_[ii] + ros::Duration(1.0 / publishRate_) < time)
      {
        // try to publish
        if (realtimePubs_[ii]->trylock())
        {
          // we're actually publishing, so increment time
          lastPublishTimes_[ii] =
            lastPublishTimes_[ii] + ros::Duration(1.0 / publishRate_);

          // populate message
          realtimePubs_[ii]->msg_.header.stamp = time;
          realtimePubs_[ii]->msg_.header.frame_id = sensors_[ii].getFrameId();

          realtimePubs_[ii]->msg_.roll = *(sensors_[ii].getRoll());
          realtimePubs_[ii]->msg_.pitch = *(sensors_[ii].getPitch());
          realtimePubs_[ii]->msg_.yaw = *(sensors_[ii].getYaw());

          realtimePubs_[ii]->unlockAndPublish();
        }
      }
    }
  }

  void ImuRPYController::stopping(const ros::Time& time)
  {
  }
}  // namespace imu
}  // namespace pandora_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pandora_hardware_interface::imu::ImuRPYController,
  controller_interface::ControllerBase)
