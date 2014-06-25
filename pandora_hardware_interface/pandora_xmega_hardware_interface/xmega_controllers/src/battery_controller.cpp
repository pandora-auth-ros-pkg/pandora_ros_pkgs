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
* Author:  Evangelos Apostolidis
*********************************************************************/
#include "xmega_controllers/battery_controller.h"

namespace pandora_hardware_interface
{
namespace xmega
{
  bool BatteryController::init(
    BatteryInterface*
      batteryInterface,
    ros::NodeHandle& rootNodeHandle,
    ros::NodeHandle& controllerNodeHandle)
  {
    // Get sensor names from interface
    const std::vector<std::string>& batteryNames =
      batteryInterface->getNames();

    // Read publish rate from yaml
    if (!controllerNodeHandle.getParam("publish_rate", publishRate_))
    {
      publishRate_ = 50;
      ROS_ERROR(
        "Parameter 'publish_rate' not set in yaml, using default 50Hz");
    }

    for (int ii = 0; ii < batteryNames.size(); ii++)
    {
      // Get sensor handles from interface
      batteryHandles_.push_back(
        batteryInterface->getHandle(batteryNames[ii]));

      // Create publisher
      realtimePublisher_.reset(
        new realtime_tools::RealtimePublisher<
          pandora_xmega_hardware_interface::BatteryMsg>(
          rootNodeHandle, "/sensors/battery", 4));
    }

    return true;
  }

  void BatteryController::starting(const ros::Time& time)
  {
    // Initialize last time published
    lastTimePublished_ = time;
  }

  void BatteryController::update(
    const ros::Time& time, const ros::Duration& period)
  {
    // Publish messages

    if (lastTimePublished_ + ros::Duration(1.0/publishRate_) < time)
    {
      if (realtimePublisher_->trylock())
      {
        realtimePublisher_->msg_.name.clear();
        realtimePublisher_->msg_.voltage.clear();
        for (int ii = 0; ii < batteryHandles_.size(); ii++)
        {
          lastTimePublished_ =
            lastTimePublished_ + ros::Duration(1.0/publishRate_);

          // Fill voltage
          realtimePublisher_->msg_.name.push_back(
            batteryHandles_[ii].getName());
          double averageVoltage = 0;
          for (int jj = 0; jj < 5; jj++)
          {
            averageVoltage =
              averageVoltage + batteryHandles_[ii].getVoltage()[jj]/5;
          }
          realtimePublisher_->msg_.voltage.push_back(averageVoltage);
        }
        realtimePublisher_->unlockAndPublish();
      }
    }
  }

  void BatteryController::stopping(const ros::Time& time)
  {
  }

  BatteryController::BatteryController()
  {
  }

  BatteryController::~BatteryController()
  {
  }
}  // namespace xmega
}  // namespace pandora_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pandora_hardware_interface::xmega::BatteryController,
  controller_interface::ControllerBase)
