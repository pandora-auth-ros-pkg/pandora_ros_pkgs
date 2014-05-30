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
#include <arm_controllers/co2_sensor_controller.h>

namespace pandora_hardware_interface
{
namespace arm
{
  bool Co2SensorController::init(
    Co2SensorInterface*
      co2SensorInterface,
    ros::NodeHandle& rootNodeHandle,
    ros::NodeHandle& controllerNodeHandle)
  {
    // Get sensor names from interface
    const std::vector<std::string>& co2SensorNames =
      co2SensorInterface->getNames();

    // Read publish rate from yaml
    if (!controllerNodeHandle.getParam("publish_rate", publishRate_))
    {
      publishRate_ = 50;
      ROS_ERROR(
        "Parameter 'publish_rate' not set in yaml, using default 50Hz");
    }

    for (int ii = 0; ii < co2SensorNames.size(); ii++)
    {
      // Get sensor handles from interface
      sensorHandles_.push_back(
        co2SensorInterface->getHandle(co2SensorNames[ii]));

      // Create publisher for each controller
      Co2RealtimePublisher publisher(
        new realtime_tools::RealtimePublisher<
          pandora_arm_hardware_interface::Co2Msg>(
            rootNodeHandle, "/sensors/co2", 4));
      realtimePublishers_.push_back(publisher);
    }

    // resize last times published
    lastTimePublished_.resize(co2SensorNames.size());
    return true;
  }

  void Co2SensorController::starting(const ros::Time& time)
  {
    // Initialize last time published
    for (int ii = 0; ii < lastTimePublished_.size(); ii++)
    {
      lastTimePublished_[ii] = time;
    }
  }

  void Co2SensorController::update(
    const ros::Time& time, const ros::Duration& period)
  {
    // Publish messages
    for (int ii = 0; ii < realtimePublishers_.size(); ii++)
    {
      if (lastTimePublished_[ii] + ros::Duration(1.0/publishRate_) < time)
      {
        if (realtimePublishers_[ii]->trylock())
        {
          lastTimePublished_[ii] =
            lastTimePublished_[ii] + ros::Duration(1.0/publishRate_);

          // Fill co2 msg
          realtimePublishers_[ii]->msg_.header.stamp = time;
          realtimePublishers_[ii]->msg_.header.frame_id =
            sensorHandles_[ii].getFrameId();

          realtimePublishers_[ii]->msg_.co2_percentage =
            *sensorHandles_[ii].getCo2Percentage();
          realtimePublishers_[ii]->unlockAndPublish();
        }
      }
    }
  }

  void Co2SensorController::stopping(const ros::Time& time)
  {
  }

  Co2SensorController::Co2SensorController()
  {
  }

  Co2SensorController::~Co2SensorController()
  {
  }
}  // namespace arm
}  // namespace pandora_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pandora_hardware_interface::arm::Co2SensorController,
  controller_interface::ControllerBase)
