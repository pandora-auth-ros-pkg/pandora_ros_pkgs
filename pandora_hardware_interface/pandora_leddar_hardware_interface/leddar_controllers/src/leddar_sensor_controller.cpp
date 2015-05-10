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
#include <leddar_controllers/leddar_sensor_controller.h>

namespace pandora_hardware_interface
{
namespace leddar
{
  bool LeddarSensorController::init(
    LeddarSensorInterface* leddarSensorInterface,
    ros::NodeHandle& rootNodeHandle,
    ros::NodeHandle& controllerNodeHandle)
  {
    // Get sensor names from interface
    const std::vector<std::string>& leddarSensorNames =
      leddarSensorInterface->getNames();

    // Read publish rate from yaml
    if (!controllerNodeHandle.getParam("publish_rate", publishRate_))
    {
      publishRate_ = 50;
      ROS_ERROR("Parameter 'publish_rate' not set in yaml, using default 50Hz");
    }

    for (int ii = 0; ii < leddarSensorNames.size(); ii++)
    {
      // Get sensor handles from interface
      sensorHandles_.push_back(
        leddarSensorInterface->getHandle(leddarSensorNames[ii]));

      // Create publisher for each controller
      LeddarRealtimePublisher publisher(
        new realtime_tools::RealtimePublisher<
          sensor_msgs::LaserScan>(
            rootNodeHandle, "/sensors/leddar", 4));
      realtimePublishers_.push_back(publisher);
      // TODO set params of msg (angle_min, angle_max, angle_increment, 
      // time_increment, scan_time, range_min, range_max) from yaml
      
      double temp_param;
      controllerNodeHandle.getParam("angle_min", temp_param); 
      realtimePublishers_[ii]->msg_.angle_min = (float)temp_param;
      controllerNodeHandle.getParam("angle_max", temp_param);
      realtimePublishers_[ii]->msg_.angle_max = (float)temp_param;
      controllerNodeHandle.getParam("angle_increment", temp_param); 
      realtimePublishers_[ii]->msg_.angle_increment = (float)temp_param;
//    controllerNodeHandle.getParam("time_increment", temp_param); 
//    &realtimePublishers_[ii]->msg_.time_increment = (float)temp_param;
      controllerNodeHandle.getParam("scan_time", temp_param); 
      realtimePublishers_[ii]->msg_.scan_time = (float)temp_param;                
      controllerNodeHandle.getParam("range_min", temp_param);  
      realtimePublishers_[ii]->msg_.range_min = (float)temp_param;        
      controllerNodeHandle.getParam("range_max", temp_param);  
      realtimePublishers_[ii]->msg_.range_max = (float)temp_param;
    }

    // resize last times published
    lastTimePublished_.resize(leddarSensorNames.size());
    return true;
  }

  void LeddarSensorController::starting(const ros::Time& time)
  {
    // Initialize last time published
    for (int ii = 0; ii < lastTimePublished_.size(); ii++)
    {
      lastTimePublished_[ii] = time;
    }
  }

  void LeddarSensorController::update(
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

          // Fill leddar msg
          realtimePublishers_[ii]->msg_.header.stamp = time;
          realtimePublishers_[ii]->msg_.header.frame_id =
            sensorHandles_[ii].getFrameId();
          int count = *(sensorHandles_[ii].getLeddarDetectionCount());
          realtimePublishers_[ii]->msg_.ranges.clear();
          realtimePublishers_[ii]->msg_.intensities.clear();
          
          for (int jj=0; jj<count; jj++){
            realtimePublishers_[ii]->msg_.ranges.push_back( 
              *(sensorHandles_[ii].getLeddarDistances() + jj) );
            realtimePublishers_[ii]->msg_.intensities.push_back(
              *(sensorHandles_[ii].getLeddarAmplitudes() + jj) );
          }
          realtimePublishers_[ii]->unlockAndPublish();
        }
      }
    }
  }

  void LeddarSensorController::stopping(const ros::Time& time)
  {
  }

  LeddarSensorController::LeddarSensorController()
  {
  }

  LeddarSensorController::~LeddarSensorController()
  {
  }
}  // namespace leddar
}  // namespace pandora_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pandora_hardware_interface::leddar::LeddarSensorController,
  controller_interface::ControllerBase)
