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
#ifndef LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H
#define LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H

#include <controller_interface/controller.h>
#include <leddar_hardware_interface/leddar_sensor_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<realtime_tools::RealtimePublisher<
  sensor_msgs::LaserScan> > LeddarRealtimePublisher;

namespace pandora_hardware_interface
{
namespace leddar
{
  /**
   @class LeddarSensorController
   @brief Controller used to publish leddar sensor measurements
  **/
  class LeddarSensorController :
    public controller_interface::Controller<LeddarSensorInterface>
  {
    public:
      /**
       @brief Default Constructor
      **/
      LeddarSensorController();

      /**
       @brief Default Destructor
      **/
      ~LeddarSensorController();

      /**
       @brief Initializes the leddar controller
       @param leddarSensorInterface [LeddarSensorInterface*] : Leddar Sensor Interface
       @param rootNodeHandle [ros::NodeHandle&] : root node ros node handle
       @param controllerNodeHandle [ros::NodeHandle] : controller node handle
       @return void
      **/
      virtual bool init(
        LeddarSensorInterface* leddarSensorInterface,
        ros::NodeHandle& rootNodeHandle,
        ros::NodeHandle& controllerNodeHandle);

      /**
       @brief Starts the leddar controller
       @param time [const ros::Time&] : current time
       @return void
      **/
      virtual void starting(const ros::Time& time);

      /**
       @brief Updates the leddar controller and publishes the new measurements
       @param time [const ros::Time&] : current time
       @param period [const ros::Duration&] : time since last update
       @return void
      **/
      virtual void update(const ros::Time& time, const ros::Duration& period);

      /**
       @brief Stops the controller
       @param time [const ros::Time& time] : current time
       @return void
      **/
      virtual void stopping(const ros::Time& time);

    private:
      std::vector<LeddarSensorHandle> sensorHandles_;  //!< leddar sensor handles' vector
      std::vector<LeddarRealtimePublisher> realtimePublishers_;  //!< leddar publisher
      std::vector<ros::Time> lastTimePublished_;  //!< time since last publishing
      double publishRate_;  //!< publishing frequency
  };
}  // namespace leddar
}  // namespace pandora_hardware_interface
#endif  // LEDDAR_CONTROLLERS_LEDDAR_SENSOR_CONTROLLER_H
