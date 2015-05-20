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
#ifndef ARM_CONTROLLERS_RANGE_SENSOR_CONTROLLER_H
#define ARM_CONTROLLERS_RANGE_SENSOR_CONTROLLER_H

#include <controller_interface/controller.h>
#include <arm_hardware_interface/range_sensor_interface.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Range.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<realtime_tools::RealtimePublisher<
  sensor_msgs::Range> > RangeRealtimePublisher;

namespace pandora_hardware_interface
{
namespace arm
{
  class RangeSensorController :
    public controller_interface::Controller<
      RangeSensorInterface>
  {
   public:
    /**
     @class Co2SensorController
     @brief Controller used to publish CO2 meausurements with fixed frequency
    **/
    RangeSensorController();

    /**
     @brief Default Destructor
    **/
    ~RangeSensorController();

    /**
     @brief Initializes controller
     @param rangeSensorInterface [rangeSensorInterface*] : range sensor interface
     @param rootNodeHandle [ros::NodeHandle&] : Node handle at root namespace
     @param controllerNodeHandle [ros::NodeHandle&] : Node handle inside the 
       controller namespace
     @return bool
    **/
    virtual bool init(
      RangeSensorInterface* rangeSensorInterface,
      ros::NodeHandle& rootNodeHandle,
      ros::NodeHandle& controllerNodeHandle);

    /**
     @brief Starts the range sensor controller
     @param time [ros::Time&] : Current time
     @return void
    **/      
    virtual void starting(const ros::Time& time);

    /**
     @brief Updates the range Controller and publishes new range measurements
     @param time [ros::Time&] : Current time
     @param period [ros::Duration&] : Time since last update
     @return void
    **/    
    virtual void update(const ros::Time& time, const ros::Duration& period);

    /**
     @brief Stops the range sensor controller
     @param time [ros::Time&] : Current time
     @return void
    **/   
    virtual void stopping(const ros::Time& time);

   private:
    //!< node handle of root namespace
    const ros::NodeHandle* rootNodeHandle_;
    //!< range sensor handle
    std::vector<RangeSensorHandle> sensorHandles_;
    //!< range msg publisher
    std::vector<RangeRealtimePublisher> realtimePublishers_;
    //!< Time since last publishing
    std::vector<ros::Time> lastTimePublished_;
    //!< range sensor interface
    RangeSensorInterface* rangeSensorInterface_;
    //!< CO2 msg publishing frequency
    double publishRate_;
  };
}  // namespace arm
}  // namespace pandora_hardware_interface
#endif  // ARM_CONTROLLERS_RANGE_SENSOR_CONTROLLER_H
