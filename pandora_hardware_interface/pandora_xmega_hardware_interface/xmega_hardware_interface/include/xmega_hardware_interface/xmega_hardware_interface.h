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
#ifndef XMEGA_HARDWARE_INTERFACE_XMEGA_HARDWARE_INTERFACE_H
#define XMEGA_HARDWARE_INTERFACE_XMEGA_HARDWARE_INTERFACE_H

#include <boost/math/constants/constants.hpp>
#include "ros/ros.h"
#include "tf/tf.h"
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <xmega_hardware_interface/battery_interface.h>
#include <xmega_hardware_interface/range_sensor_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <xmega_serial_interface/xmega_serial_interface.h>
#include <sensor_msgs/Range.h>

namespace pandora_hardware_interface
{
namespace xmega
{
  class XmegaHardwareInterface : public hardware_interface::RobotHW
  {
    private:
      ros::NodeHandle nodeHandle_;
      XmegaSerialInterface serialInterface;

      BatteryInterface batteryInterface_;
      RangeSensorInterface rangeSensorInterface_;
      hardware_interface::JointStateInterface jointStateInterface_;
      std::vector<BatteryHandle::Data>
        batteryData_;
      std::vector<RangeSensorHandle::Data>
        rangeData_;

      std::vector<std::string> batteryNames_;
      double** voltage_;
      int* voltageBufferCounter_;

      std::vector<std::string> rangeSensorName_;
      std::vector<std::string> frameId_;
      int* radiationType_;
      double* fieldOfView_;
      double* minRange_;
      double* maxRange_;
      double** range_;
      int* rangeBufferCounter_;
      int* i2c_address_;  // not stored in handle

      std::vector<std::string> jointNames_;
      double position_[2];
      double velocity_[2];
      double effort_[2];

      void registerBatteryInterface();
      void registerRangeSensorInterface();
      void registerJointStateInterface();
    public:
      explicit XmegaHardwareInterface(
        ros::NodeHandle nodeHandle);
      ~XmegaHardwareInterface();
      void read();
  };
}  // namespace xmega
}  // namespace pandora_hardware_interface
#endif  // XMEGA_HARDWARE_INTERFACE_XMEGA_HARDWARE_INTERFACE_H
