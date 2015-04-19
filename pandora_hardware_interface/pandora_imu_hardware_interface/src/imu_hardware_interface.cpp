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
* Author:  George Kouros
*********************************************************************/
#include "pandora_imu_hardware_interface/imu_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace imu
{
  ImuHardwareInterface::ImuHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    imuSerialInterface("/dev/imu", 38400, 100),
    ahrsSerialInterface("/dev/imu", 38400, 100)
  {
    std::string device_type;
    if (nodeHandle_.getParam("device_type", device_type))
    {
      if (device_type == "imu")
      {
        nodeHandle_.param("imu_roll_offset", rollOffset_, 0.0);
        nodeHandle_.param("imu_pitch_offset", pitchOffset_, 0.0);

        imuSerialInterface.init();
        serialInterface = &imuSerialInterface;
      }
      else if (device_type == "ahrs")
      {
        nodeHandle_.param("ahrs_roll_offset", rollOffset_, 0.0);
        nodeHandle_.param("ahrs_pitch_offset", pitchOffset_, 0.0);

        ahrsSerialInterface.init();
        serialInterface = &ahrsSerialInterface;
      }
      else
      {
        ROS_FATAL("[ERROR]: device_type not set in parameter server.");
        exit(-1);
      }
    }

    // initialize to zero imu data arrays
    for (int ii = 0; ii < 3; ii++)
    {
      imuOrientation_[ii] = 0;
      imuAngularVelocity_[ii] = 0;
      imuLinearAcceleration_[ii] = 0;
    }
    imuOrientation_[3] = 1;

    imuData_.orientation = imuOrientation_;
    imuData_.angular_velocity = imuAngularVelocity_;
    imuData_.linear_acceleration = imuLinearAcceleration_;
    imuData_.name="/sensors/imu";
    imuData_.frame_id="base_link";

    hardware_interface::ImuSensorHandle imuSensorHandle(imuData_);
    imuSensorInterface_.registerHandle(imuSensorHandle);
    registerInterface(&imuSensorInterface_);
  }

  ImuHardwareInterface::~ImuHardwareInterface()
  {
  }

  void ImuHardwareInterface::read()
  {
    float yaw, pitch, roll, aV[3], lA[3];
    serialInterface->read();
    serialInterface->getData(
      &yaw,
      &pitch,
      &roll,
      aV,
      lA);

    yaw = (yaw - 180) * (2*boost::math::constants::pi<double>()) / 360;
    pitch = -pitch * (2*boost::math::constants::pi<double>()) / 360;
    roll = roll * (2*boost::math::constants::pi<double>()) / 360;
    geometry_msgs::Quaternion orientation;

    orientation = tf::createQuaternionMsgFromRollPitchYaw(
      roll - rollOffset_, pitch -pitchOffset_, yaw);
    imuOrientation_[0] = orientation.x;
    imuOrientation_[1] = orientation.y;
    imuOrientation_[2] = orientation.z;
    imuOrientation_[3] = orientation.w;

    for (int ii = 0; ii < 3; ii++)
    {
      imuAngularVelocity_[ii] = static_cast<double>(aV[ii]);
      imuLinearAcceleration_[ii] = static_cast<double>(lA[ii]);
    }
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
