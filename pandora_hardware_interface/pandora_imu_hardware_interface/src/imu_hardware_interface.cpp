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
#include "pandora_imu_hardware_interface/imu_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace imu
{
  ImuHardwareInterface::ImuHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    imuSerialInterface(
      "/dev/imu",
      38400,
      100)

  {
    imuSerialInterface.init();

    // connect and register imu sensor interface
    imuOrientation[0] = 0;
    imuOrientation[1] = 0;
    imuOrientation[2] = 0;
    imuOrientation[3] = 1;
    imuData_.orientation = imuOrientation;
    imuData_.name="/sensors/imu";  // /sensors might become namespace
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
    float yaw, pitch, roll;
    imuSerialInterface.read();
    imuSerialInterface.getData(&yaw, &pitch, &roll);

    yaw = (yaw - 180) * (2*boost::math::constants::pi<double>()) / 360;
    pitch = -pitch * (2*boost::math::constants::pi<double>()) / 360;
    roll = roll * (2*boost::math::constants::pi<double>()) / 360;
    geometry_msgs::Quaternion orientation;

    orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    imuOrientation[0] = orientation.x;
    imuOrientation[1] = orientation.y;
    imuOrientation[2] = orientation.z;
    imuOrientation[3] = orientation.w;
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
