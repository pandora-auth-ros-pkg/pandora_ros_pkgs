/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: George Kouros
*********************************************************************/

#include "leddar_serial_interface/leddar_serial_interface.h"
#include <iostream>

using pandora_hardware_interface::leddar::LeddarSerialInterface;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "leddar_demo");
  ros::NodeHandle nh;

  ROS_INFO("Leddar demo node launched");

  std::ostringstream ss;
  std::ostringstream sm;
  LtAcquisition* measurements;

  LeddarSerialInterface serial("leddar", "leddar", 1);
  serial.init();

  while (ros::ok())
  {
    serial.read();
    measurements = serial.getLAcquisition();

    ROS_INFO("Number of Detections: %d", measurements->mDetectionCount);
    for (int ii = 0; ii < measurements->mDetectionCount; ii++)
    {
      ss << " " << measurements->mDetections[ii].mSegment;
      sm << " " << measurements->mDetections[ii].mDistance;
    }
    ROS_INFO("#: %s", ss.str().c_str());
    ROS_INFO("D: %s", sm.str().c_str());

    ss.str(std::string());
    sm.str(std::string());

    ros::Duration(0.5).sleep();
  }
  return 0;
}
