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
 * Author: George Kouros
 *********************************************************************/

#include "arm_usb_interface/arm_usb_interface.h"

int main(int argc, char** argv)
{
  ros::Time::init();

  pandora_hardware_interface::arm::ArmUsbInterface arm;

  uint16_t range[2];
  float co2Percentage;
  uint16_t encoderDegrees;
  uint16_t tempVoltage[2];
  double voltage[2];
  uint8_t temperature[64];
  std::stringstream ss;

  while (true)
  {
    arm.readSonarValues('L', range);
    ROS_INFO("1");
    arm.readSonarValues('R', range + 1);
    ROS_INFO("2");
    arm.readCo2Value(&co2Percentage);
    ROS_INFO("3");
    arm.readEncoderValue(&encoderDegrees);
    ROS_INFO("4");
    arm.readBatteryValues('M', tempVoltage);
    ROS_INFO("5");
    arm.readBatteryValues('S', tempVoltage + 1);
    ROS_INFO("6");
    arm.readGrideyeValues('C', temperature);
    ROS_INFO("7");

    encoderDegrees = encoderDegrees * 360 / 1024;

    for (int ii = 0; ii < 2; ii++)
      voltage[ii] = (tempVoltage[ii] / 4096.) * 33.0;

    ROS_INFO("============================================");
    ROS_INFO("Left Sonar measurement: %d", range[0]);
    ROS_INFO("Right Sonar measurement: %d", range[1]);
    ROS_INFO("CO2 measurement: %f", co2Percentage);
    ROS_INFO("Encoder measurement: %d", encoderDegrees);
    ROS_INFO("Motor Battery Voltage measurement: %f", voltage[0]);
    ROS_INFO("Supply Batter Voltage measurement: %f", voltage[1]);
    ROS_INFO("============================================");


    ss << "\n============== Thermal Image ===============\n";
    for (int ii = 0; ii < 8; ii++)
    {
      for (int jj = 0; jj < 8; jj++)
      {
        ss << " " << static_cast<int>(temperature[ii * 8 + jj]);
      }
      ss << "\n";
    }
    ss << "============================================";

    ROS_INFO("%s", ss.str().c_str());

    for (int ii = 0; ii < 64; ii++){
      // 0 in temperature indicates communication problem
      if (temperature[ii] < 1 || temperature[ii] > 255)
        {
          ROS_INFO("GridEye values were not read correctly.");
          break;
        }
    }

    ss.str(std::string());

    ros::Duration(1.0).sleep();
  }
  return 0;
}

