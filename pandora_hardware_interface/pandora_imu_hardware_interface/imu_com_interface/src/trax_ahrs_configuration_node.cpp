/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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

#include "imu_com_interface/ahrs_com_interface.h"

/*
 * Node Used for the setup of the PNI Trax AHRS sensor, so that the device can 
 * be used with the imu_hardware_interface_node
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trax_ahrs_configuration_node");
  ros::NodeHandle nodeHandle;

  pandora_hardware_interface::imu::AhrsComInterface
    serial("/dev/ahrs", 38400, 100);

  serial.init();
  ROS_INFO("Starting Configuration:");
  ROS_INFO("=======================");

  // configure device for ahrs mode (not compass mode)
  char setFunctionalModeCmd[2] = {K_SET_FUNCTIONAL_MODE, K_AHRS_MODE};
  serial.write(setFunctionalModeCmd, 2);
  ROS_INFO("Functional Mode of device set to AHRS mode");

  // configure endianess of data in ahrs packet
  char endianessCmd[3] = {
    K_SET_CONFIG,
    K_BIG_ENDIAN,
    K_FALSE};
  serial.write(endianessCmd, 3);
  ROS_INFO("Endianess of data in packets set to little endian");

  // configure packet composition of ahrs
  char pkgCompositionCmd[11] = {
    K_SET_DATA_COMPONENTS,
    0x09,
    K_HEADING,
    K_PITCH,
    K_ROLL,
    K_ACCEL_X,
    K_ACCEL_Y,
    K_ACCEL_Z,
    K_GYRO_X,
    K_GYRO_Y,
    K_GYRO_Z};
  serial.write(pkgCompositionCmd, 11);
  ROS_INFO("Data composition configured as: Y,P,R,Ax,Ay,Az,Gx,Gy,Gz");

  // configure ahrs for polling mode
  char setNonContinuousModeCmd = K_STOP_CONTINUOUS_MODE;
  serial.write(&setNonContinuousModeCmd, 1);
  ROS_INFO("Acquisition Mode set to polling");


  // tell ahrs to save the configurations
  char saveConfigurationsCmd = K_SAVE;
  serial.write(&saveConfigurationsCmd, 1);
  ROS_INFO("=====================");
  ROS_INFO("Configurations saved!");

  return 0;
}
