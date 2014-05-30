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
 * Author: Orestis Zachariadis
 *********************************************************************/

#include "arm_usb_interface/arm_usb_interface.h"

namespace pandora_hardware_interface
{
namespace arm
{

//  fcntl(fd, F_SETFL, FNDELAY);    //make read() non-blocking
//  fcntl(fd, F_SETFL, 0);  //make read() blocking

ArmUSBInterface::ArmUSBInterface()
{
//To make read non-blocking use the following
//  fd = open("/dev/head", O_RDWR | O_NOCTTY | O_NDELAY);
  while ((fd = open("/dev/head", O_RDWR | O_NOCTTY)) == -1)
  {
    ROS_ERROR("[Head]: cannot open usb port\n");
    ROS_ERROR("[Head]: open() failed with error [%s]\n", strerror(errno));

    ros::Duration(0.5).sleep();
  }

  ROS_INFO("[Head]: usb port successfully opened\n");

  /*Needs some time to initialize, even though it opens succesfully.
   tcflush() didn't work without waiting at least 8 ms*/
  ros::Duration(0.03).sleep();
}

ArmUSBInterface::~ArmUSBInterface()
{
  close(fd);
  ROS_INFO("[Head]: usb port closed because of program termination\n");
}

int ArmUSBInterface::grideyeValuesGet(const char& grideyeSelect,
                                      uint8_t * values)
{
  int nr;
  uint8_t bufOUT;

  switch (grideyeSelect)
  {
    case 'C':
      bufOUT = COMMAND_GEYE_CENTER;
      break;
    case 'L':
      bufOUT = COMMAND_GEYE_LEFT;
      break;
    case 'R':
      bufOUT = COMMAND_GEYE_RIGHT;
      break;
    default:
      bufOUT = COMMAND_GEYE_CENTER;
      break;
  }

  tcflush(fd, TCIFLUSH); //empties incoming buffer

  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUSB();
    return -1;
  }

  nr = read(fd, values, GEYE_NBYTES); //blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUSB();
    return -2;
  }
  else if (nr != GEYE_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    return -3;
  }
  else
  {
    std::stringstream ss; //TODO(orestis): slow ?

    ss << "[Head]: " << grideyeSelect << " GridEYE = ";
    for (int i = 0; i < GEYE_NBYTES; ++i)
    {
      ss << static_cast<int>(values[i]) << " ";
    }
    ROS_DEBUG("%s", ss.str().c_str());

    return 1;
  }
}

float ArmUSBInterface::co2ValueGet()
{
  union
  {
    uint8_t CO2bufIN[CO2_NBYTES];
    float CO2bufIN_float;
  };

  int nr;
  uint8_t bufOUT;

  tcflush(fd, TCIFLUSH); //empties incoming buffer

  bufOUT = COMMAND_CO2;
  nr = write(fd, (const void *)&bufOUT, COMMAND_NBYTES);
  if (nr != 1)
  {
    ROS_ERROR("[Head]: Write Error\n");
    reconnectUSB();
    return -1;
  }

  nr = read(fd, CO2bufIN, CO2_NBYTES); //blocking
  if (nr < 0)
  {
    ROS_ERROR("[Head]: Read Error\n");
    reconnectUSB();
    return -2;
  }
  else if (nr != CO2_NBYTES)
  {
    ROS_ERROR("[Head]: Wrong number of bytes read\n");
    return -3;
  }
  else
  {
    ROS_DEBUG("[Head]: CO2 Gas Reading = %f\n", CO2bufIN_float);
    return CO2bufIN_float;
  }
}

void ArmUSBInterface::reconnectUSB()
{
  //reconnectUSB() should be called until communication is restored.
  close(fd);
  ROS_INFO("[Head]: usb port closed\n");
  ros::Duration(1.5).sleep();

  while ((fd = open("/dev/head", O_RDWR | O_NOCTTY)) == -1)
  {
    ROS_ERROR("[Head]: failed to reopen usb port\n");
    ROS_ERROR("[Head]: open() failed with error [%s]\n", strerror(errno));

    ros::Duration(0.5).sleep();
  }

  ROS_INFO("[Head]: usb port successfully reopened\n");

  /*Needs some time to initialize, even though it opens succesfully.
   tcflush() didn't work without waiting at least 8 ms*/
  ros::Duration(0.03).sleep();
}

} // namespace arm
} // namespace pandora_hardware_interface
