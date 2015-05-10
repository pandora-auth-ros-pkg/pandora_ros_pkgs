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

namespace pandora_hardware_interface
{
namespace leddar
{
  LeddarSerialInterface::LeddarSerialInterface(
    std::string device,
    std::string port_name,
    int address)
  :
    device_(device),
    port_name_(port_name),
    address_(address)
  {
    lAcquisition_ = new LtAcquisition();
  }

  LeddarSerialInterface::~LeddarSerialInterface()
  {
    delete[] lAcquisition_;
    LeddarDisconnect();
  }

  void LeddarSerialInterface::init()
  {
    char* pn = new char[port_name_.length()+1];
    strncpy(pn, port_name_.c_str(), sizeof(port_name_.c_str()));

    ROS_INFO("[leddar] Attempting to establish serial communication...");
    int connection_attempts = 10;
    while (LeddarConnect(pn, address_) != LT_SUCCESS)
    {
      connection_attempts--;
      ROS_ERROR("[leddar] Unable to establish serial communication [%d attempts left]",
        connection_attempts);
      if (connection_attempts == 0)
      {
        ROS_FATAL("Failed to establish serial communication. Aborting...");
        exit(-1);
      }
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("[leddar] Connection established.");
  }


  void LeddarSerialInterface::read()
  {
    int acquisition_attempts = 3;
    while (LeddarGetResults(lAcquisition_) != LT_SUCCESS && ros::ok())
    {
      ROS_ERROR("[leddar] Acquisition failed. Reattempting Acquisition...");
      acquisition_attempts--;
      if (acquisition_attempts == 0)
      {
        LeddarDisconnect();
        init();
      }
    }
  }
}  // namespace leddar
}  // namespace pandora_hardware_interface
