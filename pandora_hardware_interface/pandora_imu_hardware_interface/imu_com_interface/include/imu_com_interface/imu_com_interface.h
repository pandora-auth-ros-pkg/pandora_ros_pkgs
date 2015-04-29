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
* Author: Chris Zalidis
*********************************************************************/

#ifndef IMU_COM_INTERFACE_IMU_COM_INTERFACE_H
#define IMU_COM_INTERFACE_IMU_COM_INTERFACE_H

#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include "imu_com_interface/abstract_imu_com_interface.h"

namespace pandora_hardware_interface
{
namespace imu
{
  /**
   @class ImuComInterface
   @brief Class used for serial communication with Compass OS-5000 IMU
  **/
  class ImuComInterface : public AbstractImuComInterface
  {
   public:
    /**
     @brief Default Constructor
     @param device [std::string &] : IMU device com port name
     @param speed [int] : Serial communication speed (baud rate)
     @param timeout [int] : Connection response timeout
    **/
    ImuComInterface(
      const std::string& device,
      int speed,
      int timeout);

    /**
     @brief Establishes serial communication
     @return void
    **/
    void init();

    /**
     @brief Reads raw data from the IMU and calculates yaw, pitch and roll 
     @details Init must be called first to establish serial communication
     @return void
    **/
    void read();

   private:
    /**
     @brief Transform raw IMU data to yaw, pitch and roll meausurements
     @param packet [std::string&] : packet containing the raw imu data
     @return void
    **/
    void parse(const std::string& packet);

    /**
     @brief Check size of latest received IMU data packet
     @return bool
    **/
    bool check(const std::string& packet, int crc);

    //!< expression used to extract data from imu packet
    const boost::regex regex_;
  };
}  // namespace imu
}  // namespace pandora_hardware_interface

#endif  // IMU_COM_INTERFACE_IMU_COM_INTERFACE_H
