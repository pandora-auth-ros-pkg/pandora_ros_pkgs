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
* Author: George Kouros
*********************************************************************/

#ifndef PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H
#define PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H

#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <stdlib.h>

#include "pandora_imu_hardware_interface/abstract_imu_serial_interface.h"

#define K_SET_DATA_COMPONENTS 0x03
#define K_GET_DATA 0x04
#define K_GET_DATA_RESP 0x05
#define K_SET_CONFIG 0x06
#define K_BIG_ENDIAN 0x06
#define K_SAVE 0x09
#define K_START_CONTINUOUS_MODE 0x15
#define K_STOP_CONTINUOUS_MODE 0x16
#define K_SET_FUNCTIONAL_MODE 0x4F
#define K_AHRS_MODE 0x01

#define K_HEADING 0x05
#define K_PITCH 0x18
#define K_ROLL 0x19
#define K_ACCEL_X 0x15
#define K_ACCEL_Y 0x16
#define K_ACCEL_Z 0x17
#define K_GYRO_X 0x4A
#define K_GYRO_Y 0x4B
#define K_GYRO_Z 0x4C


#define K_TRUE 0x01
#define K_FALSE 0x00

namespace pandora_hardware_interface
{
namespace imu
{
  /**
   @class AhrsSerialInterface
   @brief Class used for serial communication with a Trax AHRS
  **/
  class AhrsSerialInterface : public AbstractImuSerialInterface
  {
   public:
    /**
     @brief Default Constructor
     @param device [std::string &] : AHRS device com port name
     @param speed [int] : Serial communication speed (baud rate)
     @param timeout [int] : Connection response timeout
    **/
    AhrsSerialInterface(
      const std::string& device,
      int speed,
      int timeout);

    /**
     @brief Establishes serial communication
     @details Also sends a command to the device to send the data in little 
     endian format (big endian format is default)
     @return void
    **/
    void init();

    /**
     @brief Reads yaw, pitch and roll from the Trax AHRS
     @details Init must be called first to establish serial communication
     @return void
    **/
    void read();

    /**
     @brief Writes command to trax ahrs
     @param commandCode [char*] : command code bytes 
     @param length [size_t] : number of bytes of command code
     @return void
    **/
    void write(char* commandCode, size_t length);

   private:
    /**
     @brief Extract yaw, pitch, roll from packet
     @param packet [std::string&] : packet containing the raw ahrs data
     @return void
    **/
    void parse(const std::string& packet);

    /**
     @brief Checks if the calculated crc matches the one in the received packet
     @param packet [const std::string&] : reference to the received data packet
     @param crc [int] : the calculated crc of the packet
     @return bool
    **/
    bool check(const std::string& packet, int crc);

    /**
     @brief Returns the crc of a byte stream using the xmodem crc algorithm
     @param data [unsigned char*] : data packet as char array
     @param dataSize [size_t] : size of packet char array
     @param storeCrcInData [bool] : true -> store the crc code in the last two 
     bytes of the packet, else false
     @return uint16_t crc
    **/
    uint16_t calcCrc(unsigned char* data, size_t dataSize, bool storeCrcInPacket);

   private:
    //!< expression used to extract yaw, pitch, roll from packet
    const boost::regex regex_;
  };
}  // namespace imu
}  // namespace pandora_hardware_interface

#endif  // PANDORA_IMU_HARDWARE_INTERFACE_AHRS_SERIAL_INTERFACE_H
