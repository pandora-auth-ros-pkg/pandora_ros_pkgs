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

#include <imu_com_interface/ahrs_com_interface.h>

namespace pandora_hardware_interface
{
namespace imu
{
  AhrsComInterface::AhrsComInterface(
    const std::string& device,
    int speed,
    int timeout)
  :
    AbstractImuComInterface(device, speed, timeout),
    regex_(
      ".*\x05(.{4})\x18(.{4})\x19(.{4})"
      "\x15(.{4})\x16(.{4})\x17(.{4})"
      "\x4a(.{4})\x4b(.{4})\x4c(.{4}).*",
      boost::regex::extended)
  {
  }


  void AhrsComInterface::init()
  {
    if (serialPtr_ == NULL || !serialPtr_->isOpen())
    {
      try
      {
        serialPtr_.reset(
          new serial::Serial(
            device_,
            speed_,
            serial::Timeout::simpleTimeout(timeout_)));
      }
      catch (serial::IOException& ex)
      {
        ROS_FATAL("[Trax AHRS] Cannot open port!!");
        ROS_FATAL("%s", ex.what());
        exit(-1);
      }
    }
    else
    {
      ROS_ERROR("Init called twice!!");
    }
  }


  void AhrsComInterface::read()
  {
    if (serialPtr_ == NULL){
      ROS_ERROR("read() called before init(). Calling init() now...");
      init();
    }

    // flush input buffer from remainder packets
    serialPtr_->flushInput();

    // write getData command
    char getDataCmd = K_GET_DATA;
    write(&getDataCmd, 1);

    // read data from serial port
    std::string buffer;
    serialPtr_->read(buffer, 51);

    // check data packet
    char charBuffer[51];
    strncpy(charBuffer, buffer.c_str(), 51);

    if (
      check(buffer, calcCrc((unsigned char*)buffer.c_str(), 51, false)))
    {
      parse(buffer);  // parse data packet
    }
    else
    {
      ROS_ERROR("Check CRC failed");
    }
  }


  void AhrsComInterface::write(char* commandCode, size_t length)
  {
    unsigned char command[4+length];
    command[0] = (4 + length) & 0xFF00;
    command[1] = (4 + length) & 0x00FF;

    for (int ii = 0; ii < length; ii++)
    {
      command[2 + ii] = commandCode[ii];
    }
    uint16_t crc = calcCrc(command, 4 + length, true);
    command[4 + length - 2] = static_cast<unsigned char>(crc >> 8);
    command[4 + length - 1] = static_cast<unsigned char>(crc & 0x00FF);

    // write command to ahrs
    serialPtr_->write(command, 4 + length);
  }


  bool AhrsComInterface::check(const std::string& packet, int crc)
  {
    int crcInPacket =
      ((packet[packet.size()-2] << 8) & 0xFF00) |
      (packet[packet.size()-1] & 0x00FF);

    if (crcInPacket == crc)
      return true;
    else
      return false;
  }


  uint16_t AhrsComInterface::calcCrc(
    unsigned char* packet, size_t packetSize, bool storeCrcInPacket)
  {
    uint16_t crc = 0;
    for (int ii = 0; ii < packetSize-2; ii++)
    {
      crc = crc ^ ((uint16_t)packet[ii] << 8);
      for (int jj = 0; jj < 8; jj++)
      {
        if (crc & 0x8000)
          crc = (crc << 1) ^ 0x1021;
        else
          crc <<= 1;
      }
    }
    return crc;
  }


  void AhrsComInterface::parse(const std::string& packet)
  {
    boost::match_results<std::string::const_iterator> data;

    if (boost::regex_match(packet, data, regex_))
    {
      memcpy(
        &yaw_,
        data[1].str().c_str(),
        sizeof(float));
      memcpy(
        &pitch_,
        data[2].str().c_str(),
        sizeof(float));
      memcpy(
        &roll_,
        data[3].str().c_str(),
        sizeof(float));
      for (int ii = 0; ii < 3; ii++)
      {
        memcpy(
          linearAcceleration_ + ii,
          data[ii + 4].str().c_str(),
          sizeof(float));
        memcpy(
          angularVelocity_ + ii,
          data[ii + 7].str().c_str(),
          sizeof(float));
      }
    }
    else
      ROS_ERROR("Did not match received packet to desirable pattern");
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
