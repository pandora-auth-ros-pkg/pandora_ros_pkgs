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

#include <pandora_imu_hardware_interface/imu_serial_interface.h>

namespace pandora_hardware_interface
{
namespace imu
{
  ImuSerialInterface::ImuSerialInterface(
      const std::string& device,
      int speed,
      int timeout)
  :
    AbstractImuSerialInterface(device, speed, timeout),
    // TODO(czalidis): add accel
    regex_(
      "C([0-9]+\\.[0-9]+)P([-]*[0-9]+\\.[0-9]+)R([-]*[0-9]+\\.[0-9]+)"
      "Ax([-]*[0-9]+\\.[0-9]+)Ay([-]*[0-9]+\\.[0-9]+)Az([-]*[0-9]+\\.[0-9]+).*")
  {
  }

  void ImuSerialInterface::init()
  {
    if (serialPtr_ == NULL)
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
        ROS_FATAL("[compass] Cannot open port!!");
        ROS_FATAL("%s", ex.what());
        exit(-1);
      }
    }
    else
    {
      throw std::logic_error("Init called twice!!");
    }
  }

  void ImuSerialInterface::read()
  {
    if (serialPtr_ == NULL)
      throw std::logic_error("read() called before init()!");

    std::string buffer;
    size_t start = std::string::npos;
    size_t end = std::string::npos;

    int count = 0;

    // get whole packet
    while (start == std::string::npos && end == std::string::npos)
    {
      serialPtr_->readline(buffer);

      start = buffer.find("$");
      end = buffer.find("*");

      // timeout handling
      if (count > 10)
      {
        throw std::runtime_error(
          "Read from serial timed out!! Is it connected?");
      }

      count++;
    }

    std::string packet = buffer.substr(start + 1, end - start - 1);
    std::string crc = buffer.substr(end + 1,  2);
    std::stringstream stream(crc);
    int crcInt;
    stream >> std::hex >> crcInt;

    if (check(packet, crcInt))
      parse(packet);
  }

  bool ImuSerialInterface::check(const std::string& packet, int crc)
  {
    char sum = 0;

    for (int i = 0; i < packet.size(); i++)
    {
      sum = sum ^ packet[i];
    }

    if (static_cast<int>(sum) == crc)
    {
      return true;
    }
    return false;
  }

  void ImuSerialInterface::parse(const std::string& packet)
  {
    boost::match_results<std::string::const_iterator> data;

    if (boost::regex_match(packet, data, regex_))
    {
      yaw_ = boost::lexical_cast<float> (data[1]);
      pitch_ = boost::lexical_cast<float> (data[2]);
      roll_ = boost::lexical_cast<float> (data[3]);
      for (int ii = 0; ii < 3; ii++)
      {
        linearAcceleration_[ii] =
          9.81 * boost::lexical_cast<float>(data[ii + 4]);
      }
    }
/*
    ROS_INFO("yaw[%f], pitch[%f], roll[%f], Ax[%f], Ay[%f], Az[%f]", 
      yaw_, pitch_, roll_, linearAcceleration_[0], linearAcceleration_[1], 
      linearAcceleration_[2]);
*/
  }
}  // namespace imu
}  // namespace pandora_hardware_interface
