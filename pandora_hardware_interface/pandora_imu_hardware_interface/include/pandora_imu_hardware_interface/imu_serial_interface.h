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

#ifndef PANDORA_IMU_HARDWARE_INTERFACE_IMU_SERIAL_INTERFACE_H
#define PANDORA_IMU_HARDWARE_INTERFACE_IMU_SERIAL_INTERFACE_H

#include <boost/scoped_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <iostream>
#include <stdexcept>

#include <ros/ros.h>
#include <serial/serial.h>

namespace pandora_hardware_interface
{
namespace imu
{
  class ImuSerialInterface : private boost::noncopyable
  {
   public:
    ImuSerialInterface(
      const std::string& device,
      int speed,
      int timeout);

    void init();

    void read();

    inline float getRoll() const
    {
      return roll_;
    }
    inline float getPitch() const
    {
      return pitch_;
    }
    inline float getYaw() const
    {
      return yaw_;
    }

    inline void getData(
      float* yaw,
      float* pitch,
      float* roll) const
    {
      *yaw = yaw_;
      *pitch = pitch_;
      *roll = roll_;
    }

   private:
    void parse(const std::string& packet);
    bool check(const std::string& packet, int crc);

   private:
    float yaw_;
    float pitch_;
    float roll_;

    const std::string device_;
    const int speed_;
    const int timeout_;

    const boost::regex regex_;

    boost::scoped_ptr<serial::Serial> serialPtr_;
  };
}  // namespace imu
}  // namespace pandora_hardware_interface

#endif  // PANDORA_IMU_HARDWARE_INTERFACE_IMU_SERIAL_INTERFACE_H
