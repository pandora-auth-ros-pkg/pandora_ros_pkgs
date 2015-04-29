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

#ifndef IMU_HARDWARE_INTERFACE_IMU_RPY_INTERFACE_H
#define IMU_HARDWARE_INTERFACE_IMU_RPY_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <string>

namespace pandora_hardware_interface
{
namespace imu
{
  /**
  @brief A handle used to read the state of a IMU sensor.
  @details Depending on the sensor, not all readings exposed by the handle class
  might be available
  **/
  class ImuRPYHandle
  {
  public:
    struct Data
    {
      Data()
        : name(""),
          frame_id(""),
          roll(0),
          pitch(0),
          yaw(0)
      {
      }

      std::string name;  //!< The name of the sensor
      std::string frame_id;  //!< The reference frame to which this sensor is associated
      double* roll;  //!< roll ptr
      double* pitch;  //!< pitch ptr
      double* yaw;  //!< yaw ptr
    };

    ImuRPYHandle(const Data& data = Data())
      : name_(data.name),
        frame_id_(data.frame_id),
        roll_(data.roll),
        pitch_(data.pitch),
        yaw_(data.yaw)
    {
    }

    std::string getName() const {return name_;}
    std::string getFrameId() const {return frame_id_;}
    inline const double* getRoll() const {return roll_;}
    inline const double* getPitch() const {return pitch_;}
    inline const double* getYaw() const {return yaw_;}

  private:
    std::string name_;
    std::string frame_id_;

    double* roll_;
    double* pitch_;
    double* yaw_;
  };

  /**
  @class ImuRPYInterface
  @briefHardware interface to support reading the state of an IMU sensor
  **/
  class ImuRPYInterface :
    public hardware_interface::HardwareResourceManager<ImuRPYHandle>
  {
  };
}  // namespace imu
}  // namespace pandora_hardware_interface
#endif  // IMU_HARDWARE_INTERFACE_IMU_RPY_INTERFACE_H
