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
* Author: Petros Evangelakos
*********************************************************************/

#include "linear_actuator_com_interface/firgelli_com_interface.h"

namespace pandora_hardware_interface
{
namespace linear_actuator
{
  int  FirgelliComInterface::mDebug_;
  FirgelliComInterface::FirgelliComInterface()
  {
    mCtx_ = NULL;
    mHandle_ = NULL;
    mInterface_ = 0;
    libusb_init(&mCtx_);
    rank_ = 1;
    feedback_ = 0;
  }

  FirgelliComInterface::~FirgelliComInterface()
  {
    int retval = libusb_release_interface(mHandle_, mInterface_);
    assert(retval == 0);
    if (mHandle_)
      closeDevice();
    libusb_exit(mCtx_);
  }

  void FirgelliComInterface::init()
  {
    uint8_t buf[3];
    int command;
    openDevice();

    // set accuracy
    command = 1;
    buf[0] = SET_ACCURACY;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set retract limit
    command = 5;
    buf[0] = SET_RETRACT_LIMIT;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set extend limit
    command = 1000;
    buf[0] = SET_EXTEND_LIMIT;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set movement threshold
    command = 3;
    buf[0] = SET_MOVEMENT_THRESHOLD;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set stall time
    command = 10000;
    buf[0] = SET_STALL_TIME;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set PWM threshold
    command = 80;
    buf[0] = SET_PWM_THRESHOLD;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set Derivative Threshold
    command = 3;
    buf[0] = SET_DERIVATIVE_THRESHOLD;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set max derivative
    command = 1023;
    buf[0] = SET_DERIVATIVE_MAXIMUM;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set min derivative
    command = 0;
    buf[0] = SET_DERIVATIVE_MINIMUM;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set max pwm
    command = 1023;
    buf[0] = SET_PWM_MAXIMUM;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set min pwm
    command = 0;
    buf[0] = SET_PWM_MINIMUM;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set Kp
    command = 1;
    buf[0] = SET_PROPORTIONAL_GAIN;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set Kd
    command = 10;
    buf[0] = SET_DERIVATIVE_GAIN;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set average RC
    command = 4;
    buf[0] = SET_AVERAGE_RC;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set average ADC
    command = 8;
    buf[0] = SET_AVERAGE_ADC;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
    // set speed
    command = 1023;
    buf[0] = SET_SPEED;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High
    write(buf, sizeof(buf));
  }

  void FirgelliComInterface::openDevice()
  {
    int rval;
    uint16_t vid = 0x04d8;  // Microtech
    uint16_t pid = 0xfc5f;  // ??
    ROS_INFO("firgelli:: vid=%x",
      vid);

    mHandle_ = libusb_open_device_with_vid_pid(mCtx_, vid, pid);
    if (mHandle_ == NULL){
      ROS_INFO("firgelli::open device vid=0x%x pid=0x%x not found\n",
        vid, pid);
      exit(1);
    }

    rval = libusb_claim_interface(mHandle_, mInterface_);
    assert(rval == 0);
  }



  bool FirgelliComInterface::write(const uint8_t* data, size_t size)
  {
    int transferred = 0;
    unsigned int timeout = 1000;
    uint8_t buf[3];
    buf[0] = data[0];
    buf[1] = data[1];
    buf[2] = data[2];
    unsigned char endpoint = 1 | LIBUSB_ENDPOINT_OUT;

    int rval = libusb_bulk_transfer(
      mHandle_,
      endpoint,
      buf,
      size,
      &transferred,
      timeout);

    if (rval)
    {
      ROS_INFO("firgelli: bad write rval=%d\n", rval);
      exit(1);
    }

    endpoint = 1 | LIBUSB_ENDPOINT_IN;
    rval = libusb_bulk_transfer(
    mHandle_,
    endpoint,
    buf,
    size,
    &transferred,
    timeout);


    if (rval)
    {
      ROS_INFO("firgelli: bad read rval=%d \n",
        rval);
      exit(1);
    }

    feedback_ = buf[1] + buf[2] * 256;

    return true;
  }

  float FirgelliComInterface::readScaledFeedback()
  {
    uint8_t data[2];  // useless
    read(data, sizeof(data));
    return static_cast<float>(feedback_) / 1023.0 * 14.0;
  }

  void FirgelliComInterface::closeDevice()
  {
     libusb_close(mHandle_);
  }

  bool FirgelliComInterface::read(uint8_t* data, size_t size)
  {
    uint8_t buf[3];
    buf[0] = GET_FEEDBACK;
    buf[1] = 0;
    buf[2] = 0;

    bool rval = write(buf, sizeof(buf));
    return rval;
  }


  bool FirgelliComInterface::setTarget(float target)
  {
    uint8_t buf[3];
    uint16_t command = target * 1023.0 / 14.0;

    buf[0] = SET_POSITION;
    buf[1] = command % 256;  // Low
    buf[2] = command / 256;  // High

    bool rval = write(buf, sizeof(buf));
    return rval;
  }
}  // namespace linear_actuator
}  // namespace pandora_hardware_interface
