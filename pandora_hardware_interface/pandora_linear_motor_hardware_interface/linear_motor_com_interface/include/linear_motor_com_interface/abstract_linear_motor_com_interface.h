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

#ifndef LINEAR_MOTOR_COM_INTERFACE_ABSTRACT_LINEAR_MOTOR_COM_INTERFACE_H
#define LINEAR_MOTOR_COM_INTERFACE_ABSTRACT_LINEAR_MOTOR_COM_INTERFACE_H

#include <ros/ros.h>

namespace pandora_hardware_interface
{
namespace linear
{
  /**
  @class AbstractLinearMotorComInterface
  @brief Abstract Linear Motor Communication Interface class
  **/
  class AbstractLinearMotorComInterface : private boost::noncopyable
  {
   public:
    /**
    @brief Default Constructor
    **/
    AbstractLinearMotorComInterface()
    {
    }

    /**
    @brief Default Destructor
    **/
    ~AbstractLinearMotorComInterface()
    {
    }

    /**
    @brief Opens communication port and performs other initialization tasks
    @return void
    **/
    virtual void init() = 0;

    /**
    @brief Opens communication port
    @return void
    **/
    virtual void openDevice() = 0;

    /**
    @brief Closes opened communication port
    @brief This method is used to close linear motor communication port
    **/
    virtual void closeDevice() = 0;

    /**
    @brief Write command to linear motor
    @return bool : true for success, false for error
    **/
    virtual bool write(const uint8_t* data, size_t size) = 0;

    /**
    @brief Reads feedback from the linear joint
    @details Init must be called first to establish communication
    @return void
    **/
    virtual bool read(uint8_t* data, size_t size) = 0;

    /**
    @brief Reads goal position in cm
    @return int : scaled feedback value
    **/
    virtual int readScaledFeedback() = 0;

    /**
    @brief Sends position target command to linear motor
    @param target Target position value
    @return 0 : success in setting target of linear motor joint
    @return -1 : failure in setting target of linear motor joint
    **/
    virtual  int setTarget(uint16_t target)=0;
  };
}  // namespace linear
}  // namespace pandora_hardware_interface

#endif  // LINEAR_MOTOR_COM_INTERFACE_ABSTRACT_LINEAR_MOTOR_COM_INTERFACE_H
