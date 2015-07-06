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

#ifndef LINEAR_ACTUATOR_COM_INTERFACE_FIRGELLI_COM_INTERFACE_H
#define LINEAR_ACTUATOR_COM_INTERFACE_FIRGELLI_COM_INTERFACE_H

#include <assert.h>
#include <libusb-1.0/libusb.h>
#include "linear_actuator_com_interface/abstract_linear_actuator_com_interface.h"
#include "linear_actuator_com_interface/firgelli_definitions.h"
namespace pandora_hardware_interface
{
namespace linear_actuator
{
  /**
  @class FirgelliComInterface
  @brief Class used for communicating with the firgelli linear actuator via usb
  **/
  class FirgelliComInterface : public AbstractLinearActuatorComInterface
  {
   public:
    /**
    @brief Default Constructor
    **/
    FirgelliComInterface();

    /**
    @brief Default Destructor
    **/
    ~FirgelliComInterface();

    /**
    @brief Opens communication port and performs other initialization tasks
    @return void
    **/
    void init();


    /**
    @brief Opens communication port
    @return void
    **/
    void openDevice();

    /**
    @brief Closes opened communication port
    @brief This method is used to close linear actuator communication port
    **/
    void closeDevice();

    /**
    @brief Write goal position to linear actuator
    @return bool : 1 for success, 0 for error
    **/
    bool write(const uint8_t* data, size_t size);

    /**
    @brief Gets position feedback from the linear joint
    @details Init must be called first to establish communication
    @return void
    **/
    bool read(uint8_t* data, size_t size);

    /**
    @brief Reads goal position in cm
    @return int : scaled feedback value
    **/
    int readScaledFeedback();


    /**
    @brief Sends position target command to linear actuator
    @param target Target position value
    @return 0 : success in setting target of linear actuator joint
    @return -1 : failure in setting target of linear actuator joint
    **/
    int setTarget(uint16_t target);

   private:
    /**
    @brief Returns control type
    @param attr [int] : attribute
    @return static const char* : control type
    **/
    static const char* controlType(int attr);

   private:
    int mInterface_;
    libusb_device_handle *mHandle_;
    struct libusb_context *mCtx_;
    static int mDebug_;
    int rank_;
  };
}  // namespace linear_actuator
}  // namespace pandora_hardware_interface
#endif  // LINEAR_ACTUATOR_COM_INTERFACE_FIRGELLI_COM_INTERFACE_H
