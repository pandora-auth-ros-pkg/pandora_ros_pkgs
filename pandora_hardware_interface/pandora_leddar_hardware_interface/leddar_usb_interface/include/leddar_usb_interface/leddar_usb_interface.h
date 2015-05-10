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

#ifndef LEDDAR_USB_INTERFACE_LEDDAR_USB_INTERFACE_H
#define LEDDAR_USB_INTERFACE_LEDDAR_USB_INTERFACE_H

#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include "leddar_usb_interface/LeddarC.h"
#include "leddar_usb_interface/LeddarProperties.h"

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

namespace pandora_hardware_interface
{
namespace leddar
{
  /**
   @class LeddarUsbInterface
   @brief Interface for communication with the leddar sensor
  **/
  class LeddarUSBInterface : private boost::noncopyable
  {
    public:
      /**
       @brief Default Constructor
      **/
      LeddarUSBInterface(
        std::string device,
        std::string lAddress);

      /**
       @brief Default Destructor
      **/
      ~LeddarUSBInterface();

      /**
       @brief Initializes the communication parameters and class variables
       @return void
      **/
      void init();

      /**
       @brief Pings the leddar sensor
       @return void
      **/
      void ping();

      /**
       @brief
       @return unsigned char : 1 to get called again or zero otherwise
      **/
      static unsigned char dataCallback(void* aHandle, unsigned int aLevels);

    private:
      std::string device_;  //!< port name the device is connected to
      std::string lAddress_;  //!< serial number of the sensor
    public:
      static LeddarHandle leddarHandle_;  //!< leddar handle
      static LdDetection* measurements_;  //!< leddar measurements struct
      static int leddarDetectionCount_;  //!< number of detections of sensor
  };
}  // namespace leddar
}  // namespace pandora_hardware_interface
#endif  // LEDDAR_USB_INTERFACE_LEDDAR_USB_INTERFACE_H
