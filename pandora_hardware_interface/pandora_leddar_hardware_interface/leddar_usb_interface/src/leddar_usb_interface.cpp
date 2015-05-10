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

#include <leddar_usb_interface/leddar_usb_interface.h>


namespace pandora_hardware_interface
{
namespace leddar
{
  LeddarUSBInterface::LeddarUSBInterface(
    std::string device,
    std::string address)
  :
    device_(device),
    lAddress_(address)
  {
    measurements_ = new LdDetection[50];
  }

  LeddarUSBInterface::~LeddarUSBInterface()
  {
    LeddarStopDataTransfer(leddarHandle_);
    LeddarRemoveCallback(leddarHandle_, dataCallback, leddarHandle_);

    LeddarDisconnect(leddarHandle_);
    LeddarDestroy(leddarHandle_);
  }

  void LeddarUSBInterface::init()
  {
    leddarHandle_ = LeddarCreate();

    if (LeddarConnect(leddarHandle_, lAddress_.c_str()) == LD_SUCCESS)
    {
      // Spawn a worker thread that continuously reads the sensor
      int status = LeddarStartDataTransfer(leddarHandle_, LDDL_DETECTIONS);
      if (status == LD_SUCCESS)
      {
        // spawn a worker thread that activates for receiving the measurements
        status = LeddarAddCallback(leddarHandle_, dataCallback, leddarHandle_);
      }
      if (status != LD_SUCCESS)
      {
        LtChar lMessage[200];
        LeddarGetErrorMessage(status, lMessage, ARRAY_LEN(lMessage));
        ROS_FATAL(LTS("LeddarC error (%d): %s\n"), status, lMessage);
        exit(-1);
      }
    }
    else
    {
      ROS_FATAL("[leddar] Connection failed.");
      exit(-1);
    }
  }

  void LeddarUSBInterface::ping()
  {
    if (!LeddarGetConnected(leddarHandle_) &&
      LeddarPing(leddarHandle_) != LD_SUCCESS)
    {
      ROS_ERROR("[leddar] Communication Error");
      exit(-1);
    }
  }

  unsigned char LeddarUSBInterface::
    dataCallback(void* aHandle, unsigned int aLevels)
  {
    LdDetection lDetections[50];

    leddarDetectionCount_ = LeddarGetDetectionCount(aHandle);

    if (leddarDetectionCount_ > ARRAY_LEN(lDetections))
    {
      leddarDetectionCount_ = ARRAY_LEN(lDetections);
    }

    LeddarGetDetections(aHandle, lDetections, ARRAY_LEN(lDetections));

    for (int ii = 0; ii < leddarDetectionCount_; ii++)
    {
      measurements_[ii] = lDetections[ii];
    }

    return 1;  // 1->gets called again, 0->gets called only once
  }

}  // namespace leddar
}  // namespace pandora_hardware_interface
