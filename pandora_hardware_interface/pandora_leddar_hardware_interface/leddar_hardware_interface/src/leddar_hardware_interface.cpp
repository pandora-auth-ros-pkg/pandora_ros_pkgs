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

#include "leddar_hardware_interface/leddar_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace leddar
{
  LeddarHardwareInterface::LeddarHardwareInterface(ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    leddarSerialInterface_("leddar", "ttyS0", 1)
  {
    // initialize serial communication
    leddarSerialInterface_.init();
    // connect and register leddar interface
    registerLeddarSensorInterface();
  }

  LeddarHardwareInterface::~LeddarHardwareInterface()
  {
  }

  void LeddarHardwareInterface::read()
  {
    leddarSerialInterface_.read();
    // get the measurements from the serial interface object
    LtAcquisition* lAcquisition = leddarSerialInterface_.getLAcquisition();
    *leddarDetectionCount_ = lAcquisition->mDetectionCount;
    for (int ii = 0; ii < *leddarDetectionCount_; ii++)
    {
      leddarDistances_[ii] = lAcquisition->mDetections[ii].mDistance;
      leddarAmplitudes_[ii] = lAcquisition->mDetections[ii].mAmplitude;
    }
  }

  void LeddarHardwareInterface::registerLeddarSensorInterface()
  {
    XmlRpc::XmlRpcValue leddarSensorList;
    nodeHandle_.getParam("leddar_sensor", leddarSensorList);
    ROS_ASSERT(leddarSensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    leddarDetectionCount_ = new int;
    leddarDistances_ = new float[LEDDAR_MAX_DETECTIONS];
    leddarAmplitudes_ = new float[LEDDAR_MAX_DETECTIONS];

    ROS_ASSERT(
      leddarSensorList[0].getType() == XmlRpc::XmlRpcValue::TypeStruct);

    std::string key;

    key = "name";
    ROS_ASSERT(
      leddarSensorList[0][key].getType() == XmlRpc::XmlRpcValue::TypeString);
    name_ = static_cast<std::string>(leddarSensorList[0][key]);

    key = "frame_id";
    ROS_ASSERT(
      leddarSensorList[0][key].getType() == XmlRpc::XmlRpcValue::TypeString);
    frameId_ = static_cast<std::string>(leddarSensorList[0][key]);

    LeddarSensorHandle::Data data;
    data.name = name_;
    data.frameId = frameId_;
    data.leddarDetectionCount =  leddarDetectionCount_;
    data.leddarDistances = leddarDistances_;
    data.leddarAmplitudes = leddarAmplitudes_;
    // create leddar sensor handle
    leddarSensorData_ = data;
    LeddarSensorHandle leddarSensorHandle(leddarSensorData_);
    // register the handle on the leddar sensor interface
    leddarSensorInterface_.registerHandle(leddarSensorHandle);
    // register the leddar sensor interface
    registerInterface(&leddarSensorInterface_);
  }
}  // namespace leddar
}  // namespace pandora_hardware_interface
