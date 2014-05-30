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
* Author:  Evangelos Apostolidis
*********************************************************************/
#include "arm_hardware_interface/arm_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace arm
{
  ArmHardwareInterface::ArmHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    // connect and register co2 sensor interface
    registerCo2SensorInterface();

    // connect and register thermal sensor interface
    registerThermalSensorInterface();
  }

  ArmHardwareInterface::~ArmHardwareInterface()
  {
  }

  void ArmHardwareInterface::read()
  {
    for (int ii = 0; ii < co2SensorName_.size(); ii++)
    {
      co2Percentage_[ii] = arm_.co2ValueGet();
    }

    for (int ii = 0; ii < thermalSensorName_.size(); ii++)
    {
      arm_.grideyeValuesGet(*address_[ii].c_str(), thermalData_[ii]);
    }
  }

  void ArmHardwareInterface::registerCo2SensorInterface()
  {
    XmlRpc::XmlRpcValue co2SensorList;
    nodeHandle_.getParam("co2_sensors", co2SensorList);
    ROS_ASSERT(
      co2SensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    co2Percentage_ = new float[co2SensorList.size()];
    std::string key;
    for (int ii = 0; ii < co2SensorList.size(); ii++)
    {
      ROS_ASSERT(
        co2SensorList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        co2SensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      co2SensorName_.push_back(
        static_cast<std::string>(co2SensorList[ii][key]));

      key = "frame_id";
      ROS_ASSERT(
        co2SensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      co2SensorFrameId_.push_back(
        static_cast<std::string>(co2SensorList[ii][key]));

      co2Percentage_[ii] = 0;

      Co2SensorHandle::Data data;
      data.name = co2SensorName_[ii];
      data.frameId = co2SensorFrameId_[ii];
      data.co2Percentage = &co2Percentage_[ii];
      co2SensorData_.push_back(data);
      Co2SensorHandle handle(
        co2SensorData_[ii]);
      co2SensorInterface_.registerHandle(handle);
    }
    registerInterface(&co2SensorInterface_);
  }

  void ArmHardwareInterface::registerThermalSensorInterface()
  {
    XmlRpc::XmlRpcValue thermalSensorList;
    nodeHandle_.getParam("thermal_sensors", thermalSensorList);
    ROS_ASSERT(
      thermalSensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    height_ = new int[thermalSensorList.size()];
    width_ = new int[thermalSensorList.size()];
    step_ = new int[thermalSensorList.size()];
    thermalData_ = new uint8_t*[thermalSensorList.size()];

    std::string key;
    for (int ii = 0; ii < thermalSensorList.size(); ii++)
    {
      ROS_ASSERT(
        thermalSensorList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      thermalSensorName_.push_back(
        static_cast<std::string>(thermalSensorList[ii][key]));

      key = "frame_id";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      thermalFrameId_.push_back(
        static_cast<std::string>(thermalSensorList[ii][key]));

      key = "height";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      height_[ii] = static_cast<int>(thermalSensorList[ii][key]);

      key = "width";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      width_[ii] = thermalSensorList[ii][key];

      key = "step";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      step_[ii] = thermalSensorList[ii][key];

      thermalData_[ii] = new uint8_t[step_[ii] * height_[ii]];
      for (int jj = 0; jj < step_[ii] * height_[ii]; jj++)
      {
        thermalData_[ii][jj] = 0;
      }

      key = "address";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      address_.push_back(
        static_cast<std::string>(thermalSensorList[ii][key]));

      ThermalSensorHandle::Data data;
      data.name = thermalSensorName_[ii];
      data.frameId = thermalFrameId_[ii];
      data.height = &height_[ii];
      data.width = &width_[ii];
      data.step = &step_[ii];
      data.data = thermalData_[ii];
      thermalSensorData_.push_back(data);
      ThermalSensorHandle handle(
        thermalSensorData_[ii]);
      thermalSensorInterface_.registerHandle(handle);
    }
    registerInterface(&thermalSensorInterface_);
  }
}  // namespace arm
}  // namespace pandora_hardware_interface
