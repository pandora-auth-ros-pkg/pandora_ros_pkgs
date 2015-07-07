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
* Author:  George Kouros
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
    try
    {
      arm_ = new ArmUsbInterface();
    }
    catch(std::exception& ex)
    {
      ROS_ERROR("%s", ex.what());
      exit(-1);
    }
    // connect and register co2 sensor interface
    registerCo2SensorInterface();

    // connect and register thermal sensor interface
    registerThermalSensorInterface();

    // connect and register range sensor interface
    registerRangeSensorInterface();

    // connect and register battery interface
    registerBatteryInterface();

    // connect and register joint state interface
    registerJointStateInterface();
  }

  ArmHardwareInterface::~ArmHardwareInterface()
  {
    delete arm_;
  }

  void ArmHardwareInterface::read()
  {
    uint8_t *u8Array = new uint8_t[GEYE_NBYTES];
    uint16_t uValue;
    float fValue;

    // ROS_INFO("Will read CO2");
    // read CO2 percentage from CO2 sensors
    for (int ii = 0; ii < co2SensorName_.size(); ii++)
    {
      if (arm_->readCo2Value(&fValue) > 0)
        co2Percentage_[ii] = fValue;
    }

    // read thermal image from grideye sensors
    for (int ii = 0; ii < thermalSensorName_.size(); ii++)
    {
      // ROS_INFO("Will read GEYE loop/n");
      if (arm_->readGrideyeValues(thermalSensorCode_[ii], u8Array) > 0)
        memcpy(thermalData_[ii], u8Array, GEYE_NBYTES * sizeof(uint8_t));
    }

    // ROS_INFO("Will read SONARS");
    // read distances from range sensors
    for (int ii = 0; ii < rangeSensorName_.size(); ii++)
    {
      if (arm_->readSonarValues(rangeSensorCode_[ii], &uValue) > 0)
        range_[ii] = static_cast<double>(uValue) / 100;
    }
    // ROS_INFO("Will read BATTERIES");
    // read voltage of batteries
    for (int ii = 0; ii < batteryName_.size(); ii++)
    {
        if (arm_->readBatteryValues(batteryCode_[ii], &uValue) > 0 && uValue > 12.0)
          voltage_[ii] = uValue / 4096.0 * 33.0;
    }
    // read encoder degrees
    if (arm_->readEncoderValue(&uValue) > 0)
    {
      double degrees = static_cast<double>(uValue) * 360.0 / 1024.0;
      double radians =
        degrees / 180.0 * boost::math::constants::pi<double>() +
        encoder_offset_;

      // make radians value between [-pi, pi]
      if (radians > boost::math::constants::pi<double>())
        radians = radians - 2 * boost::math::constants::pi<double>();

      position_[0] = radians;
      position_[1] = -radians;
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
    thermalSensorCode_ = new char[thermalSensorList.size()];

    std::string key;
    for (int ii = 0; ii < thermalSensorList.size(); ii++)
    {
      ROS_ASSERT(
        thermalSensorList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() ==
          XmlRpc::XmlRpcValue::TypeString);
      thermalSensorName_.push_back(
        static_cast<std::string>(thermalSensorList[ii][key]));

      key = "frame_id";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() ==
          XmlRpc::XmlRpcValue::TypeString);
      thermalFrameId_.push_back(
        static_cast<std::string>(thermalSensorList[ii][key]));

      key = "code";
      ROS_ASSERT(
        thermalSensorList[ii][key].getType() ==
          XmlRpc::XmlRpcValue::TypeString);
      thermalSensorCode_[ii] =
        static_cast<std::string>(thermalSensorList[ii][key]).at(0);

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


  void ArmHardwareInterface::registerRangeSensorInterface()
  {
    XmlRpc::XmlRpcValue rangeSensorList;
    nodeHandle_.getParam("range_sensors", rangeSensorList);
    ROS_ASSERT(
      rangeSensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    rangeSensorCode_ = new char[rangeSensorList.size()];
    radiationType_ = new int[rangeSensorList.size()];
    fieldOfView_ = new double[rangeSensorList.size()];
    minRange_ = new double[rangeSensorList.size()];
    maxRange_ = new double[rangeSensorList.size()];
    range_ = new double[rangeSensorList.size()];

    std::string key;
    for (int ii = 0; ii < rangeSensorList.size(); ii++)
    {
      ROS_ASSERT(
        rangeSensorList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      rangeSensorName_.push_back(
        static_cast<std::string>(rangeSensorList[ii][key]));

      key = "frame_id";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      rangeSensorFrameId_.push_back(
        static_cast<std::string>(rangeSensorList[ii][key]));

      key = "code";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      rangeSensorCode_[ii] =
        static_cast<std::string>(rangeSensorList[ii][key]).at(0);

      key = "radiation_type";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      radiationType_[ii] = static_cast<int>(rangeSensorList[ii][key]);

      key = "field_of_view";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      fieldOfView_[ii] = static_cast<double>(rangeSensorList[ii][key]);

      key = "min_range";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      minRange_[ii] = static_cast<double>(rangeSensorList[ii][key]);

      key = "max_range";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      maxRange_[ii] = static_cast<double>(rangeSensorList[ii][key]);

      RangeSensorHandle::Data data;
      data.name = rangeSensorName_[ii];
      data.frameId = rangeSensorFrameId_[ii];
      data.radiationType = &radiationType_[ii];
      data.fieldOfView = &fieldOfView_[ii];
      data.minRange = &minRange_[ii];
      data.maxRange = &maxRange_[ii];
      data.range = &range_[ii];
      rangeSensorData_.push_back(data);
      RangeSensorHandle handle(rangeSensorData_[ii]);
      rangeSensorInterface_.registerHandle(handle);
    }
    registerInterface(&rangeSensorInterface_);
  }

  void ArmHardwareInterface::registerBatteryInterface()
  {
    XmlRpc::XmlRpcValue batteryList;
    nodeHandle_.getParam("batteries", batteryList);
    ROS_ASSERT(
      batteryList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    batteryCode_ = new char[batteryList.size()];
    voltage_ = new double[batteryList.size()];

    std::vector<BatteryHandle>
      batteryHandle;
    std::string key;
    for (int ii = 0; ii < batteryList.size(); ii++)
    {
      ROS_ASSERT(
        batteryList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      batteryName_.push_back(static_cast<std::string>(batteryList[ii][key]));

      key = "code";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      batteryCode_[ii] = (static_cast<std::string>(batteryList[ii][key])).at(0);

      key = "max_voltage";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      voltage_[ii] = static_cast<double>(batteryList[ii][key]);

      BatteryHandle::Data data;
      data.name = batteryName_[ii];
      data.voltage = &voltage_[ii];
      batteryData_.push_back(data);
      BatteryHandle handle(batteryData_[ii]);
      batteryInterface_.registerHandle(handle);
    }
    registerInterface(&batteryInterface_);
  }


  void ArmHardwareInterface::registerJointStateInterface()
  {
    // read joint names from param server
    std::string name;
    nodeHandle_.getParam(
      "differential_joints/left_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "differential_joints/right_joint",
      name);
    jointNames_.push_back(name);

    nodeHandle_.getParam(
      "differential_joints/encoder_offset",
      encoder_offset_);

    // connect and register the joint state interface
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      position_[ii] = 0;
      velocity_[ii] = 0;
      effort_[ii] = 0;
      hardware_interface::JointStateHandle jointStateHandle(
        jointNames_[ii],
        &position_[ii],
        &velocity_[ii],
        &effort_[ii]);
      jointStateInterface_.registerHandle(jointStateHandle);
    }
    registerInterface(&jointStateInterface_);
  }

}  // namespace arm
}  // namespace pandora_hardware_interface
