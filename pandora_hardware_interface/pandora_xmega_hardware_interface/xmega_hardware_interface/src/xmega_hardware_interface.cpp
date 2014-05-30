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
#include "xmega_hardware_interface/xmega_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace xmega
{
  XmegaHardwareInterface::XmegaHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    serialInterface("/dev/ttyUSB0", 115200, 100)
  {
    serialInterface.init();

    // connect and register power supply interface
    registerBatteryInterface();

    // connect and register range sensor interface
    registerRangeSensorInterface();

    // connect and register joint state interface
    registerJointStateInterface();
  }

  XmegaHardwareInterface::~XmegaHardwareInterface()
  {
  }

  void XmegaHardwareInterface::read()
  {
    serialInterface.read();
    serialInterface.getBatteryData(&voltage_[0], &voltage_[1]);

    RangeMap sensorMap;
    sensorMap = serialInterface.getRangeData();
    bool exists[2] = {false, false};
    for (RangeMap::iterator it = sensorMap.begin(); it != sensorMap.end(); ++it)
    {
      int address = it->first;
      for (int jj = 0; jj < rangeData_.size(); jj++)
      {
        if ( address == i2c_address_[jj])
        {
          if (radiationType_[jj] == sensor_msgs::Range::ULTRASOUND)
          {
            range_[jj][bufferCounter_[jj]] =
              static_cast<double>(it->second.sonarRange) / 100;
            bufferCounter_[jj] = fmod(bufferCounter_[jj] + 1, 5);
            exists[0] = true;
            if (exists[1] == true)
            {
              break;
            }
          }
          else if (radiationType_[jj] == sensor_msgs::Range::INFRARED)
          {
            range_[jj][bufferCounter_[jj]] =
              static_cast<double>(it->second.irRange) / 100;
            bufferCounter_[jj] = fmod(bufferCounter_[jj] + 1, 5);
            exists[1] = true;
            if (exists[0] == true)
            {
              break;
            }
          }
        }
      }
    }
    double pi = boost::math::constants::pi<double>();
    double radians = serialInterface.getEncoderDegrees() / 180 * pi;
    // make radians value between [-pi, pi]
    if (radians > pi)
    {
      radians = radians - 2 * pi;
    }
    position_[0] = radians;
    position_[1] = -radians;
  }

  void XmegaHardwareInterface::registerBatteryInterface()
  {
    XmlRpc::XmlRpcValue batteryList;
    nodeHandle_.getParam("batteries", batteryList);
    ROS_ASSERT(
      batteryList.getType() == XmlRpc::XmlRpcValue::TypeArray);

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
      batteryNames_.push_back(
        static_cast<std::string>(batteryList[ii][key]));

      key = "max_voltage";
      ROS_ASSERT(
        batteryList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      voltage_[ii] = static_cast<double>(batteryList[ii][key]);

      BatteryHandle::Data data;
      data.name = batteryNames_[ii];
      data.voltage = &voltage_[ii];
      batteryData_.push_back(data);
      BatteryHandle handle(
        batteryData_[ii]);
      batteryInterface_.registerHandle(handle);
    }
    registerInterface(&batteryInterface_);
  }

  void XmegaHardwareInterface::registerRangeSensorInterface()
  {
    XmlRpc::XmlRpcValue rangeSensorList;
    nodeHandle_.getParam("range_sensors", rangeSensorList);
    ROS_ASSERT(
      rangeSensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    radiationType_ = new int[rangeSensorList.size()];
    fieldOfView_ = new double[rangeSensorList.size()];
    minRange_ = new double[rangeSensorList.size()];
    maxRange_ = new double[rangeSensorList.size()];
    range_ = new double*[rangeSensorList.size()];
    bufferCounter_ = new int[rangeSensorList.size()];
    i2c_address_ = new int[rangeSensorList.size()];


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
      frameId_.push_back(
        static_cast<std::string>(rangeSensorList[ii][key]));

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

      range_[ii] = new double[5];
      for (int jj = 0; jj < 5; jj++)
      {
        range_[ii][jj] = static_cast<double>(rangeSensorList[ii][key]);
      }

      bufferCounter_[ii] = 0;

      key = "i2c_address";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      i2c_address_[ii] = static_cast<int>(rangeSensorList[ii][key]);

      RangeSensorHandle::Data data;
      data.name = rangeSensorName_[ii];
      data.frameId = frameId_[ii];
      data.radiationType = &radiationType_[ii];
      data.fieldOfView = &fieldOfView_[ii];
      data.minRange = &minRange_[ii];
      data.maxRange = &maxRange_[ii];
      data.range = range_[ii];
      rangeData_.push_back(data);
      RangeSensorHandle handle(
        rangeData_[ii]);
      rangeSensorInterface_.registerHandle(handle);
    }
    registerInterface(&rangeSensorInterface_);
  }
  void XmegaHardwareInterface::registerJointStateInterface()
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
}  // namespace xmega
}  // namespace pandora_hardware_interface
