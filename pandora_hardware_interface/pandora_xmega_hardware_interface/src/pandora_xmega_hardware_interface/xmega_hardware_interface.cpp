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
#include "pandora_xmega_hardware_interface/xmega_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace xmega
{
  XmegaHardwareInterface::XmegaHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle),
    serialInterface(
      "/dev/ttyUSB0",
      115200,
      100)
  {
    // commented for testing
    serialInterface.init();

    // connect and register power supply interface
    registerPowerSupplyInterface();

    // connect and register power supply interface
    registerRangeSensorInterface();
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
      for (int jj = 2; jj < rangeData_.size(); jj++)
      {
        if ( address == i2c_address_[jj])
        {
          if (radiationType_[jj] == sensor_msgs::Range::ULTRASOUND)
          {
            range_[jj][bufferCounter_[jj]] =
              static_cast<double>(it->second.sonarRange);
            bufferCounter_[jj] = fmod(bufferCounter_[jj]++, 5);
            exists[0] = true;
            if (exists[1] == true)
            {
              break;
            }
          }
          else if (radiationType_[jj] == sensor_msgs::Range::INFRARED)
          {
            range_[jj][bufferCounter_[jj]] =
              static_cast<double>(it->second.irRange);
            bufferCounter_[jj] = fmod(bufferCounter_[jj]++, 5);
            exists[1] = true;
            if (exists[0] == true)
            {
              break;
            }
          }
        }
      }

      std::vector<std::string> prefix;
      std::string str;
      str = "/sensors/sonar";
      prefix.push_back(str);
      str = "/sensors/ir";
      prefix.push_back(str);

      for (int jj = 0; jj < 2; jj++)
      {
        if (!exists[jj])
        {
          rangeSensorName_.push_back(prefix[jj] + boost::lexical_cast<std::string>(address));
          frameId_.push_back(frameId_[jj]);
          radiationType_.push_back(jj);
          fieldOfView_.push_back(fieldOfView_[jj]);
          minRange_.push_back(minRange_[jj]);
          maxRange_.push_back(maxRange_[jj]);
          range_.push_back(range_[jj]);
          bufferCounter_.push_back(bufferCounter_[jj]);
          i2c_address_.push_back(address);

          RangeSensorHandle::Data data;
          int kk = rangeData_.size();
          data.name = rangeSensorName_[kk];
          data.frameId = frameId_[kk];
          data.radiationType = &radiationType_[kk];
          data.fieldOfView = &fieldOfView_[kk];
          data.minRange = &minRange_[kk];
          data.maxRange = &maxRange_[kk];
          data.range = &range_[kk];
          rangeData_.push_back(data);

          RangeSensorHandle handle(
              rangeData_[kk]);
          rangeSensorInterface_.registerHandle(handle);
        }
      }
    }
  }

  void XmegaHardwareInterface::registerPowerSupplyInterface()
  {
    XmlRpc::XmlRpcValue powerSupplyList;
    nodeHandle_.getParam("power_supplies", powerSupplyList);
    ROS_ASSERT(
      powerSupplyList.getType() == XmlRpc::XmlRpcValue::TypeArray);

    std::vector<PowerSupplyHandle>
      powerSupplyHandle;
    std::string key;
    for (int ii = 0; ii < powerSupplyList.size(); ii++)
    {
      ROS_ASSERT(
        powerSupplyList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

      key = "name";
      ROS_ASSERT(
        powerSupplyList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      powerSupplyNames_.push_back(
        static_cast<std::string>(powerSupplyList[ii][key]));

      key = "max_voltage";
      ROS_ASSERT(
        powerSupplyList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      voltage_.push_back(
        static_cast<double>(powerSupplyList[ii][key]));

      PowerSupplyHandle::Data data;
      data.name = powerSupplyNames_[ii];
      data.voltage = &voltage_[ii];
      powerSupplyData_.push_back(data);
      PowerSupplyHandle handle(
        powerSupplyData_[ii]);
      powerSupplyInterface_.registerHandle(handle);
    }
    registerInterface(&powerSupplyInterface_);
  }

  void XmegaHardwareInterface::registerRangeSensorInterface()
  {
    XmlRpc::XmlRpcValue rangeSensorList;
    nodeHandle_.getParam("range_sensors", rangeSensorList);
    ROS_ASSERT(
      rangeSensorList.getType() == XmlRpc::XmlRpcValue::TypeArray);

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
      radiationType_.push_back(
        static_cast<int>(rangeSensorList[ii][key]));

      key = "field_of_view";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      fieldOfView_.push_back(
        static_cast<double>(rangeSensorList[ii][key]));

      key = "min_range";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      minRange_.push_back(
        static_cast<double>(rangeSensorList[ii][key]));

      key = "max_range";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      maxRange_.push_back(
        static_cast<double>(rangeSensorList[ii][key]));

      boost::array<double, 5> initialRange;
      initialRange[0] = initialRange[1] =
        initialRange[2] = initialRange[3] =
        initialRange[4] = static_cast<double>(rangeSensorList[ii][key]);
      range_.push_back(initialRange);
      bufferCounter_.push_back(0);

      key = "i2c_address";
      ROS_ASSERT(
        rangeSensorList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      i2c_address_.push_back(
        static_cast<int>(rangeSensorList[ii][key]));

      RangeSensorHandle::Data data;
      data.name = rangeSensorName_[ii];
      data.frameId = frameId_[ii];
      data.radiationType = &radiationType_[ii];
      data.fieldOfView = &fieldOfView_[ii];
      data.minRange = &minRange_[ii];
      data.maxRange = &maxRange_[ii];
      data.range = &range_[ii];
      rangeData_.push_back(data);
      if (ii > 1)
      {
        RangeSensorHandle handle(
          rangeData_[ii]);
        rangeSensorInterface_.registerHandle(handle);
      }
    }
    registerInterface(&rangeSensorInterface_);
  }
}  // namespace xmega
}  // namespace pandora_hardware_interface
