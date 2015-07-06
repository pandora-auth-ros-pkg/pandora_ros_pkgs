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
* Author:     Konstantinos Panayiotou   <klpanagi@gmail.com>
* Maintainer: Konstantinos Panayiotou   <klpanagi@gmail.com>
*
*********************************************************************/

#ifndef EPOS2_GATEWAY_EPOS2_GATEWAY_H
#define EPOS2_GATEWAY_EPOS2_GATEWAY_H

#include <stdint.h>
#include "ros/ros.h"
#include "epos2_gateway/epos2_definitions.h"
#include <boost/scoped_ptr.hpp>
//#include <map>
//#include <cstring.h>

namespace pandora_hardware_interface
{
namespace motor
{

  struct Interface
  {
    char* deviceName;
    char* protocolStackName;
    char* interfaceName;
    char* portName;
    uint32_t baudrate;
    uint32_t timeout;
    uint32_t error;
  };


  class Epos2Gateway
  {
    private:
      boost::scoped_ptr<Interface> comInterface_;
      void* comHandler_;
      uint32_t error_;
    public:
      Epos2Gateway(const std::string port, const uint32_t baudrate,
        const uint32_t timeout, const std::string deviceName,
        const std::string protocolStackName, const std::string interfaceName);
      ~Epos2Gateway();

  //=================GATEWAY COMMUNICATION Methods=========================

      /*!
       * @brief Opens the port to sent and receive commands
       */
      void openDevice(void);

      /*!
       * @brief Closes the communication port
       */
      void closeDevice(void);

      bool eval_communicationParameters(void);

  //=======================STATE MACHINE Methods===========================

      /*!
       * @brief Sends the NMT serviec "Reset Node"
       * Command is without acknowledge
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t resetNode(uint16_t nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t setEnableState(uint16_t nodeId);

      /*!
       * @brief Changes the device's state to "Enable"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t setDisableState(uint16_t nodeId);

      /*!
       * @brief Changes the Epos2 Node state to "Quick Stop"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t setQuickStopState(uint16_t nodeId);

      /*!
       * @brief Changes the device state from "fault" to "disabled"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t clearFaultState(uint16_t nodeId);

      /*!
       * @brief Checks if the device is Enabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isEnableState(uint16_t nodeId);

      /*!
       * @brief Checks if the device is Disabled
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isDisableState(uint16_t nodeId);

      /*!
       * @brief Checks if the device is at "Quick Stop" state
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isQuickStopState(uint16_t nodeId);

      /*!
       * @brief Checks if the device is at "Fault" State
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      bool isFaultState(uint16_t nodeId);

      /*!
       * @brief Reads the state of the state machine
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t readState(uint16_t nodeId, uint16_t* nodeState);


  //=======================PROFILE VELOCITY MODE Methods===================

      /*!
       * @brief Changes the operation mode of an epos2 controller
       * to "Profile Velocity Mode"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      uint32_t activate_profileVelocityMode(uint16_t nodeId);

      /*!
       * @brief Sets the velocity profile parameters.
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus.
       * @param acceleration Velocity profile acceleration.
       * @param deceleration Velocity profile deceleration.
       */
      uint32_t set_profileVelocityParameters(uint16_t nodeId,
        uint16_t acceleration, uint16_t deceleration);

      /*!
       * @brief Returns the velocity profile parameters.
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus.
       * @param acceleration Velocity profile acceleration.
       * @param deceleration Velocity profile deceleration.
       * @return Error code for executing the command.
       *  If no errors recorded it returns zero 0.
       */
      uint32_t get_profileVelocityParameters(uint16_t nodeId,
        uint16_t* acceleration, uint16_t* deceleration);

      /*!
       * @brief Commands target velocity
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       * @param vel Target velocity (RPM).
       *
       */
      uint32_t set_targetVelocity(uint16_t nodeId, int32_t vel);

      /*!
       * @brief Stops the movement with profile deceleration
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t haltMovement(uint16_t nodeId);

      /*!
       * @brief Activates the velocity window.
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       * @param velWindow Velocity window value.
       * @param velWindowTime Velocity window time value.
       */
      uint32_t enableVelocityWindow(uint16_t nodeId, uint32_t velWindow,
        uint16_t velWindowTime);

      /*!
       * @brief Deactivates the velocity window.
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t disableVelocityWindow(uint16_t nodeId);

      /*!
       * @brief Reads the commaned target Velocity under profile
       * velocity mode.
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       * @param targetVel Commanded target velocity, given in rolls per
       * minute aka RPM.
       */
      uint32_t read_targetVelocity(uint16_t nodeId, int32_t* targetVel);

  //=======================CURRENT MODE Methods============================

      /*!
       * @brief Changes the operation mode of an epos2 controller
       * to "Profile Velocity Mode"
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t activate_currentMode(uint16_t nodeId);

      /*!
       * @brief Writes current mode setting value;
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t set_targetCurrent(uint16_t nodeId, int16_t currentMust);

      /*!
       * @brief Writes current mode setting value;
       * @param nodeId NodeID of the epos2 controller defined on CAN-Bus
       */
      uint32_t read_targetCurrent(uint16_t nodeId, int16_t* currentMust);
  //======================MOTION INFO Methods==============================

      /*!
       * @brief Reads the velocity actual value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      uint32_t read_velocityActual(uint16_t nodeId, int32_t* velActual);

      /*!
       * @brief Reads the velocity average value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      uint32_t read_velocityAvg(uint16_t nodeId, int32_t* velAvg);

      /*!
       * @brief Reads the current actual value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      uint32_t read_currentActual(uint16_t nodeId, int16_t* currentActual);

      /*!
       * @brief Reads the current average value 
       * param nodeId NodeID of the epos2 controller defined on CAN-Bus
       *
       *  If nodeId = 0 then command trasmits to everyone.
       */
      uint32_t read_currentAvg(uint16_t nodeId, int16_t* currentAvg);
  };
}  // namespace motor
}  // namespace pandora_hardware_interface
#endif  // EPOS2_GATEWAY_EPOS2_GATEWAY_H
