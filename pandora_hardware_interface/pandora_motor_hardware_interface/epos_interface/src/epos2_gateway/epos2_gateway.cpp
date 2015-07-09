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
*********************************************************************/
#include "epos2_gateway/epos2_gateway.h"
#include "Utils/Utils.h"

namespace pandora_hardware_interface
{
namespace motor
{
  Epos2Gateway::Epos2Gateway(const std::string port,
    const uint32_t baudrate, const uint32_t timeout,
    const std::string deviceName, const std::string protocolStackName,
    const std::string interfaceName)
  {
    comInterface_.reset(new Interface());
    comInterface_->deviceName = new char[32];
    comInterface_->protocolStackName = new char[32];
    comInterface_->interfaceName = new char[32];
    comInterface_->portName = new char[32];
    strcpy(comInterface_->deviceName, deviceName.c_str());
    strcpy(comInterface_->protocolStackName, protocolStackName.c_str());
    strcpy(comInterface_->interfaceName, interfaceName.c_str());
    strcpy(comInterface_->portName, port.c_str());
    comInterface_->baudrate = baudrate;
    comInterface_->timeout = timeout;
    // TODO(klpanagi): --- Read number of Nodes from a yaml
  }

  Epos2Gateway::~Epos2Gateway()
  {
    // Deleting void pointer is undefined;
    // delete comHandler_;
  }

  // =================<GATEWAY COMMUNICATION Methods>=====================
  // #####################################################################

  bool Epos2Gateway::openDevice(void)
  {
    /*--<Open Device Communication Port>--*/
    comHandler_ = VCS_OpenDevice(comInterface_->deviceName,
      comInterface_->protocolStackName, comInterface_->interfaceName,
      comInterface_->portName, &error_);

    return ( ( comHandler_ != 0 || error_ == 0) && 
      eval_communicationParameters() == 0 );
  }

  bool Epos2Gateway::closeDevice(void)
  {
    return (VCS_CloseDevice(comHandler_, &error_) !=0 && error_ == 0)
      ? true : false;
  }


  bool Epos2Gateway::eval_communicationParameters(void)
  {
    uint32_t _baudrate;
    uint32_t _timeout;

    int condition = ( ( VCS_GetProtocolStackSettings(comHandler_, 
      &_baudrate, &_timeout, &error_) != 0 ) &&
      ( VCS_SetProtocolStackSettings(comHandler_, 
      comInterface_->baudrate, comInterface_->timeout, &error_) != 0 )
      && ( VCS_GetProtocolStackSettings(comHandler_, &_baudrate,
      &_timeout, &error_) != 0 ) );

    return condition;
  }


  void Epos2Gateway::loadErrorCodes(const std::string fileUri)
  {
    errorCodes_ = Utils::readErrorCodesMap(fileUri);
  }
  // =======================<STATE MACHINE Methods>=======================
  // #####################################################################

  uint32_t Epos2Gateway::resetNode(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_ResetDevice(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- isEnableState, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Motors]: Reset NodeId[%d]", nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::setEnableState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_SetEnableState(comHandler_, nodeId, &_errorCode) == 0)
    {
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- setEnableState, nodeId=%d",
        _errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at {Enabled-State}", nodeId);
    }
    return _errorCode;
  }


  bool Epos2Gateway::isEnableState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    int isEnabled = 0;
    // isEnabled => 1: Device Enabled, 0: Device NOT Enabled
    if (VCS_GetEnableState(comHandler_, nodeId, &isEnabled, &_errorCode) == 0)
    
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      //ROS_FATAL("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        //"execution -- isEnableState, nodeId=%d",
        //_errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      if (isEnabled)
      {
        // Device is at Enabled State
        return true;
      }
      else
      {
        // Device is NOT at Enabled State
        return false;
      }
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::setDisableState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_SetDisableState(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- setDisableState, nodeId=%d",
        _errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      ROS_INFO("[Motors]: NodeId[%d] is set at Disabled State", nodeId);
    }
    return _errorCode;
  }


  bool Epos2Gateway::isDisableState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    int isDisabled = 0;
    if (VCS_GetDisableState(comHandler_, nodeId, &isDisabled, 
        &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      //ROS_FATAL("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        //"execution -- isDisableState, nodeId=%d",
        //_errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      if (isDisabled)
      {
        // Device is at Disabled State
        return true;
      }
      else
      {
        // Device is NOT at Disabled State
        return false;
      }
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::clearFaultState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_ClearFault(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- isFaultState, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Motors]: Cleared Fault state of NodeId[%d]", nodeId);
    }
    return _errorCode;
  }


  bool Epos2Gateway::isFaultState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    int isFault = 0;
    if (VCS_GetFaultState(comHandler_, nodeId, &isFault, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- isFaultState, nodeId=%d",
        _errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      if (isFault)
      {
        // Device is at Fault State
        return true;
      }
      else
      {
        return false;
      }
    }
    return _errorCode;
  }


  bool Epos2Gateway::isQuickStopState(uint16_t nodeId)
  {
    uint32_t _errorCode;
    int isQuickStopped = 0;
    if (VCS_GetQuickStopState(comHandler_, nodeId, &isQuickStopped,
        &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("\033[0m[Epos2-Gateway]: Received error {%d} on command "
        "execution -- isQuickStopState, nodeId=%d",
        _errorCode, nodeId);
    }
    else  // Command executed succesfully
    {
      if (isQuickStopped)
      {
        // Device is at "QuickStop" State
        return true;
      }
      else
      {
        return false;
      }
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::setQuickStopState(uint16_t nodeId)
  {
    uint32_t _errorCode = 0;
    if (VCS_SetQuickStopState(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- setQuickStopState, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Epos2-gateway]: NodeId[%d] is set at QuickStop state", nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::readState(uint16_t nodeId, uint16_t* nodeState)
  {
    uint32_t _errorCode = 0;
    if (VCS_GetState(comHandler_, nodeId, nodeState, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- readState, nodeId=%d",
        _errorCode, nodeId);
      *nodeState = 9;
    }
    return _errorCode;
  }

  // =======================PROFILE VELOCITY MODE Methods=================
  // #####################################################################

  uint32_t Epos2Gateway::activate_profileVelocityMode(uint16_t nodeId)
  {
    uint32_t _errorCode = 0;
    if (VCS_ActivateProfileVelocityMode(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- activate_profileVelocity, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Epos2-Gateway]: NodeId[%d] is set at ProfileVelocity Mode", nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::set_targetVelocity(uint16_t nodeId, int32_t vel)
  {
    uint32_t _errorCode;
    if (VCS_MoveWithVelocity(comHandler_, nodeId, vel, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- moveWithVelocity, nodeId=%d", _errorCode, nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::set_profileVelocityParameters(uint16_t nodeId,
    uint16_t acceleration, uint16_t deceleration)
  {
    uint32_t _errorCode;
    if (VCS_SetVelocityProfile(comHandler_, nodeId,
        acceleration, deceleration, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_FATAL("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- setVelocityProfile, nodeId=%d",
        _errorCode, nodeId);
    }
    return _errorCode;
  }

  uint32_t Epos2Gateway::get_profileVelocityParameters(uint16_t nodeId,
    uint16_t* acceleration, uint16_t* deceleration)
  {
    uint32_t _acc, _dec;
    uint32_t _errorCode;
    if (VCS_GetVelocityProfile(
      comHandler_, nodeId, &_acc, &_dec, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- getVelocityProfile, nodeId=%d",
        _errorCode, nodeId);
    }
    *acceleration = static_cast<uint16_t>(_acc);
    *deceleration = static_cast<uint16_t>(_dec);
    return _errorCode;
  }

  uint32_t Epos2Gateway::haltMovement(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_HaltVelocityMovement(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- haltVelocityMovement, nodeId=%d",
        _errorCode, nodeId);
    }
    return _errorCode;
  }

  uint32_t Epos2Gateway::enableVelocityWindow(uint16_t nodeId,
    uint32_t velWindow, uint16_t velWindowTime)
  {
    uint32_t _errorCode;
    if (VCS_EnableVelocityWindow(
      comHandler_, nodeId, velWindow, velWindowTime, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- enableVelocityWindow, nodeId=%d",
        _errorCode, nodeId);
    }
    return _errorCode;
  }

  uint32_t Epos2Gateway::disableVelocityWindow(uint16_t nodeId)
  {
    uint32_t _errorCode;
    if (VCS_DisableVelocityWindow(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- disableVelocityWindow, nodeId=%d",
        _errorCode, nodeId);
    }
    return _errorCode;
  }

  uint32_t Epos2Gateway::read_targetVelocity(uint16_t nodeId, int32_t* targetVel)
  {
    uint32_t _errorCode;
    int64_t _targetVel;
    // input value for target velocity should be int32_t type according to
    // the EPOS_COMMAND_LIBRARY but it aint -- BUG_REPORT
    if (VCS_GetTargetVelocity(comHandler_, nodeId, &_targetVel, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- disableVelocityWindow, nodeId=%d",
        _errorCode, nodeId);
    }
    *targetVel = static_cast<int32_t>(_targetVel);
    return _errorCode;
  }


  // =======================CURRENT MODE Methods==========================
  // #####################################################################


  uint32_t Epos2Gateway::activate_currentMode(uint16_t nodeId)
  {
    uint32_t _errorCode = 0;
    if (VCS_ActivateCurrentMode(comHandler_, nodeId, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- activate_profileVelocity, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Epos2-Gateway]: NodeId[%d] is set at ProfileVelocity Mode", 
        nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::set_targetCurrent(uint16_t nodeId, int16_t currentMust)
  {
    ROS_INFO("%d", currentMust);
    uint32_t _errorCode = 0;
    if (VCS_SetCurrentMust(comHandler_, nodeId, currentMust, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- activate_profileVelocity, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Epos2-Gateway]: NodeId[%d] is set at ProfileVelocity Mode", 
        nodeId);
    }
    return _errorCode;
  }

  uint32_t Epos2Gateway::read_targetCurrent(uint16_t nodeId, int16_t* currentMust)
  {
    uint32_t _errorCode = 0;
    if (VCS_GetCurrentMust(comHandler_, nodeId, currentMust, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve errorCode
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution -- activate_profileVelocity, nodeId=%d",
        _errorCode, nodeId);
    }
    else
    {
      ROS_INFO("[Epos2-Gateway]: NodeId[%d] is set at ProfileVelocity Mode", 
        nodeId);
    }
    return _errorCode;
  }

  // ======================MOTION INFO Methods============================
  // #####################################################################

  uint32_t Epos2Gateway::read_velocityActual(uint16_t nodeId, int32_t* velActual)
  {
    uint32_t _errorCode = 0;
    if (VCS_GetVelocityIs(comHandler_, nodeId, velActual, &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_FATAL("[Epos2-Gateway]: Received error {%d} on command "
        "execution --getVelocityIs, nodeId=%d", _errorCode, nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::read_velocityAvg(uint16_t nodeId, int32_t* velAvg)
  {
    uint32_t _errorCode;
    if (VCS_GetVelocityIsAveraged(comHandler_, nodeId, velAvg,
        &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution --getVelocityIsAverage, nodeId=%d", _errorCode, nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::read_currentActual(
    uint16_t nodeId,
    int16_t* currentActual)
  {
    uint32_t _errorCode;
    if (VCS_GetCurrentIs(comHandler_, nodeId, currentActual,
        &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution --getCurrentIs, nodeId=%d", _errorCode, nodeId);
    }
    return _errorCode;
  }


  uint32_t Epos2Gateway::read_currentAvg(uint16_t nodeId, int16_t* currentAvg)
  {
    uint32_t _errorCode;
    if (VCS_GetCurrentIsAveraged(comHandler_, nodeId, currentAvg,
        &_errorCode) == 0)
    {
      // TODO(klpanagi): --- resolve error
      // Will return error to serial_epos2_handler to handle.
      ROS_WARN("[Epos2-Gateway]: Received error {%d} on command "
        "execution --getCurrentIsAverage, nodeId=%d", _errorCode, nodeId);
    }
    return _errorCode;
  }

}  // namespace motor
}  // namespace pandora_hardware_interface

