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

#include "epos2_handler/serial_epos2_handler.h"

namespace pandora_hardware_interface
{
namespace motor
{

  SerialEpos2Handler::SerialEpos2Handler():
    epos2_nh_("/motor/epos2")
  {
    std::string _portName, _deviceName, _protocolStackName, _interfaceName;
    int _baudrate, _timeout, _numControllers, _gatewayId;
    int _nodeId[4];
    std::string _motorId[4];

    /*--<Load epos2 interface configs from parameter server>--*/
    epos2_nh_.getParam("interface/portName", _portName);
    epos2_nh_.getParam("interface/baudrate", _baudrate);
    epos2_nh_.getParam("interface/timeout", _timeout);
    epos2_nh_.getParam("interface/deviceName", _deviceName);
    epos2_nh_.getParam("interface/protocolStackName", _protocolStackName);
    epos2_nh_.getParam("interface/interfaceName", _interfaceName);
    epos2_nh_.getParam("controllers/epos2Gateway_id", _gatewayId);
    epos2_nh_.getParam("controllers/num", _numControllers);
    epos2_nh_.getParam("controllers/node1/id", _nodeId[0]);
    epos2_nh_.getParam("controllers/node1/index", _motorId[0]);
    epos2_nh_.getParam("controllers/node2/id", _nodeId[1]);
    epos2_nh_.getParam("controllers/node2/index", _motorId[1]);
    epos2_nh_.getParam("controllers/node3/id", _nodeId[2]);
    epos2_nh_.getParam("controllers/node3/index", _motorId[2]);
    epos2_nh_.getParam("controllers/node4/id", _nodeId[3]);
    epos2_nh_.getParam("controllers/node4/index", _motorId[3]);
    /*-------------------------------------------------------*/

    epos2Gateway_.reset(new Epos2Gateway(_portName, _baudrate, _timeout,
        _deviceName, _protocolStackName, _interfaceName));

    rightFrontMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(rightFrontMotor_);
    rightFrontMotor_->nodeId_ = static_cast<uint16_t>(_nodeId[0]);
    rightFrontMotor_->motorId_ = _motorId[0];

    rightRearMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(rightRearMotor_);
    rightRearMotor_->nodeId_ = static_cast<uint16_t>(_nodeId[1]);
    rightRearMotor_->motorId_ = _motorId[1];

    leftFrontMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(leftFrontMotor_);
    leftFrontMotor_->nodeId_ = static_cast<uint16_t>(_nodeId[2]);
    leftFrontMotor_->motorId_ = _motorId[2];

    leftRearMotor_ = new Epos2Controller();
    epos2Controllers_.push_back(leftRearMotor_);
    leftRearMotor_->nodeId_ = static_cast<uint16_t>(_nodeId[3]);
    leftRearMotor_->motorId_ = _motorId[3];

    gatewayId_ = static_cast<uint16_t>(_gatewayId);

    // Open device communication port
    epos2Gateway_->openDevice();

    // Initialize motor controller states {Enabled}
    readStates();   // Read current state of the Motors
    stateHandle();  // Calls the state handler to handle the states

    // Set every epos2 controller at profileVelocityMode on startup
    epos2Gateway_->activate_profileVelocityMode(rightFrontMotor_->nodeId_);
    epos2Gateway_->activate_profileVelocityMode(rightRearMotor_->nodeId_);
    epos2Gateway_->activate_profileVelocityMode(leftFrontMotor_->nodeId_);
    epos2Gateway_->activate_profileVelocityMode(leftRearMotor_->nodeId_);

    // Setting operation mode to velocity_mode
    operation_mode_ = 0;
  }


  SerialEpos2Handler::~SerialEpos2Handler()
  {
    epos2Gateway_->set_targetVelocity(rightFrontMotor_->nodeId_, 0);
    epos2Gateway_->set_targetVelocity(rightRearMotor_->nodeId_, 0);
    epos2Gateway_->set_targetVelocity(leftFrontMotor_->nodeId_, 0);
    epos2Gateway_->set_targetVelocity(leftRearMotor_->nodeId_, 0);
    epos2Gateway_->setDisableState(rightFrontMotor_->nodeId_);
    epos2Gateway_->setDisableState(rightRearMotor_->nodeId_);
    epos2Gateway_->setDisableState(leftFrontMotor_->nodeId_);
    epos2Gateway_->setDisableState(leftRearMotor_->nodeId_);
    epos2Gateway_->closeDevice();
  }


  void SerialEpos2Handler::stateHandle(void)
  {
    for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
    {
      switch (epos2Controllers_.at(_ii)->state_)
      {
        case 0:  // DISABLE STATE
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 1:  // ENABLE STATE
          break;
        case 2:  // QUICKSTOP STATE
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 3:  // FAULTY STATE
          epos2Gateway_->clearFaultState(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        case 9:  // CANNOT COMMUNICATE
          if (epos2Controllers_.at(_ii)->nodeId_ == gatewayId_)
          {
            // Cannot communicate with epos2-Gateway
            ROS_FATAL("[Motors]: Cannot communicate with epos2-Gateway.");
          }
          epos2Gateway_->resetNode(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
        default:  // UKNOWN STATE
          ROS_FATAL("[Motors]: UKNOWN STATE --> [%d]",
            epos2Controllers_.at(_ii)->state_);
          epos2Gateway_->resetNode(epos2Controllers_.at(_ii)->nodeId_);
          epos2Gateway_->setEnableState(epos2Controllers_.at(_ii)->nodeId_);
          break;
      }
    }

    readStates();

    for (uint16_t _ii = 0; _ii < epos2Controllers_.size(); _ii++)
    {
      if (epos2Controllers_.at(_ii)->state_ != 1)
      {
        stateHandle();
      }
    }
  }


  void SerialEpos2Handler::getRPM(int* leftRearRpm, int* leftFrontRpm,
    int* rightRearRpm, int* rightFrontRpm)
  {
    epos2Gateway_->read_velocityAvg(rightFrontMotor_->nodeId_,
      &rightFrontMotor_->rpm_);
    epos2Gateway_->read_velocityAvg(rightRearMotor_->nodeId_,
      &rightRearMotor_->rpm_);
    epos2Gateway_->read_velocityAvg(leftFrontMotor_->nodeId_,
      &leftFrontMotor_->rpm_);
    epos2Gateway_->read_velocityAvg(leftRearMotor_->nodeId_,
      &leftRearMotor_->rpm_);
    *rightFrontRpm = rightFrontMotor_->rpm_;
    *rightRearRpm = rightRearMotor_->rpm_;
    *leftFrontRpm = leftFrontMotor_->rpm_;
    *leftRearRpm = leftRearMotor_->rpm_;
  }

  void SerialEpos2Handler::getCurrent(int* leftRearCurrent,
    int* leftFrontCurrent, int* rightRearCurrent, int* rightFrontCurrent)
  {
    epos2Gateway_->read_currentAvg(rightFrontMotor_->nodeId_,
      &rightFrontMotor_->current_);
    epos2Gateway_->read_currentAvg(rightRearMotor_->nodeId_,
      &rightRearMotor_->current_);
    epos2Gateway_->read_currentAvg(leftFrontMotor_->nodeId_,
      &leftFrontMotor_->current_);
    epos2Gateway_->read_currentAvg(leftRearMotor_->nodeId_,
      &leftRearMotor_->current_);
    *rightFrontCurrent = static_cast<int>(rightFrontMotor_->current_);
    *rightRearCurrent = static_cast<int>(rightRearMotor_->current_);
    *leftFrontCurrent = static_cast<int>(leftFrontMotor_->current_);
    *leftRearCurrent = static_cast<int>(leftRearMotor_->current_);
  }


  Error SerialEpos2Handler::getError()
  {
  }

  uint16_t SerialEpos2Handler::writeRPM(const int leftRpm, const int rightRpm)
  {
    ROS_DEBUG("[Motors]: Setting speed %d, %d", leftRpm, rightRpm);
    epos2Gateway_->set_targetVelocity(rightFrontMotor_->nodeId_, -rightRpm);
    epos2Gateway_->set_targetVelocity(rightRearMotor_->nodeId_, -rightRpm);
    epos2Gateway_->set_targetVelocity(leftFrontMotor_->nodeId_, leftRpm);
    epos2Gateway_->set_targetVelocity(leftRearMotor_->nodeId_, leftRpm);
    return 1;
  }

  void SerialEpos2Handler::readStates(void)
  {
    epos2Gateway_->readState(rightFrontMotor_->nodeId_,
      &rightFrontMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", rightFrontMotor_->nodeId_,
      rightFrontMotor_->state_);
    epos2Gateway_->readState(rightRearMotor_->nodeId_,
      &rightRearMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", rightRearMotor_->nodeId_,
      rightRearMotor_->state_);
    epos2Gateway_->readState(leftFrontMotor_->nodeId_,
      &leftFrontMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", leftFrontMotor_->nodeId_,
      leftFrontMotor_->state_);
    epos2Gateway_->readState(leftRearMotor_->nodeId_,
     &leftRearMotor_->state_);
    ROS_INFO("[Motors]: NodeId-%d State==%d", leftRearMotor_->nodeId_,
      leftRearMotor_->state_);
  }


  void SerialEpos2Handler::getTorque(
    double* leftRearTorque,
    double* leftFrontTorque,
    double* rightRearTorque,
    double* rightFrontTorque)
  {
    // Define new array to store current values
    int _currentFeed[4];

    // Fill with current values using getCurrent method
    this->getCurrent(&_currentFeed[0], &_currentFeed[1],
      &_currentFeed[2], &_currentFeed[3]);

    // Convert to Torques
    *leftRearTorque = this->currentToTorque(_currentFeed[0]);
    *leftFrontTorque = this->currentToTorque(_currentFeed[1]);
    *rightRearTorque = this->currentToTorque(_currentFeed[2]);
    *rightFrontTorque = this->currentToTorque(_currentFeed[3]);
  }


  double SerialEpos2Handler::currentToTorque(int _input_current)
  {
    
    return static_cast<double>(_input_current * 33.5 * 113 / 10 / 10 / 10 );
  }


  int16_t SerialEpos2Handler::torqueToCurrent(double _input_torque)
  {
    return static_cast<int16_t>(_input_torque / 33.5 / 113 * 1000 );
  }

  uint16_t SerialEpos2Handler::writeTorques(
        double leftRearTorque,
        double leftFrontTorque,
        double rightRearTorque,
        double rightFrontTorque)
  {
    // Step I :Convert Torques to currents
    int16_t leftRearCurrent = torqueToCurrent(leftRearTorque);
    int16_t leftFrontCurrent = torqueToCurrent(leftFrontTorque);
    int16_t rightRearCurrent = torqueToCurrent(rightRearTorque);
    int16_t rightFrontCurrent = torqueToCurrent(rightFrontTorque);


    // Step II: Send commands to motors
    ROS_DEBUG("[Motors]: Setting torques %f, %f, %f, %f",
      leftRearTorque, leftFrontTorque, 
      rightRearTorque, rightFrontTorque);

    epos2Gateway_->set_targetCurrent(leftRearMotor_->nodeId_,
      leftRearCurrent);
    epos2Gateway_->set_targetCurrent(leftFrontMotor_->nodeId_,
      leftFrontCurrent);
    epos2Gateway_->set_targetCurrent(rightRearMotor_->nodeId_,
      rightRearCurrent);
    epos2Gateway_->set_targetCurrent(rightFrontMotor_->nodeId_,
      rightFrontCurrent);

    return 1;
  }

  void SerialEpos2Handler::setMode(int mode)
  {
    switch (mode)
    {
      case 0:
        // Activate Velocity Mode
        ROS_INFO("Entering Velocity Mode");
        epos2Gateway_->activate_profileVelocityMode(rightFrontMotor_->nodeId_);
        epos2Gateway_->activate_profileVelocityMode(rightRearMotor_->nodeId_);
        epos2Gateway_->activate_profileVelocityMode(leftFrontMotor_->nodeId_);
        epos2Gateway_->activate_profileVelocityMode(leftRearMotor_->nodeId_);

        operation_mode_ = 0;

        break;

      case 1:
        // Activate Current Mode
        ROS_INFO("Entering Current Mode");
        epos2Gateway_->activate_currentMode(rightFrontMotor_->nodeId_);
        epos2Gateway_->activate_currentMode(rightRearMotor_->nodeId_);
        epos2Gateway_->activate_currentMode(leftFrontMotor_->nodeId_);
        epos2Gateway_->activate_currentMode(leftRearMotor_->nodeId_);

        operation_mode_ = 1;

        break;

      default:
        ROS_WARN("There is no such state");
        break;
    }
  }

  int SerialEpos2Handler::getMode(void)
  {
    return operation_mode_;
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
