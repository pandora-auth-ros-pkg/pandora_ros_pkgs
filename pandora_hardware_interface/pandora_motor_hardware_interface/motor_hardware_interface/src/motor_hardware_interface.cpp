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
#include "motor_hardware_interface/motor_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace motor
{
  MotorHardwareInterface::MotorHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    //~ motors_ = new SerialEposHandler("/dev/ttyUSB0", 115200, 500);
    readJointNameFromParamServer();

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

    // connect and register the joint velocity interface
    for (int ii = 0; ii < jointNames_.size(); ii++)
    {
      hardware_interface::JointHandle jointVelocityHandle(
        jointStateInterface_.getHandle(jointNames_[ii]),
        &command_[ii]);
      velocityJointInterface_.registerHandle(jointVelocityHandle);
    }
    registerInterface(&velocityJointInterface_);
  }

  MotorHardwareInterface::~MotorHardwareInterface()
  {
  }

  void MotorHardwareInterface::read()
  {
    int feedback[4];
    motors_->getRPM(&feedback[2], &feedback[0], &feedback[3], &feedback[1]);

    for (int ii = 0; ii < 4; ii++)
    {
      velocity_[ii] = static_cast<double>(feedback[ii]);
    }
  }

  void MotorHardwareInterface::write()
  {
    motors_->writeRPM(command_[0], command_[1]);
  }

  void MotorHardwareInterface::readJointNameFromParamServer()
  {
    std::string name;
    nodeHandle_.getParam(
      "motor_joints/robot_movement_joints/left_front_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "motor_joints/robot_movement_joints/right_front_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "motor_joints/robot_movement_joints/left_rear_joint",
      name);
    jointNames_.push_back(name);
    nodeHandle_.getParam(
      "motor_joints/robot_movement_joints/right_rear_joint",
      name);
    jointNames_.push_back(name);
  }
}  // namespace motor
}  // namespace pandora_hardware_interface
