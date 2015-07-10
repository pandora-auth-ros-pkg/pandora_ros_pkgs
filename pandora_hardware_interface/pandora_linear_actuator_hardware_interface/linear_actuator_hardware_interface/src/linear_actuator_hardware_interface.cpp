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
* Author: George Kouros
*********************************************************************/
#include "linear_actuator_hardware_interface/linear_actuator_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace linear_actuator
{
  LinearActuatorHardwareInterface::LinearActuatorHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    nodeHandle_.getParam("linear_actuator_name", jointName_);

    std::string jointType;
    nodeHandle_.param<std::string>(
      "linear_actuator_type", jointType, "firgelli");

    if (jointType == "jrk")
    {
      comInterfacePtr_ = new JrkComInterface("/dev/linear_actuator", 115200, 100);
      comInterfacePtr_->init();
    }
    else if (jointType == "firgelli")
    {
      comInterfacePtr_ = new FirgelliComInterface();
      comInterfacePtr_->init();
      // initialize position of linear actuator
      comInterfacePtr_->setTarget(0);
    }
    else
    {
      ROS_FATAL("Parameter 'joint_type' not set right in parameter server");
      exit(-1);
    }

    // connect and register the joint state interface
    position_ = 0;
    velocity_ = 0;
    effort_ = 0;
    hardware_interface::JointStateHandle jointStateHandle(
      jointName_,
      &position_,
      &velocity_,
      &effort_);
    jointStateInterface_.registerHandle(jointStateHandle);
    registerInterface(&jointStateInterface_);

    // connect and register the joint position interface
    hardware_interface::JointHandle jointPositionHandle(
      jointStateInterface_.getHandle(jointName_),
      &command_);
    positionJointInterface_.registerHandle(jointPositionHandle);
    registerInterface(&positionJointInterface_);
  }

  LinearActuatorHardwareInterface::~LinearActuatorHardwareInterface()
  {
    delete comInterfacePtr_;
  }

  void LinearActuatorHardwareInterface::read()
  {
    position_ = comInterfacePtr_->readScaledFeedback();
    ROS_DEBUG_STREAM("Feedback: " << position_);
  }

  void LinearActuatorHardwareInterface::write()
  {
    float target = static_cast<float>(command_);
    // clip target command
    if (target < 0)
      target = 0;
    else if (target > 14)
      target = 14;

    comInterfacePtr_->setTarget(target);
  }
}  // namespace linear_actuator
}  // namespace pandora_hardware_interface
