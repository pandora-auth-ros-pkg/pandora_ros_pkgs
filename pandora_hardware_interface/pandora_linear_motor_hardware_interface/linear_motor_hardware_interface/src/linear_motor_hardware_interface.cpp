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
#include "linear_motor_hardware_interface/linear_motor_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace linear
{
  LinearMotorHardwareInterface::LinearMotorHardwareInterface(
    ros::NodeHandle nodeHandle)
  :
    nodeHandle_(nodeHandle)
  {
    nodeHandle_.getParam("linear_motor_joint", jointName_);

    serialIO_ = new JrkSerial("/dev/linear",115200,100);

    serialIO_->openDevice();

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

  LinearMotorHardwareInterface::~LinearMotorHardwareInterface()
  {
    delete serialIO_;
  }

  void LinearMotorHardwareInterface::read()
  {
    int feedback = serialIO_->readScaledFeedback();
    position_ = static_cast<float>(feedback)/4080*0.23;
    ROS_DEBUG_STREAM("Feedback: " << position_);
  }

  void LinearMotorHardwareInterface::write()
  {
    int target = static_cast<int>(command_/0.23*4080);
    if (target >= 0 && target <= 3215)
    {
      serialIO_->setTarget(target);
    }
    else
    {
      ROS_DEBUG_STREAM("Linear command out of bounds");
    }
  }
}  // namespace linear
}  // namespace pandora_hardware_interface
