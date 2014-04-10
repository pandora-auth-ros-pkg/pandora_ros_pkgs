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
#include "motor_controllers/skid_steer_drive_controller.h"

namespace pandora_hardware_interface
{
namespace motor
{
  bool SkidSteerDriveController::init(
    hardware_interface::VelocityJointInterface* velocityJointInterface,
    ros::NodeHandle& rootNodeHandle,
    ros::NodeHandle& controllerNodeHandle)
  {
    rootNodeHandle_ = &rootNodeHandle;
    velocityJointInterface_ = velocityJointInterface;
    // Get joint names from interface
    const std::vector<std::string>& motorJointNames =
      velocityJointInterface_->getNames();

    for (int ii = 0; ii < motorJointNames.size(); ii++)
    {
      // Get motor handles from interface
      jointHandles_.push_back(
        velocityJointInterface_->getHandle(motorJointNames[ii]));
    }

    // Get params from param server
    if (controllerNodeHandle.hasParam("wheel_separation"))
      controllerNodeHandle.getParam("wheel_separation", wheelSeparation_);
    else
    {
      ROS_ERROR_STREAM("Parameter wheel_separation was not defined");
      return false;
    }

    if (controllerNodeHandle.hasParam("wheel_radius"))
      controllerNodeHandle.getParam("wheel_radius", wheelRadius_);
    else
    {
      ROS_ERROR_STREAM("Parameter wheel_radius was not defined");
      return false;
    }

    if (controllerNodeHandle.hasParam("max_angular_velocity"))
      controllerNodeHandle.getParam("max_angular_velocity", maxAngularVelocity_);
    else
    {
      ROS_ERROR_STREAM("Parameter max_angular_velocity was not defined");
      return false;
    }
      

    command_.push_back(ros::Time::now().toSec());
    command_.push_back(0);
    command_.push_back(0);
    commandBuffer_.writeFromNonRT(command_);

    commandSubscriber_ = controllerNodeHandle.subscribe(
      "/cmd_vel",
      1,
      &SkidSteerDriveController::twistCallback,
      this);

    return true;
  }

  void SkidSteerDriveController::starting(const ros::Time& time)
  {
    setMotorCommands(0, 0);
  }

  void SkidSteerDriveController::update(
    const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> currentCommand = *(commandBuffer_.readFromRT());
    double commandRecency = (time.toSec() - currentCommand[0]);
    if (commandRecency < COMMAND_DELAY_THRESHOLD)
    {
      // Compute wheels velocities:
      double leftVelocity =
        (currentCommand[1] - currentCommand[2] * wheelSeparation_ / 2.0)
        / wheelRadius_;
      double rightVelocity =
        (currentCommand[1] + currentCommand[2] * wheelSeparation_ / 2.0)
        / wheelRadius_;

      setMotorCommands(leftVelocity, rightVelocity);
    }
    else
    {
      setMotorCommands(0, 0);
    }
  }

  void SkidSteerDriveController::stopping(const ros::Time& time)
  {
    setMotorCommands(0, 0);
  }

  void SkidSteerDriveController::twistCallback(
    const geometry_msgs::Twist& twist)
  {
    command_[0] = ros::Time::now().toSec();
    command_[1] = twist.linear.x;
    command_[2] = twist.angular.z;
    commandBuffer_.writeFromNonRT(command_);
  }

  void SkidSteerDriveController::setMotorCommands(
    const double leftVelocity, const double rightVelocity)
  {
    double leftCommand, rightCommand;
    if (fabs(leftVelocity) > fabs(maxAngularVelocity_))
    {
      ROS_DEBUG_STREAM("Limiting left wheel speed, it's to high");
      leftCommand = copysign(maxAngularVelocity_, leftVelocity);
    }
    else
      leftCommand = leftVelocity;

    if (fabs(rightVelocity) > fabs(maxAngularVelocity_))
    {
      ROS_DEBUG_STREAM("Limiting right wheel speed, it's to high");
      rightCommand = copysign(maxAngularVelocity_, rightVelocity);
    }
    else
      rightCommand = rightVelocity;

    modf( (leftCommand/maxAngularVelocity_) * 255, &leftCommand);
    modf( (rightCommand/maxAngularVelocity_) * 255, &rightCommand);

    std::string str1 = "left";
    std::string str2 = "right";

    for (int ii = 0; ii < jointHandles_.size(); ii++)
    {
      std::size_t foundLeft = jointHandles_[ii].getName().find(str1);
      std::size_t foundRight = jointHandles_[ii].getName().find(str2);

      // Set motor commands
      if (foundLeft != std::string::npos)
        jointHandles_[ii].setCommand(leftCommand);
      else if (foundRight != std::string::npos)
        jointHandles_[ii].setCommand(rightCommand);
    }
  }

  SkidSteerDriveController::SkidSteerDriveController()
  {
  }

  SkidSteerDriveController::~SkidSteerDriveController()
  {
  }

  PLUGINLIB_EXPORT_CLASS(
    pandora_hardware_interface::motor::SkidSteerDriveController,
    controller_interface::ControllerBase);
}  // namespace motor
}  // namespace pandora_hardware_interface
