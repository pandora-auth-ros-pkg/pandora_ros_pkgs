/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
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
 *   * Neither the name of the PAL Robotics nor the names of its
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
 *********************************************************************/

/*
 * Author: Konstantinos Panayiotou   <klpanagi@gmail.com>
 * Author: Konstantinos Zisis        <zisikons@gmail.com>
 * Author: Elisabet Papadopoulou     <papaelisabet@gmail.com >
 */

#include "motor_controllers/skid_steer_torque_controller.h"

namespace pandora_hardware_interface
{
namespace motor
{

  bool SkidSteerTorqueController::init(hardware_interface::EffortJointInterface* hw,
                ros::NodeHandle &nh)
  {
    // Load Joints from HW Interface , load joint NAMES from YAML
    std::string left_front_wheel_joint_name, right_front_wheel_joint_name;
    std::string left_rear_wheel_joint_name, right_rear_wheel_joint_name;

    ROS_INFO("STARTING CONTROLLER");

    if (!nh.getParam("left_front_wheel", left_front_wheel_joint_name))
    {
      ROS_ERROR("Could not find left fron wheel joint name");
      return false;
    }
    if (!nh.getParam("right_front_wheel", right_front_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!nh.getParam("left_rear_wheel", left_rear_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!nh.getParam("right_rear_wheel", right_rear_wheel_joint_name))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // Get joint Handles from hw interface
    left_front_wheel_joint_ = hw->getHandle(left_front_wheel_joint_name);
    right_front_wheel_joint_ = hw->getHandle(right_front_wheel_joint_name);
    left_rear_wheel_joint_ = hw->getHandle(left_rear_wheel_joint_name);
    right_rear_wheel_joint_ = hw->getHandle(right_rear_wheel_joint_name);

    // Subscirbe to cmd_vel
    command_listener_ = nh.subscribe("/cmd_trq",
                                       1,
                                       &SkidSteerTorqueController::commandCallback,
                                       this);

    ROS_INFO("Successfully Initiallized controller!");
    return true;
  }

  void SkidSteerTorqueController::update(const ros::Time& time, const ros::Duration& period)
  {
    left_front_wheel_joint_.setCommand(command_struct_.left_rear_wheel_torque);
    left_rear_wheel_joint_.setCommand(command_struct_.left_front_wheel_torque);
    right_front_wheel_joint_.setCommand(command_struct_.right_rear_wheel_torque);
    right_rear_wheel_joint_.setCommand(command_struct_.right_front_wheel_torque);
  }

  void SkidSteerTorqueController::commandCallback(const pandora_sensor_msgs::TorqueMsg & command)
  {
    // Update command_struct_ with latest information
    command_struct_.left_rear_wheel_torque   = command.left_rear_wheel_torque;
    command_struct_.left_front_wheel_torque  = command.left_front_wheel_torque;
    command_struct_.right_rear_wheel_torque  = command.right_rear_wheel_torque;
    command_struct_.right_front_wheel_torque = command.right_front_wheel_torque;
    command_struct_.stamp                    = ros::Time::now();
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
