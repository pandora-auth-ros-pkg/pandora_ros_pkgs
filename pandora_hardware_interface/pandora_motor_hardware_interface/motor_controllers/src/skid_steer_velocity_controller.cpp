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

#include "motor_controllers/skid_steer_velocity_controller.h"

#include <algorithm>

// min(max(x, minVal), maxVal).
template<typename T>
T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace pandora_hardware_interface
{
namespace motor
{

  bool SkidSteerVelocityController::init(hardware_interface::VelocityJointInterface* hw,
                                                                  ros::NodeHandle &ns)
  {
    // Load Joints from HW Interface , load joint NAMES from YAML
    std::string left_front_wheel_joint_name, right_front_wheel_joint_name;
    std::string left_rear_wheel_joint_name, right_rear_wheel_joint_name;


    ROS_INFO("STARTING CONTROLLER");

    if (!ns.getParam("left_front_wheel", left_front_wheel_joint_name))
    {
      ROS_ERROR("Could not find left fron wheel joint name");
      return false;
    }
    if (!ns.getParam("right_front_wheel", right_front_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!ns.getParam("left_rear_wheel", left_rear_wheel_joint_name ))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }
    if (!ns.getParam("right_rear_wheel", right_rear_wheel_joint_name))
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
    command_listener_ = ns.subscribe("/cmd_vel",
                                       1,
                                       &SkidSteerVelocityController::commandCallback,
                                       this);

    ROS_INFO("Successfully Initiallized controller!");
    return true;
  }

  void SkidSteerVelocityController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Update with latest cmd_vel commands
    double w = command_struct_.ang;
    double v = command_struct_.lin;
    double a = 1.5;
    // double a = command_struct_.terrain_parameter;
    double B = 0.35;
    // double il = command_struct_.slip_factor_left;
    // double ir = command_struct_.slip_factor_right;
    double wheel_radius = 0.0975;

    // velocity limits (m/s and m/s^2) linear.
    double max_vel = 0.5;
    double min_vel = -0.5;

    // velocity limits (r/s) angular.
    double max_ang = 0.8;
    double min_ang = -0.8;

    // motor velocity limits (r/s) (5500 motor rpm->5.09 wheel r/s)
    double min_velocity = -5.09;
    double max_velocity = 5.09;

    // Limiting cmd_vel.
    // If velocities over the limits,clamp.

    // v=clamp(v,min_vel,max_vel);
    // w=clamp(w,min_ang,max_ang);


    // Compute wheels velocities:  (1.Equations pandora_skid_steering.pdf )
    double vel_left  = (1/wheel_radius)*v-((a*B)/(2*wheel_radius))*w;
    double vel_right = (1/wheel_radius)*v+((a*B)/(2*wheel_radius))*w;
    // BEWARE!! : invert axes !! (paper vs URDF)

    // Compute wheels velocities:  (2.Equations pandora_skid_steering.pdf )

    // vel_right = v/(wheel_radius*(1-il)) + w/(2*wheel_radius*(1-il));
    // vel_left = v/(wheel_radius*(1-ir)) - w/(2*wheel_radius*(1-ir));

    // Limiting motor velocities

    // vel_left=clamp(vel_left, min_velocity, max_velocity);
    // vel_right=clamp(vel_right, min_velocity, max_velocity);

    // Set Joint Commands
    // ROS_INFO("%f %f",vel_left,vel_right);

    left_front_wheel_joint_.setCommand(vel_left);
    left_rear_wheel_joint_.setCommand(vel_left);
    right_front_wheel_joint_.setCommand(vel_right);
    right_rear_wheel_joint_.setCommand(vel_right);
  }

  void SkidSteerVelocityController::commandCallback(const geometry_msgs::Twist& command)
  {
    // Update command struct
    // Update command struct
    // add terrain_parameter
    command_struct_.ang   = command.angular.z;
    command_struct_.lin   = command.linear.x;
    // command_struct_.terrain_parameter = command.terrain_param;
    // command_struct_.slip_factor_left = command.scale_left;
    // command_struct_.slip_factor_right = command.scale_right;

    command_struct_.stamp = ros::Time::now();
  }

  /*
  void SkidSteerVelocityController::commandCallback(const pandora_motor_hardware_interface::KinodynamicCommand& command)
  {
    // Update command struct
    // Update command struct
    // add terrain_parameter
    command_struct_.ang   = command.cmd_vel.angular.z;
    command_struct_.lin   = command.cmd_vel.linear.x;
    command_struct_.terrain_parameter = command.terrain_param;
    // command_struct_.slip_factor_left = command.scale_left;
    // command_struct_.slip_factor_right = command.scale_right;

    command_struct_.stamp = ros::Time::now();
  }
  */

}  // namespace motor
}  // namespace pandora_hardware_interface
