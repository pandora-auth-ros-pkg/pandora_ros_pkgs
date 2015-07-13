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

#ifndef MOTOR_CONTROLLERS_SKID_STEER_VELOCITY_CONTROLLER_H
#define MOTOR_CONTROLLERS_SKID_STEER_VELOCITY_CONTROLLER_H

#include <gsl/gsl_multifit.h>

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <pandora_motor_hardware_interface/KinematicParameters.h>

namespace pandora_hardware_interface
{
namespace motor
{
  class SkidSteerVelocityController
      : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
    public:
      SkidSteerVelocityController() {}

     /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param controller_nh Node handle inside the controller namespace
     */
      bool init(
          hardware_interface::VelocityJointInterface* hw,
          ros::NodeHandle &ns);

      void update(const ros::Time& time, const ros::Duration& period);
      void starting(const ros::Time& time) { }
      void stopping(const ros::Time& time) { }

      void commandCallbackTwist(const geometry_msgs::Twist& command);
      void updateParameters(const pandora_motor_hardware_interface::KinematicParameters& command);

    private:
      // Remap velocities using the calculated polynom
      void remapVelocities(
          double& linear,
          double& angular);

      // Calculate polynom's coefficients using polynomial regression
      void polynomialFit(
          const int& degree,
          const std::vector<double>& actualValues,
          const std::vector<double>& expectedValues,
          std::vector<double>& coefficients);

    private:
      /// Hardware joint handles:
      hardware_interface::JointHandle left_front_wheel_joint_;
      hardware_interface::JointHandle right_front_wheel_joint_;
      hardware_interface::JointHandle left_rear_wheel_joint_;
      hardware_interface::JointHandle right_rear_wheel_joint_;

      // cmd_vel ROS subscriber
      ros::Subscriber command_listener_;
      ros::Subscriber parameter_listener_;

      /// Velocity command related struct
      struct Commands
      {
        double lin;
        double ang;
        float terrain_parameter;
        float slip_factor_left;
        float slip_factor_right;
        ros::Time stamp;

        Commands() : lin(0.0), ang(0.0), stamp(0.0), terrain_parameter(1.0), slip_factor_left(0), slip_factor_right(0){}
      };
      Commands command_struct_;

      // Physical properties
      double wheel_radius_;
      double track_;  // wheel_separation
      double terrain_parameter_;

      // True when running in simulation
      bool sim_;

      // Vectors containing the measurements
      std::vector<double> expectedLinear_;
      std::vector<double> actualLinear_;
      std::vector<double> expectedAngular_;
      std::vector<double> actualAngular_;

      double maxMeasuredLinear_;
      double maxMeasuredAngular_;
      double minMeasuredLinear_;
      double minMeasuredAngular_;

      // Degree and coefficients of polynoms
      int linearFitDegree_;
      int angularFitDegree_;
      std::vector<double> linearFitCoefficients_;
      std::vector<double> angularFitCoefficients_;
  };

  PLUGINLIB_EXPORT_CLASS(
    pandora_hardware_interface::motor::SkidSteerVelocityController,
    controller_interface::ControllerBase);

}  //  namespace motor
}  //  namespace pandora_hardware_interface

#endif  // MOTOR_CONTROLLERS_SKID_STEER_VELOCITY_CONTROLLER_H
