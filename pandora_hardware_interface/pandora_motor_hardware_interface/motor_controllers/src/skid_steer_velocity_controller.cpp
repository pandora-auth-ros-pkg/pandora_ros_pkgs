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

#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "motor_controllers/skid_steer_velocity_controller.h"

// min(max(x, minVal), maxVal)
template<typename T> T clamp(T x, T min, T max)
{
  return std::min(std::max(min, x), max);
}

namespace pandora_hardware_interface
{
namespace motor
{

  bool SkidSteerVelocityController::init(
      hardware_interface::VelocityJointInterface* hw,
      ros::NodeHandle &ns)
  {
    // Load parameters
    std::string left_front_wheel_joint_name, right_front_wheel_joint_name;
    std::string left_rear_wheel_joint_name, right_rear_wheel_joint_name;

    ROS_INFO("Initializing velocity controller...");

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

    // Physical properties
    ns.param("terrain_parameter", terrain_parameter_, 1.0);
    if (!ns.getParam("wheel_radius", wheel_radius_))
    {
      ROS_ERROR("Could not find wheel_radius");
      return false;
    }
    if (!ns.getParam("track", track_))
    {
      ROS_ERROR("Could not find track");
      return false;
    }

    // Detect if running on simulation
    ns.param("/sim", sim_, false);

    // Measurements for linear velocity
    XmlRpc::XmlRpcValue linearMeasurementsList;
    if (ns.hasParam("measured_linear_velocities") &&
        ns.getParam("measured_linear_velocities", linearMeasurementsList))
    {
      ROS_ASSERT(linearMeasurementsList.getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string key;
      for (int ii = 0; ii < linearMeasurementsList.size(); ii++)
      {
        ROS_ASSERT(linearMeasurementsList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

        key = "expected";
        ROS_ASSERT(linearMeasurementsList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        expectedLinear_.push_back(static_cast<double>(linearMeasurementsList[ii][key]));

        key = "actual";
        ROS_ASSERT(linearMeasurementsList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        actualLinear_.push_back(static_cast<double>(linearMeasurementsList[ii][key]));
      }
    }

    // Measurements for angular velocity
    XmlRpc::XmlRpcValue angularMeasurementsList;
    if (ns.hasParam("measured_angular_velocities") &&
        ns.getParam("measured_angular_velocities", angularMeasurementsList))
    {
      ROS_ASSERT(angularMeasurementsList.getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string key;
      for (int ii = 0; ii < angularMeasurementsList.size(); ii++)
      {
        ROS_ASSERT(angularMeasurementsList[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);

        key = "expected";
        ROS_ASSERT(angularMeasurementsList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        expectedAngular_.push_back(static_cast<double>(angularMeasurementsList[ii][key]));

        key = "actual";
        ROS_ASSERT(angularMeasurementsList[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        actualAngular_.push_back(static_cast<double>(angularMeasurementsList[ii][key]));
      }
    }

    // Get limits from measurement velocities
    maxMeasuredLinear_ = *std::max_element(actualLinear_.begin(), actualLinear_.end());
    minMeasuredLinear_ = *std::min_element(actualLinear_.begin(), actualLinear_.end());
    maxMeasuredAngular_ = *std::max_element(actualAngular_.begin(), actualAngular_.end());
    minMeasuredAngular_ = *std::min_element(actualAngular_.begin(), actualAngular_.end());

    // Degree of polynoms
    ns.param("linear_fit_degree", linearFitDegree_, 5);
    if (linearFitDegree_ < 2)
    {
      linearFitDegree_ = 2;
    }
    ns.param("angular_fit_degree", angularFitDegree_, 7);
    if (angularFitDegree_ < 2)
    {
      angularFitDegree_ = 2;
    }

    // Increase degree because the coefficients also include the constant of the polynom (a0 * x^0)
    linearFitDegree_++;
    angularFitDegree_++;

    // Calculate coefficients
    linearFitCoefficients_.resize(linearFitDegree_, 0);
    angularFitCoefficients_.resize(angularFitDegree_, 0);

    // We need (degree + 1) measurements to calculate the polynom, otherwise we set (y = 1.0 * x^1)
    if (actualLinear_.size() >= linearFitDegree_)
    {
      polynomialFit(linearFitDegree_, actualLinear_, expectedLinear_, linearFitCoefficients_);
    }
    else
    {
      linearFitCoefficients_[1] = 1.0;
    }

    if (actualAngular_.size() >= angularFitDegree_)
    {
      polynomialFit(angularFitDegree_, actualAngular_, expectedAngular_, angularFitCoefficients_);
    }
    else
    {
      angularFitCoefficients_[1] = 1.0;
    }

    // Subscirbe to cmd_vel

    ros::NodeHandle private_nh("~");
    nodeName_ = boost::to_upper_copy<std::string>(private_nh.getNamespace());

    if (!ns.getParam("/using_rl", usingRL_))
    {
      ROS_FATAL("[%s] Cound not find using rl param!", nodeName_.c_str());
      ROS_BREAK();
    }
    
    command_listener_ = ns.subscribe(
        "/cmd_vel",
        1,
        &SkidSteerVelocityController::commandCallbackTwist,
        this);
    hasCommandReached_ = false;

    parameter_listener_ = ns.subscribe(
        "/kinematic_parameters",
        1,
        &SkidSteerVelocityController::updateParameters,
        this);
    hasParameterReached_ = false;

    ROS_INFO("Successfully initiallized velocity controller!");
    return true;
  }

  void SkidSteerVelocityController::update(const ros::Time& time, const ros::Duration& period)
  {
    // Update cmd_vel commands
    double angular = command_struct_.ang;
    double linear = command_struct_.lin;
    double terrain_parameter_ = command_struct_.terrain_parameter;
    double scale_left = command_struct_.scale_factor_left;
    double scale_right = command_struct_.scale_factor_right;

    if (!sim_)
    {
      remapVelocities(linear, angular);
    }

    // Compute wheels velocities
    double vel_left  = (1 / wheel_radius_) * linear - ((terrain_parameter_ * track_) / (2 * wheel_radius_)) * angular;
    double vel_right = (1 / wheel_radius_) * linear + ((terrain_parameter_ * track_) / (2 * wheel_radius_)) * angular;

    left_front_wheel_joint_.setCommand(scale_left * vel_left);
    left_rear_wheel_joint_.setCommand(scale_left * vel_left);
    right_front_wheel_joint_.setCommand(scale_right * vel_right);
    right_rear_wheel_joint_.setCommand(scale_right * vel_right);
  }

  void SkidSteerVelocityController::commandCallbackTwist(const geometry_msgs::TwistConstPtr& command)
  {
    hasCommandReached_ = true;
    velocitiesCommand_ = command;

    pandora_motor_hardware_interface::KinematicParametersPtr defaultParameters;
    if (!usingRL_)
    {
      defaultParameters.reset( new pandora_motor_hardware_interface::KinematicParameters );
      defaultParameters->terrain_param = 1.0;
      defaultParameters->scale_left = 1.0;
      defaultParameters->scale_right = 1.0;
      syncCallback(velocitiesCommand_, defaultParameters);
    }
    else
    {
      if (hasParameterReached_ && hasCommandReached_)
        syncCallback(velocitiesCommand_, parameterCommand_);
    }
  }

  void SkidSteerVelocityController::updateParameters(
      const pandora_motor_hardware_interface::KinematicParametersConstPtr& command)
  {
    hasParameterReached_ = true;
    parameterCommand_ = command;

    if (hasParameterReached_ && hasCommandReached_)
      syncCallback(velocitiesCommand_, parameterCommand_);
  }

  void SkidSteerVelocityController::syncCallback(
      const geometry_msgs::TwistConstPtr& velocitiesCommand,
      const pandora_motor_hardware_interface::KinematicParametersConstPtr& parameterCommand)
  {
    if (usingRL_)
    {
      if (!hasParameterReached_ || !hasCommandReached_)
      {
        ROS_ERROR("[%s] Incorrect sync callback: has not received enough info",
            nodeName_.c_str());
        return;
      }
      hasParameterReached_ = false;
      hasCommandReached_ = false;
    }

    command_struct_.ang   = velocitiesCommand->angular.z;
    command_struct_.lin   = velocitiesCommand->linear.x;
    command_struct_.terrain_parameter = parameterCommand->terrain_param;
    command_struct_.scale_factor_left = parameterCommand->scale_left;
    command_struct_.scale_factor_right = parameterCommand->scale_right;
    command_struct_.stamp = ros::Time::now();
  }

  void SkidSteerVelocityController::remapVelocities(
      double& linear,
      double& angular)
  {
    double newLinear;
    double newAngular;

    clamp(linear, minMeasuredLinear_, maxMeasuredLinear_);
    clamp(angular, minMeasuredAngular_, maxMeasuredAngular_);

    newLinear = 0;
    for (int i = 0; i < linearFitDegree_; i++)
    {
      newLinear += linearFitCoefficients_[i] * pow(linear, i);
    }

    newAngular = 0;
    for (int i = 0; i < angularFitDegree_; i++)
    {
      newAngular += angularFitCoefficients_[i] * pow(angular, i);
    }

    linear = newLinear;
    angular = newAngular;
  }

  void SkidSteerVelocityController::polynomialFit(
      const int& degree,
      const std::vector<double>& actualValues,
      const std::vector<double>& expectedValues,
      std::vector<double>& coefficients)
  {
    int obs = actualValues.size();

    gsl_multifit_linear_workspace *ws;
    gsl_matrix *cov, *X;
    gsl_vector *y, *c;
    double chisq;

    X = gsl_matrix_alloc(obs, degree);
    y = gsl_vector_alloc(obs);
    c = gsl_vector_alloc(degree);
    cov = gsl_matrix_alloc(degree, degree);

    for (int i = 0; i < obs; i++)
    {
      gsl_matrix_set(X, i, 0, 1.0);
      for (int j = 0; j < degree; j++)
      {
        gsl_matrix_set(X, i, j, pow(actualValues[i], j));
      }
      gsl_vector_set(y, i, expectedValues[i]);
    }

    ws = gsl_multifit_linear_alloc(obs, degree);
    gsl_multifit_linear(X, y, c, cov, &chisq, ws);

    for (int i = 0; i < degree; i++)
    {
      coefficients[i] = gsl_vector_get(c, i);
    }

    gsl_multifit_linear_free(ws);
    gsl_matrix_free(X);
    gsl_matrix_free(cov);
    gsl_vector_free(y);
    gsl_vector_free(c);
  }

}  // namespace motor
}  // namespace pandora_hardware_interface
