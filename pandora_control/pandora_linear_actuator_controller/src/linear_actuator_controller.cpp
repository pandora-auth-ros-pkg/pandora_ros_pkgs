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
* Author:  Chris Zalidis
*********************************************************************/

#include <pandora_linear_actuator_controller/linear_actuator_controller.h>

namespace pandora_control
{
  LinearActuatorActionServer::LinearActuatorActionServer(const std::string& actionName) :
    nodeHandle_(""),
    privateNodeHandle_("~"),
    actionServer_(
      nodeHandle_,
      actionName,
      boost::bind(&LinearActuatorActionServer::callback, this, _1), false),
    actionName_(actionName)
  {
    // get params from param server
    if (getControllerParams())
    {
      linearActuatorCommandPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          linearActuatorCommandTopic_,
          5, true);

      std_msgs::Float64 targetPosition;
      targetPosition.data = previousTarget_ = 0;
      linearActuatorCommandPublisher_.publish(targetPosition);
      actionServer_.registerPreemptCallback(
          boost::bind(&LinearActuatorActionServer::preemptCallback, this));
      actionServer_.start();
    }
  }

  LinearActuatorActionServer::~LinearActuatorActionServer(void)
  {
  }

  void LinearActuatorActionServer::callback(
      const pandora_linear_actuator_controller::MoveLinearActuatorGoalConstPtr& goal)
  {
    command_ = goal->command;
    if (command_ == pandora_linear_actuator_controller::MoveLinearActuatorGoal::TEST)
    {
      testLinearActuator();
    }
    else if (command_ == pandora_linear_actuator_controller::MoveLinearActuatorGoal::LOWER)
    {
      lowerLinearActuator();
    }
    else if (command_ == pandora_linear_actuator_controller::MoveLinearActuatorGoal::MOVE)
    {
      moveLinearActuator(goal->point_of_interest, goal->center_point,
        movementThreshold_);
    }
    else if (command_ == pandora_linear_actuator_controller::MoveLinearActuatorGoal::LAX_MOVE)
    {
      moveLinearActuator(goal->point_of_interest, goal->center_point,
        laxMovementThreshold_);
    }
    else
    {
      ROS_DEBUG("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
    }
  }

  void LinearActuatorActionServer::preemptCallback()
  {
    ROS_INFO("%s: Preempted", actionName_.c_str());
    actionServer_.setPreempted();
  }

  bool LinearActuatorActionServer::getControllerParams()
  {
    privateNodeHandle_.param("min_command", minCommand_, 0.0);
    privateNodeHandle_.param("max_command", maxCommand_, 0.18);
    privateNodeHandle_.param("movement_threshold", movementThreshold_, 0.005);
    privateNodeHandle_.param("lax_movement_threshold", laxMovementThreshold_, 0.03);
    movementThreshold_ = fabs(movementThreshold_);
    privateNodeHandle_.param("command_timeout", commandTimeout_, 15.0);

    ROS_INFO("private ns: %s", privateNodeHandle_.getNamespace().c_str());

    if (privateNodeHandle_.getParam("linear_actuator_command_topic", linearActuatorCommandTopic_))
    {
      ROS_INFO_STREAM("Got param linear_actuator_command_topic: " << linearActuatorCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param linear_actuator_command_topic shuting down");
      return false;
    }

    if (privateNodeHandle_.getParam("linear_actuator_frame", linearActuatorFrame_))
    {
      ROS_INFO_STREAM("Got param linear_actuator_frame: " << linearActuatorFrame_);
    }
    else
    {
      ROS_FATAL("Failed to get param linear_actuator_frame shuting down");
      return false;
    }

    // Parse robot description
    const std::string model_param_name = "/robot_description";
    std::string robot_model_str="";
    while (!privateNodeHandle_.getParam(model_param_name, robot_model_str))
    {
      ROS_ERROR_STREAM(
        "Robot descripion couldn't be retrieved from param server.");
    }

    boost::shared_ptr<urdf::ModelInterface> model(
      urdf::parseURDF(robot_model_str));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> link =
      model->getLink(linearActuatorFrame_);
    minElevation_ =
      link->parent_joint->parent_to_joint_origin_transform.position.z;
    ROS_INFO_STREAM("Got minElevation_ from URDF: " << minElevation_);

    return true;
  }

  void LinearActuatorActionServer::testLinearActuator()
  {
    tf::StampedTransform linearActuatorTransform;
    try
    {
      tfListener_.lookupTransform(
        "/base_link", linearActuatorFrame_,
        ros::Time(0), linearActuatorTransform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    double startX = linearActuatorTransform.getOrigin()[2];
    double step = -0.006;
    if (startX + step < minElevation_)
    {
      step = - step;
    }
    std_msgs::Float64 targetPosition;
    targetPosition.data = previousTarget_ = startX + step - minElevation_;
    // Step could be a param
    linearActuatorCommandPublisher_.publish(targetPosition);

    ros::Time begin = ros::Time::now();

    double linearZ = -1;
    while (ros::ok() &&
      fabs(linearZ - (startX + step)) >= movementThreshold_)
    {
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearActuatorFrame_,
          ros::Time(0), linearActuatorTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      linearZ = linearActuatorTransform.getOrigin()[2];
      if (ros::Time::now().toSec() - begin.toSec() > commandTimeout_)
      {
        ROS_DEBUG("%s: Aborted", actionName_.c_str());
        // set the action state to succeeded
        actionServer_.setAborted();
        return;
      }
    }
    ROS_DEBUG("%s: Succeeded", actionName_.c_str());
    // set the action state to succeeded
    actionServer_.setSucceeded();
  }

  void LinearActuatorActionServer::lowerLinearActuator()
  {
    std_msgs::Float64 targetPosition;
    targetPosition.data = previousTarget_ = 0;
    linearActuatorCommandPublisher_.publish(targetPosition);

    ros::Time begin = ros::Time::now();

    tf::StampedTransform linearActuatorTransform;
    double linearZ = -1;
    while (ros::ok() &&
      fabs(linearZ - minElevation_) >= movementThreshold_)
    {
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearActuatorFrame_,
          ros::Time(0), linearActuatorTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      linearZ = linearActuatorTransform.getOrigin()[2];
      if (ros::Time::now().toSec() - begin.toSec() > commandTimeout_)
      {
        ROS_DEBUG("%s: Aborted", actionName_.c_str());
        // set the action state to succeeded
        actionServer_.setAborted();
        return;
      }
    }
    ROS_DEBUG("%s: Succeeded", actionName_.c_str());
    // set the action state to succeeded
    actionServer_.setSucceeded();
  }

  void LinearActuatorActionServer::moveLinearActuator(std::string pointOfInterest,
    std::string centerPoint, double movementThreshold)
  {
    ros::Time lastTf = ros::Time::now();
    ros::Rate rate(5);
    std_msgs::Float64 targetPosition;

    while (ros::ok())
    {
      tf::StampedTransform linearActuatorTransform;
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearActuatorFrame_,
          ros::Time(0), linearActuatorTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      tf::StampedTransform linearActuatorToTargetTransform;
      try
      {
        tfListener_.lookupTransform(
          linearActuatorFrame_, pointOfInterest,
          ros::Time(0), linearActuatorToTargetTransform);
      }
      catch (tf::TransformException ex)
      {
        if (ros::Time::now() - lastTf > ros::Duration(1))
        {
          ROS_DEBUG_STREAM("Is " << pointOfInterest << " broadcasted?");
          ROS_DEBUG("%s: Aborted", actionName_.c_str());
          // set the action state to succeeded
          actionServer_.setAborted();
          return;
        }
        else
        {
          continue;
        }
      }

      tf::StampedTransform linearActuatorToCenterTransform;
      try
      {
        tfListener_.lookupTransform(
          linearActuatorFrame_, centerPoint,
          ros::Time(0), linearActuatorToCenterTransform);
      }
      catch (tf::TransformException ex)
      {
        if (ros::Time::now() - lastTf > ros::Duration(1))
        {
          ROS_DEBUG_STREAM("Is " << centerPoint << " broadcasted?");
          ROS_DEBUG("%s: Aborted", actionName_.c_str());
          // set the action state to succeeded
          actionServer_.setAborted();
          return;
        }
        else
        {
          continue;
        }
      }
      lastTf = ros::Time::now();

      double deltaZ = linearActuatorToTargetTransform.getOrigin()[2]
        - linearActuatorToCenterTransform.getOrigin()[2];

      double targetZ = linearActuatorTransform.getOrigin()[2] + deltaZ;
      if (targetZ < minElevation_)
      {
        targetPosition.data = minCommand_;
      }
      else if (targetZ <= minElevation_ + maxCommand_)
      {
        targetPosition.data = targetZ - minElevation_;
      }
      else
      {
        targetPosition.data = maxCommand_;
      }
      if (fabs(previousTarget_ - targetPosition.data) > movementThreshold)
      {
        linearActuatorCommandPublisher_.publish(targetPosition);
        previousTarget_ = targetPosition.data;
      }
      rate.sleep();
    }
  }
}  // namespace pandora_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_actuator_controller");

  pandora_control::LinearActuatorActionServer
    linearActuatorActionServer("linear_actuator_action");

  ros::spin();
}
