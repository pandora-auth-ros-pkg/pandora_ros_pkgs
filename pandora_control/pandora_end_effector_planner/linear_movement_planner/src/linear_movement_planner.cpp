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

#include <linear_movement_planner/linear_movement_planner.h>

namespace pandora_control
{
  LinearMovementActionServer::LinearMovementActionServer(
    std::string actionName,
    ros::NodeHandle nodeHandle)
  :
    actionServer_(
      nodeHandle,
      actionName,
      boost::bind(&LinearMovementActionServer::callback, this, _1), false),
    actionName_(actionName),
    nodeHandle_(nodeHandle)
  {
    // get params from param server
    if (getPlannerParams())
    {
      linearCommandPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          linearCommandTopic_,
          5, true);

      std_msgs::Float64 targetPosition;
      targetPosition.data = previousTarget_ = 0;
      linearCommandPublisher_.publish(targetPosition);
      actionServer_.start();
    }
  }

  LinearMovementActionServer::~LinearMovementActionServer(void)
  {
  }

  void LinearMovementActionServer::callback(
      const pandora_end_effector_planner::MoveLinearGoalConstPtr& goal)
  {
    command_ = goal->command;
    if (command_ == pandora_end_effector_planner::MoveLinearGoal::TEST)
    {
      testLinear();
    }
    else if (command_ == pandora_end_effector_planner::MoveLinearGoal::LOWER)
    {
      lowerLinear();
    }
    else if (command_ == pandora_end_effector_planner::MoveLinearGoal::MOVE)
    {
      moveLinear(goal->point_of_interest, goal->center_point);
    }
    else
    {
      ROS_DEBUG("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
    }
  }

  bool LinearMovementActionServer::getPlannerParams()
  {
    nodeHandle_.param("min_command", minCommand_, 0.0);
    nodeHandle_.param("max_command", maxCommand_, 0.18);
    nodeHandle_.param("movement_threshold", movementThreshold_, 0.005);
    movementThreshold_ = fabs(movementThreshold_);
    nodeHandle_.param("command_timeout", commandTimeout_, 15.0);

    if (nodeHandle_.getParam("linear_command_topic", linearCommandTopic_))
    {
      ROS_INFO_STREAM("Got param linear_command_topic: " << linearCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param linear_command_topic shuting down");
      return false;
    }

    if (nodeHandle_.getParam("linear_motor_frame", linearMotorFrame_))
    {
      ROS_INFO_STREAM("Got param linear_motor_frame: " << linearMotorFrame_);
    }
    else
    {
      ROS_FATAL("Failed to get param linear_motor_frame shuting down");
      return false;
    }

    // Parse robot description
    const std::string model_param_name = "/robot_description";
    std::string robot_model_str="";
    while (!nodeHandle_.getParam(model_param_name, robot_model_str))
    {
      ROS_ERROR_STREAM(
        "Robot descripion couldn't be retrieved from param server.");
    }

    boost::shared_ptr<urdf::ModelInterface> model(
      urdf::parseURDF(robot_model_str));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> link =
      model->getLink(linearMotorFrame_);
    minElevation_ =
      link->parent_joint->parent_to_joint_origin_transform.position.z;
    ROS_INFO_STREAM("Got minElevation_ from URDF: " << minElevation_);

    return true;
  }

  void LinearMovementActionServer::testLinear()
  {
    tf::StampedTransform linearTransform;
    try
    {
      tfListener_.lookupTransform(
        "/base_link", linearMotorFrame_,
        ros::Time(0), linearTransform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }
    double startX = linearTransform.getOrigin()[2];
    double step = -0.006;
    if (startX + step < minElevation_)
    {
      step = - step;
    }
    std_msgs::Float64 targetPosition;
    targetPosition.data = previousTarget_ = startX + step - minElevation_;
    // Step could be a param
    linearCommandPublisher_.publish(targetPosition);

    ros::Time begin = ros::Time::now();

    double linearZ = -1;
    while (ros::ok() &&
      fabs(linearZ - (startX + step)) >= movementThreshold_)
    {
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearMotorFrame_,
          ros::Time(0), linearTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      linearZ = linearTransform.getOrigin()[2];
      if (ros::Time::now().toSec() - begin.toSec() > commandTimeout_)
      {
        ROS_DEBUG("%s: Aborted", actionName_.c_str());
        // set the action state to succeeded
        actionServer_.setAborted();
        return;
      }
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s: Preempted", actionName_.c_str());
        // set the action state to preempted
        actionServer_.setPreempted();
        return;
      }
    }
    ROS_DEBUG("%s: Succeeded", actionName_.c_str());
    // set the action state to succeeded
    actionServer_.setSucceeded();
  }

  void LinearMovementActionServer::lowerLinear()
  {
    std_msgs::Float64 targetPosition;
    targetPosition.data = previousTarget_ = 0;
    linearCommandPublisher_.publish(targetPosition);

    ros::Time begin = ros::Time::now();

    tf::StampedTransform linearTransform;
    double linearZ = -1;
    while (ros::ok() &&
      fabs(linearZ - minElevation_) >= movementThreshold_)
    {
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearMotorFrame_,
          ros::Time(0), linearTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      linearZ = linearTransform.getOrigin()[2];
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_WARN("%s: Preempted", actionName_.c_str());
        // set the action state to preempted
        actionServer_.setPreempted();
        return;
      }
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

  void LinearMovementActionServer::moveLinear(std::string pointOfInterest,
    std::string centerPoint)
  {
    ros::Time lastTf = ros::Time::now();
    ros::Rate rate(5);
    std_msgs::Float64 targetPosition;

    while (ros::ok())
    {
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_DEBUG("%s: Preempted", actionName_.c_str());
        // set the action state to preempted
        actionServer_.setPreempted();
        return;
      }

      tf::StampedTransform linearTransform;
      try
      {
        tfListener_.lookupTransform(
          "/base_link", linearMotorFrame_,
          ros::Time(0), linearTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      tf::StampedTransform linearToTargetTransform;
      try
      {
        tfListener_.lookupTransform(
          linearMotorFrame_, pointOfInterest,
          ros::Time(0), linearToTargetTransform);
      }
      catch (tf::TransformException ex)
      {
        if (ros::Time::now() - lastTf > ros::Duration(1))
        {
          if (actionServer_.isPreemptRequested() || !ros::ok())
          {
            ROS_WARN("%s: Preempted", actionName_.c_str());
            // set the action state to preempted
            actionServer_.setPreempted();
            return;
          }
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

      tf::StampedTransform linearToCenterTransform;
      try
      {
        tfListener_.lookupTransform(
          linearMotorFrame_, centerPoint,
          ros::Time(0), linearToCenterTransform);
      }
      catch (tf::TransformException ex)
      {
        if (ros::Time::now() - lastTf > ros::Duration(1))
        {
          if (actionServer_.isPreemptRequested() || !ros::ok())
          {
            ROS_WARN("%s: Preempted", actionName_.c_str());
            // set the action state to preempted
            actionServer_.setPreempted();
            return;
          }
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

      double deltaZ = linearToTargetTransform.getOrigin()[2]
        - linearToCenterTransform.getOrigin()[2];

      double targetZ = linearTransform.getOrigin()[2] + deltaZ;
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
      if (fabs(previousTarget_ - targetPosition.data) > movementThreshold_)
      {
        linearCommandPublisher_.publish(targetPosition);
        previousTarget_ = targetPosition.data;
      }
      rate.sleep();
    }
  }
}  // namespace pandora_control

int main(int argc, char **argv)
{
  ros::init(argc, argv, "linear_movement_action");
  ros::NodeHandle nodeHandle;
  std::string actionName = "linear_movement_action";

  pandora_control::LinearMovementActionServer
    linearMovementActionServer(
      actionName,
      nodeHandle);

  ros::spin();
}
