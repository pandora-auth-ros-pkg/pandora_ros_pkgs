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

#include <sensor_orientation_planner/sensor_orientation_planner.h>

namespace pandora_control
{
  SensorOrientationActionServer::SensorOrientationActionServer(
    std::string actionName,
    ros::NodeHandle nodeHandle)
  :
    actionServer_(
      nodeHandle,
      actionName,
      boost::bind(&SensorOrientationActionServer::callback, this, _1), false),
    actionName_(actionName),
    nodeHandle_(nodeHandle)
  {
    // get params from param server
    if (getPlannerParams())
    {
      if (timeStep_ <= 0) {
        ROS_WARN_STREAM("[" << actionName_ << "] Wrong time step value: "
          << timeStep_ << ", updating as fast as possible!");
        timeStep_ = 0.01;
      }

      sensorPitchPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          pitchCommandTopic_,
          5, true);

      sensorYawPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          yawCommandTopic_,
          5, true);

      std_msgs::Float64 targetPosition;
      targetPosition.data = 0;
      sensorPitchPublisher_.publish(targetPosition);
      sensorYawPublisher_.publish(targetPosition);
      position_ = HIGH_START;

      actionServer_.start();
    }
  }

  SensorOrientationActionServer::~SensorOrientationActionServer(void)
  {
  }

  void SensorOrientationActionServer::callback(
      const pandora_end_effector_planner::MoveSensorGoalConstPtr& goal)
  {
    command_ = goal->command;
    if (command_ == pandora_end_effector_planner::MoveSensorGoal::CENTER )
    {
      centerSensor();
    }
    else if (command_ == pandora_end_effector_planner::MoveSensorGoal::MOVE)
    {
      scan();
    }
    else if (command_ == pandora_end_effector_planner::MoveSensorGoal::POINT)
    {
      pointSensor(goal->point_of_interest);
    }
    else
    {
      ROS_DEBUG("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
    }
  }

  bool SensorOrientationActionServer::getPlannerParams()
  {
    nodeHandle_.param(actionName_ + "/pitch_step", pitchStep_, 0.4);
    nodeHandle_.param(actionName_ + "/yaw_step", yawStep_, 0.7);
    nodeHandle_.param(actionName_ + "/min_pitch", minPitch_, -1.0);
    nodeHandle_.param(actionName_ + "/min_yaw", minYaw_, -1.0);
    nodeHandle_.param(actionName_ + "/max_pitch", maxPitch_, 1.0);
    nodeHandle_.param(actionName_ + "/max_yaw", maxYaw_, 1.0);
    if (pitchStep_ > maxPitch_ || pitchStep_ < minPitch_)
    {
      if (maxPitch_ < fabs(minPitch_))
      {
        pitchStep_ = maxPitch_;
      }
      else
      {
        pitchStep_ = fabs(minPitch_);
      }
    }
    if (yawStep_ > maxYaw_ || yawStep_ < minYaw_)
    {
      if (maxYaw_ < fabs(minYaw_))
      {
        yawStep_ = maxYaw_;
      }
      else
      {
        yawStep_ = fabs(minYaw_);
      }
    }
    nodeHandle_.param(actionName_ + "/time_step", timeStep_, 1.0);

    if (nodeHandle_.getParam(actionName_ + "/pitch_command_topic",
      pitchCommandTopic_))
    {
      ROS_INFO_STREAM("Got param pitch_command_topic: " << pitchCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param pitch_command_topic shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/yaw_command_topic",
      yawCommandTopic_))
    {
      ROS_INFO_STREAM("Got param yaw_command_topic: " << yawCommandTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param yaw_command_topic shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/sensor_frame",
      sensorFrame_))
    {
      ROS_INFO_STREAM("Got param sensor_frame: " << sensorFrame_);
    }
    else
    {
      ROS_FATAL("Failed to get param sensor_frame shuting down");
      return false;
    }
    return true;
  }

  void SensorOrientationActionServer::centerSensor()
  {
    if (position_ != HIGH_START || position_ != HIGH_CENTER)
    {
      std_msgs::Float64 pitchTargetPosition, yawTargetPosition;
      pitchTargetPosition.data = 0;
      yawTargetPosition.data = 0;
      sensorPitchPublisher_.publish(pitchTargetPosition);
      sensorYawPublisher_.publish(yawTargetPosition);
      position_ = HIGH_CENTER;
    }
    ROS_DEBUG("%s: Succeeded", actionName_.c_str());
    // set the action state to succeeded
    actionServer_.setSucceeded();
  }

  void SensorOrientationActionServer::scan()
  {
    ros::Rate rate(1/timeStep_);
    std_msgs::Float64 pitchTargetPosition, yawTargetPosition;

    while (ros::ok())
    {
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_DEBUG("%s: Preempted", actionName_.c_str());
        // set the action state to preempted
        actionServer_.setPreempted();
        return;
      }

      switch (position_)
      {
        case HIGH_START:
          pitchTargetPosition.data = pitchStep_;
          yawTargetPosition.data = 0;
          position_ = LOW_START;
          break;
        case LOW_START:
          pitchTargetPosition.data = 0;
          yawTargetPosition.data = yawStep_;
          position_ = HIGH_LEFT;
          break;
        case HIGH_LEFT:
          pitchTargetPosition.data = pitchStep_;
          yawTargetPosition.data = yawStep_;
          position_ = LOW_LEFT;
          break;
        case LOW_LEFT:
          pitchTargetPosition.data = 0;
          yawTargetPosition.data = 0;
          position_ = HIGH_CENTER;
          break;
        case HIGH_CENTER:
          pitchTargetPosition.data = pitchStep_;
          yawTargetPosition.data = 0;
          position_ = LOW_CENTER;
          break;
        case LOW_CENTER:
          pitchTargetPosition.data = 0;
          yawTargetPosition.data = -yawStep_;
          position_ = HIGH_RIGHT;
          break;
        case HIGH_RIGHT:
          pitchTargetPosition.data = pitchStep_;
          yawTargetPosition.data = -yawStep_;
          position_ = LOW_RIGHT;
          break;
        case LOW_RIGHT:
          pitchTargetPosition.data = 0;
          yawTargetPosition.data = 0;
          position_ = HIGH_START;
          break;
      }
      sensorPitchPublisher_.publish(pitchTargetPosition);
      sensorYawPublisher_.publish(yawTargetPosition);
      rate.sleep();
    }
  }

  void SensorOrientationActionServer::pointSensor(std::string pointOfInterest)
  {
    ros::Rate rate(5);
    std_msgs::Float64 pitchTargetPosition, yawTargetPosition;

    while (ros::ok())
    {
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        ROS_DEBUG("%s: Preempted", actionName_.c_str());
        // set the action state to preempted
        actionServer_.setPreempted();
        return;
      }
      tf::StampedTransform sensorTransform;
      try
      {
        tfListener_.lookupTransform(
          "/base_link", sensorFrame_,
          ros::Time(0), sensorTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }

      tf::StampedTransform targetTransform;
      try
      {
        tfListener_.lookupTransform(
          "/base_link", pointOfInterest,
          ros::Time(0), targetTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_DEBUG_STREAM("Is " << pointOfInterest << " broadcasted?");
        continue;
      }

      tf::Vector3 desiredVectorX;
      desiredVectorX = targetTransform.getOrigin() - sensorTransform.getOrigin();
      desiredVectorX = desiredVectorX.normalized();

      tf::Vector3 baseVectorZ = targetTransform.getBasis().getColumn(2);

      tf::Vector3 desiredVectorZ = baseVectorZ
        - desiredVectorX*tfDot(desiredVectorX, baseVectorZ);
      desiredVectorZ = desiredVectorZ.normalized();

      tf::Vector3 desiredVectorY = tfCross(desiredVectorZ, desiredVectorX);
      desiredVectorY = desiredVectorY.normalized();

      tf::Matrix3x3 desiredCameraBasis(
        desiredVectorX[0], desiredVectorY[0], desiredVectorZ[0],
        desiredVectorX[1], desiredVectorY[1], desiredVectorZ[1],
        desiredVectorX[2], desiredVectorY[2], desiredVectorZ[2]);

      double roll, pitch, yaw;
      desiredCameraBasis.getRPY(roll, pitch, yaw);

      pitchTargetPosition.data = pitch;
      yawTargetPosition.data = yaw;
      if (pitchTargetPosition.data < minPitch_)
      {
        pitchTargetPosition.data = minPitch_;
      }
      else if (pitchTargetPosition.data > maxPitch_)
      {
        pitchTargetPosition.data = maxPitch_;
      }
      if (yawTargetPosition.data < minYaw_)
      {
        yawTargetPosition.data = minYaw_;
      }
      else if (yawTargetPosition.data > maxYaw_)
      {
        yawTargetPosition.data = maxYaw_;
      }
      sensorPitchPublisher_.publish(pitchTargetPosition);
      sensorYawPublisher_.publish(yawTargetPosition);
      rate.sleep();
    }
  }
}  // namespace pandora_control

int main(int argc, char **argv)
{
  if (argc != 4)
  {
    ROS_FATAL_STREAM("No arguement passed. Action name is required");
    return 1;
  }

  ros::init(argc, argv, argv[1]);
  ros::NodeHandle nodeHandle;
  std::string actionName = argv[1];

  pandora_control::SensorOrientationActionServer
    sensorOrientationActionServer(
      actionName,
      nodeHandle);

  ros::spin();
}
