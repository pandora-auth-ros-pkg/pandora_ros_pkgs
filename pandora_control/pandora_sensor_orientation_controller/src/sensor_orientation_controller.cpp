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
* Author:  Kostas Peppas
*********************************************************************/

#include <string>
#include <algorithm>
#include <limits>

#include <pandora_sensor_orientation_controller/sensor_orientation_controller.h>

namespace
{

  void clamp(
      double& val,
      const double min_val,
      const double max_val)
  {
    val = std::min(std::max(val, min_val), max_val);
  }

}  // namespace

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
    if (getcontrollerParams())
    {
      scanYawTimer_ = nodeHandle_.createTimer(
                      ros::Duration(scanRate_),
                      &SensorOrientationActionServer::scan,
                      this);
      scanPitchTimer_ = nodeHandle_.createTimer(
                       ros::Duration(pitchRate_),
                       &SensorOrientationActionServer::stabilizePitch,
                       this);

      pointSensorTimer_ = nodeHandle_.createTimer(
                         ros::Duration(pitchRate_),
                         &SensorOrientationActionServer::pointSensor,
                         this);
      stopPreviousTimers();

      if (scanRate_ <= 0)
      {
        ROS_DEBUG_STREAM("[" << actionName_ << "] Wrong scan rate value: "
          << scanRate_ << ", updating as fast as possible!");
        scanRate_ = 1;
      }

      sensorPitchPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          pitchCommandTopic_,
          5, true);

      sensorYawPublisher_ =
        nodeHandle_.advertise<std_msgs::Float64>(
          yawCommandTopic_,
          5, true);

      imuSubscriber_ = nodeHandle_.subscribe(
        imuTopic_,
        1,
        &SensorOrientationActionServer::imuCallback,
        this);

      for (int ii = 0; ii < bufferSize_; ii++)
      {
        pitchBuffer_.push_back(0);
      }
      bufferCounter_ = 0;

      std_msgs::Float64 targetPosition;
      targetPosition.data = 0;
      lastPitchTarget_ = 0;
      lastYawTarget_ = 0;
      sensorPitchPublisher_.publish(targetPosition);
      sensorYawPublisher_.publish(targetPosition);
      position_ = START;
      actionServer_.registerPreemptCallback(
        boost::bind(&SensorOrientationActionServer::preemptCallback, this));
      actionServer_.start();
    }
  }

  SensorOrientationActionServer::~SensorOrientationActionServer(void)
  {
  }

  void SensorOrientationActionServer::callback(
      const pandora_sensor_orientation_controller::MoveSensorGoalConstPtr& goal)
  {
    command_ = goal->command;
    pointOfInterest_ = goal->point_of_interest;
    pointThreshold_ = movementThreshold_;
    stopPreviousTimers();
    if (command_ ==
        pandora_sensor_orientation_controller::MoveSensorGoal::TEST)
    {
      testSensor();
    }
    else if (command_ ==
             pandora_sensor_orientation_controller::MoveSensorGoal::CENTER)
    {
      centerSensor();
    }
    else if (command_ ==
             pandora_sensor_orientation_controller::MoveSensorGoal::MOVE)
    {
      scanYawTimer_.start();
      scanPitchTimer_.start();
    }
    else if (command_ ==
             pandora_sensor_orientation_controller::MoveSensorGoal::POINT)
    {
      pointThreshold_ = movementThreshold_;
      pointSensorTimer_.start();
    }
    else if (command_ ==
             pandora_sensor_orientation_controller::MoveSensorGoal::LAX_POINT)
    {
      pointThreshold_ = laxMovementThreshold_;
      pointSensorTimer_.start();
    }
    else
    {
      ROS_ERROR("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
    }
    while (actionServer_.isActive())
    {
    }
  }

  void SensorOrientationActionServer::preemptCallback()
  {
    ROS_INFO("%s: Preempted", actionName_.c_str());
    actionServer_.setPreempted();
    stopPreviousTimers();
  }

  bool SensorOrientationActionServer::getcontrollerParams()
  {
    nodeHandle_.param(actionName_ + "/pitch_step", pitchStep_, 0.4);
    nodeHandle_.param(actionName_ + "/yaw_step", yawStep_, 0.7);
    nodeHandle_.param(actionName_ + "/min_pitch", minPitch_, -1.0);
    nodeHandle_.param(actionName_ + "/min_yaw", minYaw_, -1.0);
    nodeHandle_.param(actionName_ + "/max_pitch", maxPitch_, 1.0);
    nodeHandle_.param(actionName_ + "/max_yaw", maxYaw_, 1.0);
    nodeHandle_.param(actionName_ + "/offset_pitch", offsetPitch_, 0.0);
    nodeHandle_.param(actionName_ + "/offset_yaw", offsetYaw_, 0.0);
    nodeHandle_.param(actionName_ + "/command_timeout", commandTimeout_, 3.0);
    nodeHandle_.param(actionName_ + "/movement_threshold",
                      movementThreshold_, 0.017);
    nodeHandle_.param(actionName_ + "/lax_movement_threshold",
                      laxMovementThreshold_, 0.2);
    nodeHandle_.param("buffer_size", bufferSize_, 5);
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
    nodeHandle_.param(actionName_ + "/scan_rate", scanRate_, 1.0);

    if (nodeHandle_.getParam(actionName_ + "/imu_topic",
      imuTopic_))
    {
      ROS_INFO_STREAM("Got param imu_topic: " << imuTopic_);
    }
    else
    {
      ROS_FATAL("Failed to get param imu_topic shuting down");
      return false;
    }

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

    nodeHandle_.param(actionName_ + "/pitch_rate", pitchRate_, 0.2);

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

    if (nodeHandle_.getParam(actionName_ + "/pitch_joint_parent",
      pitchJointParent_))
    {
      ROS_INFO_STREAM("Got param pitch_joint_parent: " << pitchJointParent_);
    }
    else
    {
      ROS_FATAL("Failed to get param pitch_joint_parent shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/pitch_joint_child",
      pitchJointChild_))
    {
      ROS_INFO_STREAM("Got param pitch_joint_child: " << pitchJointChild_);
    }
    else
    {
      ROS_FATAL("Failed to get param pitch_joint_child shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/yaw_joint_parent",
      yawJointParent_))
    {
      ROS_INFO_STREAM("Got param yaw_joint_parent: " << yawJointParent_);
    }
    else
    {
      ROS_FATAL("Failed to get param yaw_joint_parent shuting down");
      return false;
    }

    if (nodeHandle_.getParam(actionName_ + "/yaw_joint_child",
      yawJointChild_))
    {
      ROS_INFO_STREAM("Got param yaw_joint_child: " << yawJointChild_);
    }
    else
    {
      ROS_FATAL("Failed to get param yaw_joint_child shuting down");
      return false;
    }
    return true;
  }

  void SensorOrientationActionServer::testSensor()
  {
    pitchTargetPosition_.data = pitchStep_;
    yawTargetPosition_.data = yawStep_;
    position_ = UNKNOWN;
    lastPitchTarget_ = pitchTargetPosition_.data;
    lastYawTarget_ = yawTargetPosition_.data;
    sensorPitchPublisher_.publish(pitchTargetPosition_);
    sensorYawPublisher_.publish(yawTargetPosition_);

    setGoalState(
      checkGoalCompletion());
  }

  void SensorOrientationActionServer::centerSensor()
  {
    if (position_ != START)
    {
      pitchTargetPosition_.data = offsetPitch_;
      yawTargetPosition_.data = offsetYaw_;
      lastPitchTarget_ = pitchTargetPosition_.data;
      lastYawTarget_ = yawTargetPosition_.data;
      sensorPitchPublisher_.publish(pitchTargetPosition_);
      sensorYawPublisher_.publish(yawTargetPosition_);
      position_ = START;
      setGoalState(
        checkGoalCompletion());
    }
    else
    {
      ROS_INFO("%s: Succeeded", actionName_.c_str());
      actionServer_.setSucceeded();
    }
  }


  void SensorOrientationActionServer::scan(const ros::TimerEvent& event)
  {
    switch (position_)
    {
      case START:
        yawScan_ = yawStep_;
        pitchScan_ = 0;
        position_ = UP_LEFT_ONE;
        break;
      case UP_LEFT_ONE:
        yawScan_ = 2*yawStep_;
        pitchScan_ = 0;
        position_ = UP_LEFT_TWO;
        break;
      case UP_LEFT_TWO:
        yawScan_ = yawStep_;
        pitchScan_ = pitchStep_;
        position_ = DOWN_LEFT;
        break;
      case DOWN_LEFT:
        yawScan_ = 0;
        pitchScan_ = pitchStep_;
        position_ = DOWN_CENTER;
        break;
      case DOWN_CENTER:
        yawScan_ = -yawStep_;
        pitchScan_ = pitchStep_;
        position_ = DOWN_RIGHT;
        break;
      case DOWN_RIGHT:
        yawScan_ = -2*yawStep_;
        pitchScan_ = 0;
        position_ = UP_RIGHT_TWO;
        break;
      case UP_RIGHT_TWO:
        yawScan_ = -yawStep_;
        pitchScan_ = 0;
        position_ = UP_RIGHT_ONE;
        break;
      case UP_RIGHT_ONE:
        yawScan_ = 0;
        pitchScan_ = 0;
        position_ = UP_CENTER;
        break;
      case UP_CENTER:
        yawScan_ = yawStep_;
        pitchScan_ = 0;
        position_ = UP_LEFT_ONE;
        break;
      case UNKNOWN:
        yawScan_ = 0;
        pitchScan_ = 0;
        position_ = UP_CENTER;
        break;
    }

    publishScanPitchCommand();
    publishScanYawCommand();
  }

  void SensorOrientationActionServer::stabilizePitch(const ros::TimerEvent& event)
  {
    publishScanPitchCommand();
  }

  void SensorOrientationActionServer::imuCallback(
    const pandora_sensor_msgs::ImuRPYConstPtr& msg)
  {
    double imuPitch;
    std_msgs::Float64 str;

    imuPitch = msg->pitch;

    pitchBuffer_[bufferCounter_] = imuPitch;
    bufferCounter_ = fmod(bufferCounter_ + 1, bufferSize_);

    stabilizedPitch_ = 0;

    for (int ii = 0; ii < bufferSize_; ii++)
    {
      stabilizedPitch_ += pitchBuffer_[ii] / bufferSize_;
    }

    // Invert command in order to stabilize
    stabilizedPitch_ *= -1;

    // Convert degs to rads
    stabilizedPitch_ *= 2 * boost::math::constants::pi<double>() / 360.0;
  }

  void SensorOrientationActionServer::publishScanPitchCommand()
  {
    pitchTargetPosition_.data = offsetPitch_ + pitchScan_ + stabilizedPitch_;
    clamp(pitchTargetPosition_.data, minPitch_, maxPitch_);
    lastPitchTarget_ = pitchTargetPosition_.data;
    sensorPitchPublisher_.publish(pitchTargetPosition_);
  }

  void SensorOrientationActionServer::publishScanYawCommand()
  {
    yawTargetPosition_.data = yawScan_ + offsetYaw_;
    clamp(yawTargetPosition_.data, minYaw_, maxYaw_);
    lastYawTarget_ = yawTargetPosition_.data;
    sensorYawPublisher_.publish(yawTargetPosition_);
  }

  void SensorOrientationActionServer::pointSensor(const ros::TimerEvent& event)
  {
    ros::Time lastTf = ros::Time::now();
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
        "/base_link", pointOfInterest_,
        ros::Time(0), targetTransform);
    }
    catch (tf::TransformException ex)
    {
      if (ros::Time::now() - lastTf > ros::Duration(1))
      {
        ROS_ERROR_STREAM("Is " << pointOfInterest_ << " broadcasted?");
        ROS_ERROR("%s: Aborted", actionName_.c_str());

        // set the action state to succeeded
        actionServer_.setAborted();
        stopPreviousTimers();
        return;
      }
      else
      {
        return;
      }
    }
    lastTf = ros::Time::now();
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

    pitchTargetPosition_.data = pitch + offsetPitch_;
    yawTargetPosition_.data = yaw + offsetYaw_;
    checkAngleLimits();
    if (fabs(lastPitchTarget_ - pitchTargetPosition_.data) > pointThreshold_
      || fabs(lastYawTarget_ - yawTargetPosition_.data) > pointThreshold_)
    {
      lastPitchTarget_ = pitchTargetPosition_.data;
      lastYawTarget_ = yawTargetPosition_.data;
      sensorPitchPublisher_.publish(pitchTargetPosition_);
      sensorYawPublisher_.publish(yawTargetPosition_);
      position_ = UNKNOWN;
    }
  }

  void SensorOrientationActionServer::stopPreviousTimers()
  {
    scanYawTimer_.stop();
    scanPitchTimer_.stop();
    pointSensorTimer_.stop();
  }

  int SensorOrientationActionServer::checkGoalCompletion()
  {
    ros::Time begin = ros::Time::now();
    tf::StampedTransform pitchTransform;
    tf::StampedTransform yawTransform;
    double roll, pitch, yaw, tempPitch;
    pitch = std::numeric_limits<double>::max();
    yaw = std::numeric_limits<double>::max();

    while (ros::ok() && (
      fabs(pitch - pitchTargetPosition_.data) >= pointThreshold_ ||
      fabs(yaw - yawTargetPosition_.data) >= pointThreshold_))
    {
      try
      {
        tfListener_.lookupTransform(
          pitchJointParent_, pitchJointChild_,
          ros::Time(0), pitchTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      pitchTransform.getBasis().getRPY(roll, pitch, yaw);

      try
      {
        tfListener_.lookupTransform(
          yawJointParent_, yawJointChild_,
          ros::Time(0), yawTransform);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
      }
      yawTransform.getBasis().getRPY(roll, tempPitch, yaw);

      if (ros::Time::now().toSec() - begin.toSec() > commandTimeout_)
      {
        return 1;
      }
      if (actionServer_.isPreemptRequested() || !ros::ok())
      {
        return 2;
      }
    }
    return 0;
  }

  void SensorOrientationActionServer::setGoalState(int state)
  {
    switch (state)
    {
      case 0:
        ROS_INFO("%s: Succeeded", actionName_.c_str());
        actionServer_.setSucceeded();
        break;
      case 1:
        ROS_INFO("%s: Aborted", actionName_.c_str());
        actionServer_.setAborted();
        break;
      case 2:
        ROS_INFO("%s: Preempted", actionName_.c_str());
        actionServer_.setPreempted();
        break;
    }
  }
  void SensorOrientationActionServer::checkAngleLimits()
  {
    clamp(pitchTargetPosition_.data, minPitch_, maxPitch_);
    clamp(yawTargetPosition_.data, minYaw_, maxYaw_);
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
