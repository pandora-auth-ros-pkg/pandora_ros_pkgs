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

#include <pandora_kinect_control/kinect_control.h>

namespace pandora_control
{
  PandoraMoveKinectActionServer::PandoraMoveKinectActionServer(
    std::string name,
    ros::NodeHandle nodeHandle)
  :
    actionServer_(
      nodeHandle,
      name,
      boost::bind(&PandoraMoveKinectActionServer::callback, this, _1), false),
    actionName_(name),
    nodeHandle_(nodeHandle)
  {
    // get params from param server
    nodeHandle_.param("max_pitch", maxPitch_, 0.4);
    nodeHandle_.param("max_yaw", maxYaw_, 0.7);
    nodeHandle_.param("time_step", timeStep_, 1.0);

    if (timeStep_ <= 0) {
      ROS_WARN_STREAM("[kinect_control] Wrong time step value: " << timeStep_
         << ", updating as fast as possible!");
      timeStep_ = 0.01;
    }

    kinect_pitch_publisher =
      nodeHandle_.advertise<std_msgs::Float64>(
        "kinect_pitch_controller/command",
        5, true);

    kinect_yaw_publisher =
      nodeHandle_.advertise<std_msgs::Float64>(
        "kinect_yaw_controller/command",
        5, true);

    std_msgs::Float64 targetPosition;
    targetPosition.data = 0;
    kinect_pitch_publisher.publish(targetPosition);
    kinect_yaw_publisher.publish(targetPosition);
    position_ = CENTER;


    actionServer_.start();
  }

  PandoraMoveKinectActionServer::~PandoraMoveKinectActionServer(void)
  {
  }

  void PandoraMoveKinectActionServer::callback(
      const pandora_kinect_control::MoveKinectGoalConstPtr& goal)
  {
    command_ = goal->command;

    if (command_ == pandora_kinect_control::MoveKinectGoal::CENTER )
    {
      if (position_ != CENTER)
      {
        std_msgs::Float64 pitchTargetPosition, yawTargetPosition;
        pitchTargetPosition.data = 0;
        yawTargetPosition.data = 0;
        kinect_pitch_publisher.publish(pitchTargetPosition);
        kinect_yaw_publisher.publish(yawTargetPosition);
        position_ = CENTER;
      }
      ROS_DEBUG("%s: Succeeded", actionName_.c_str());
      // set the action state to succeeded
      actionServer_.setSucceeded();
      return;
    }
    else if (command_ == pandora_kinect_control::MoveKinectGoal::MOVE)
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
          case CENTER:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = maxYaw_;
            position_ = HIGH_LEFT;
            break;
          case HIGH_LEFT:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = maxYaw_;
            position_ = LOW_LEFT;
            break;
          case LOW_LEFT:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = 0;
            position_ = LOW_CENTER;
            break;
          case LOW_CENTER:
            pitchTargetPosition.data = maxPitch_;
            yawTargetPosition.data = -maxYaw_;
            position_ = LOW_RIGHT;
            break;
          case LOW_RIGHT:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = -maxYaw_;
            position_ = HIGH_RIGHT;
            break;
          case HIGH_RIGHT:
            pitchTargetPosition.data = 0;
            yawTargetPosition.data = 0;
            position_ = CENTER;
            break;
        }
        kinect_pitch_publisher.publish(pitchTargetPosition);
        kinect_yaw_publisher.publish(yawTargetPosition);
        rate.sleep();
      }
    }
    else
    {
      ROS_DEBUG("%s: Aborted, there is no such command", actionName_.c_str());
      // set the action state to aborted
      actionServer_.setAborted();
      return;
    }
  }
}  // namespace pandora_control
