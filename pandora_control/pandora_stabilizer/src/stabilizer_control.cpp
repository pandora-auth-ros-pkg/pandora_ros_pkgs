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
#include "pandora_stabilizer_control/stabilizer_control.h"

namespace pandora_control
{
  StabilizerController::StabilizerController(void)
  {
    std::string compassTopic;
    if ( nodeHandle_.hasParam("compass_topic") )
    {
      nodeHandle_.getParam("compass_topic", compassTopic);
      ROS_DEBUG(
        "[stabilizer_control_node]: Got parameter compass_topic : %s",
        compassTopic.c_str());
    }
    else
    {
      ROS_DEBUG(
        "[stabilizer_control_node] : Parameter compass_topic not found. Using Default");
      compassTopic = "/sensors/imu";
    }
    if ( nodeHandle_.hasParam("buffer_size") )
    {
      nodeHandle_.getParam("buffer_size", bufferSize_);
      ROS_DEBUG_STREAM(
        "[stabilizer_control_node]: Got parameter buffer_size : "
        << bufferSize_);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[stabilizer_control_node] : Parameter buffer_size not found. Using Default 5");
      bufferSize_ = 5;
    }
    nodeHandle_.param("min_roll", minRoll_, -1.57);
    nodeHandle_.param("min_pitch", minPitch_, -1.57);
    nodeHandle_.param("max_roll", maxRoll_, 1.57);
    nodeHandle_.param("max_pitch", maxPitch_, 0.785);


    compassSubscriber_ = nodeHandle_.subscribe(
      compassTopic,
      1,
      &StabilizerController::serveImuMessage,
      this);

    laserRollPublisher_ = nodeHandle_.advertise<std_msgs::Float64>(
      "/laser_roll_controller/command",
      5);
    laserPitchPublisher_ = nodeHandle_.advertise<std_msgs::Float64>(
      "/laser_pitch_controller/command",
      5);

    for (int ii = 0; ii < bufferSize_; ii++)
    {
      rollBuffer_.push_back(0);
      pitchBuffer_.push_back(0);
    }
    bufferCounter_ = 0;
  }

  void StabilizerController::serveImuMessage(
    const sensor_msgs::ImuConstPtr& msg)
  {
    double compassYaw;
    double compassPitch;
    double compassRoll;
    std_msgs::Float64 str;

    tf::Matrix3x3 matrix(
      tf::Quaternion(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w));

    matrix.getRPY(compassRoll, compassPitch, compassYaw);

    rollBuffer_[bufferCounter_] = compassRoll;
    pitchBuffer_[bufferCounter_] = compassPitch;
    bufferCounter_ = fmod(bufferCounter_ + 1, bufferSize_);

    double command[2];
    command[0] = 0;
    command[1] = 0;

    for (int ii = 0; ii < bufferSize_; ii++)
    {
      command[0] = command[0] - rollBuffer_[ii] / bufferSize_;
      command[1] = command[1] - pitchBuffer_[ii] / bufferSize_;
    }
    if (command[0] < minRoll_)
    {
      command[0] = minRoll_;
    }
    else if (command[0] > maxRoll_)
    {
      command[0] = maxRoll_;
    }
    if (command[1] < minPitch_)
    {
      command[1] = minPitch_;
    }
    else if (command[1] > maxPitch_)
    {
      command[1] = maxPitch_;
    }
    str.data = command[0];
    laserRollPublisher_.publish(str);
    str.data = command[1];
    laserPitchPublisher_.publish(str);
  }
}  // namespace pandora_control
