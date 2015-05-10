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
    std::string imuTopic;

    if ( nodeHandle_.hasParam("imu_topic") )
    {
      nodeHandle_.getParam("imu_topic", imuTopic);
      ROS_DEBUG(
        "[stabilizer_control_node]: Got parameter imu_topic : %s",
        imuTopic.c_str());
    }
    else
    {
      ROS_DEBUG(
        "[stabilizer_control_node] : Parameter imu_topic not found. Using Default");
      imuTopic = "/sensors/imu";
    }

    std::string rollCommandTopic;

    if ( nodeHandle_.hasParam("roll_command_topic") )
    {
      nodeHandle_.getParam("roll_command_topic", rollCommandTopic);
      ROS_DEBUG(
        "[stabilizer_control_node]: Got parameter roll_command_topic : %s",
        rollCommandTopic.c_str());
    }
    else
    {
      ROS_DEBUG(
        "[stabilizer_control_node] : Parameter roll_command_topic not found. Using Default");
      rollCommandTopic = "/laser_roll_controller/command";
    }

    std::string pitchCommandTopic;

    if ( nodeHandle_.hasParam("pitch_command_topic") )
    {
      nodeHandle_.getParam("pitch_command_topic", pitchCommandTopic);
      ROS_DEBUG(
        "[stabilizer_control_node]: Got parameter pitch_command_topic : %s",
        pitchCommandTopic.c_str());
    }
    else
    {
      ROS_DEBUG(
        "[stabilizer_control_node] : Parameter pitch_command_topic not found. Using Default");
      pitchCommandTopic = "/laser_pitch_controller/command";
    }
    nodeHandle_.param("buffer_size", bufferSize_, 5);
    nodeHandle_.param("min_roll", minRoll_, -1.57);
    nodeHandle_.param("min_pitch", minPitch_, -1.57);
    nodeHandle_.param("max_roll", maxRoll_, 1.57);
    nodeHandle_.param("max_pitch", maxPitch_, 0.785);
    nodeHandle_.param("roll_offset", rollOffset_, -1.57);
    nodeHandle_.param("pitch_offset", pitchOffset_, 0.0);

    imuSubscriber_ = nodeHandle_.subscribe(
      imuTopic,
      1,
      &StabilizerController::serveImuMessage,
      this);

    laserRollPublisher_ = nodeHandle_.advertise<std_msgs::Float64>(
      rollCommandTopic,
      5);
    laserPitchPublisher_ = nodeHandle_.advertise<std_msgs::Float64>(
      pitchCommandTopic,
      5);

    for (int ii = 0; ii < bufferSize_; ii++)
    {
      rollBuffer_.push_back(0);
      pitchBuffer_.push_back(0);
    }
    bufferCounter_ = 0;
  }

  void StabilizerController::serveImuMessage(
    const pandora_sensor_msgs::ImuRPYConstPtr& msg)
  {
    double imuRoll;
    double imuPitch;
    std_msgs::Float64 str;

    imuRoll = msg->roll;
    imuPitch = msg->pitch;

    rollBuffer_[bufferCounter_] = imuRoll;
    pitchBuffer_[bufferCounter_] = imuPitch;
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
    str.data = command[0] + rollOffset_;
    laserRollPublisher_.publish(str);
    str.data = command[1] + pitchOffset_;
    laserPitchPublisher_.publish(str);
  }
}  // namespace pandora_control
