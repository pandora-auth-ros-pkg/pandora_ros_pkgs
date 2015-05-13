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
#ifndef PANDORA_DYNAMIXEL_HARDWARE_INTERFACE_JOINT_STATES_WRAPPER_H
#define PANDORA_DYNAMIXEL_HARDWARE_INTERFACE_JOINT_STATES_WRAPPER_H

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

namespace pandora_hardware_interface
{
namespace dynamixel
{
  class JointStatesWrapper
  {
    private:
      ros::NodeHandle nodeHandle_;

      sensor_msgs::JointState jointStatesMsg_;

      ros::Subscriber laserRollSubscriber_;
      ros::Subscriber laserPitchSubscriber_;
      ros::Subscriber kinectPitchSubscriber_;
      ros::Subscriber kinectYawSubscriber_;
      ros::Subscriber headPitchSubscriber_;
      ros::Subscriber headYawSubscriber_;
      ros::Publisher publisher_;

      ros::Timer timer;

      void laserRollCallback(const dynamixel_msgs::JointState& msg);
      void laserPitchCallback(const dynamixel_msgs::JointState& msg);
      void kinectPitchCallback(const dynamixel_msgs::JointState& msg);
      void kinectYawCallback(const dynamixel_msgs::JointState& msg);
      void headPitchCallback(const dynamixel_msgs::JointState& msg);
      void headYawCallback(const dynamixel_msgs::JointState& msg);
      void jointStatesCallback(const ros::TimerEvent&);
    public:
    JointStatesWrapper();
    ~JointStatesWrapper();
  };

  JointStatesWrapper::JointStatesWrapper()
  {
    jointStatesMsg_.name.push_back("laser_roll_joint");
    jointStatesMsg_.position.push_back(0);
    jointStatesMsg_.name.push_back("laser_pitch_joint");
    jointStatesMsg_.position.push_back(0);
    jointStatesMsg_.name.push_back("kinect_pitch_joint");
    jointStatesMsg_.position.push_back(0);
    jointStatesMsg_.name.push_back("kinect_yaw_joint");
    jointStatesMsg_.position.push_back(0);
    jointStatesMsg_.name.push_back("linear_head_pitch_joint");
    jointStatesMsg_.position.push_back(0);
    jointStatesMsg_.name.push_back("linear_head_yaw_joint");
    jointStatesMsg_.position.push_back(0);

    laserRollSubscriber_ = nodeHandle_.subscribe(
      "/laser_roll_controller/state",
      1,
      &JointStatesWrapper::laserRollCallback,
      this);

    laserPitchSubscriber_ = nodeHandle_.subscribe(
      "/laser_pitch_controller/state",
      1,
      &JointStatesWrapper::laserPitchCallback,
      this);

    kinectPitchSubscriber_ = nodeHandle_.subscribe(
      "/kinect_pitch_controller/state",
      1,
      &JointStatesWrapper::kinectPitchCallback,
      this);

    kinectYawSubscriber_ = nodeHandle_.subscribe(
      "/kinect_yaw_controller/state",
      1,
      &JointStatesWrapper::kinectYawCallback,
      this);

    headPitchSubscriber_ = nodeHandle_.subscribe(
      "/linear_head_pitch_controller/state",
      1,
      &JointStatesWrapper::headPitchCallback,
      this);

    headYawSubscriber_ = nodeHandle_.subscribe(
      "/linear_head_yaw_controller/state",
      1,
      &JointStatesWrapper::headYawCallback,
      this);

    publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>(
      "/dynamixel/joint_states",
      1);

    timer = nodeHandle_.createTimer(
      ros::Duration(0.01),
      &JointStatesWrapper::jointStatesCallback,
      this);
  }

  JointStatesWrapper::~JointStatesWrapper()
  {
  }

  void JointStatesWrapper::laserRollCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[0] = msg.current_pos;
  }

  void JointStatesWrapper::laserPitchCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[1] = msg.current_pos;
  }

  void JointStatesWrapper::kinectPitchCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[2] = msg.current_pos;
  }

  void JointStatesWrapper::kinectYawCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[3] = msg.current_pos;
  }

  void JointStatesWrapper::headPitchCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[4] = msg.current_pos;
  }

  void JointStatesWrapper::headYawCallback(const dynamixel_msgs::JointState& msg)
  {
    jointStatesMsg_.position[5] = msg.current_pos;
  }

  void JointStatesWrapper::jointStatesCallback(const ros::TimerEvent&)
  {
    jointStatesMsg_.header.stamp = ros::Time::now();
    publisher_.publish(jointStatesMsg_);
  }
}  // namespace dynamixel
}  // namespace pandora_hardware_interface

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_joints_state_wrapper");
  pandora_hardware_interface::dynamixel::JointStatesWrapper jointStatesWrapper;
  ros::spin();
}
#endif  // PANDORA_DYNAMIXEL_HARDWARE_INTERFACE_JOINT_STATES_WRAPPER_H
