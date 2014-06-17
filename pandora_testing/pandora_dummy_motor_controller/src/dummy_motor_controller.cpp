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
#include <std_msgs/Float64.h>


namespace pandora_hardware_interface
{
namespace dummy_motor_controller
{
  class DummyMotorController
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
      ros::Subscriber linearSubscriber_;
      ros::Publisher publisher_;

      ros::Timer timer;

      void laserRollCallback(const std_msgs::Float64& msg);
      void laserPitchCallback(const std_msgs::Float64& msg);
      void kinectPitchCallback(const std_msgs::Float64& msg);
      void kinectYawCallback(const std_msgs::Float64& msg);
      void headPitchCallback(const std_msgs::Float64& msg);
      void headYawCallback(const std_msgs::Float64& msg);
      void linearCallback(const std_msgs::Float64& msg);
      void jointStatesCallback(const ros::TimerEvent&);
    public:
    DummyMotorController();
    ~DummyMotorController();
  };

  DummyMotorController::DummyMotorController()
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
    jointStatesMsg_.name.push_back("linear_elevator_joint");
    jointStatesMsg_.position.push_back(0);

    laserRollSubscriber_ = nodeHandle_.subscribe(
      "/laser_roll_controller/command",
      1,
      &DummyMotorController::laserRollCallback,
      this);

    laserPitchSubscriber_ = nodeHandle_.subscribe(
      "/laser_pitch_controller/command",
      1,
      &DummyMotorController::laserPitchCallback,
      this);

    kinectPitchSubscriber_ = nodeHandle_.subscribe(
      "/kinect_pitch_controller/command",
      1,
      &DummyMotorController::kinectPitchCallback,
      this);

    kinectYawSubscriber_ = nodeHandle_.subscribe(
      "/kinect_yaw_controller/command",
      1,
      &DummyMotorController::kinectYawCallback,
      this);

    headPitchSubscriber_ = nodeHandle_.subscribe(
      "/linear_head_pitch_controller/command",
      1,
      &DummyMotorController::headPitchCallback,
      this);

    headYawSubscriber_ = nodeHandle_.subscribe(
      "/linear_head_yaw_controller/command",
      1,
      &DummyMotorController::headYawCallback,
      this);

    linearSubscriber_ = nodeHandle_.subscribe(
      "/linear/linear_elevator_controller/command",
      1,
      &DummyMotorController::linearCallback,
      this);

    publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>(
      "/dummy/joint_states",
      1);

    timer = nodeHandle_.createTimer(
      ros::Duration(0.01),
      &DummyMotorController::jointStatesCallback,
      this);
  }

  DummyMotorController::~DummyMotorController()
  {
  }

  void DummyMotorController::laserRollCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[0] = msg.data;
  }

  void DummyMotorController::laserPitchCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[1] = msg.data;
  }

  void DummyMotorController::kinectPitchCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[2] = msg.data;
  }

  void DummyMotorController::kinectYawCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[3] = msg.data;
  }

  void DummyMotorController::headPitchCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[4] = msg.data;
  }

  void DummyMotorController::headYawCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[5] = msg.data;
  }

  void DummyMotorController::linearCallback(const std_msgs::Float64& msg)
  {
    jointStatesMsg_.position[6] = msg.data;
  }

  void DummyMotorController::jointStatesCallback(const ros::TimerEvent&)
  {
    jointStatesMsg_.header.stamp = ros::Time::now();
    publisher_.publish(jointStatesMsg_);
  }
}  // namespace dummy_motor_controller
}  // namespace pandora_hardware_interface

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_motor_controller");
  pandora_hardware_interface::dummy_motor_controller::DummyMotorController
    dummyMotorController;
  ros::spin();
}
#endif  // PANDORA_DYNAMIXEL_HARDWARE_INTERFACE_JOINT_STATES_WRAPPER_H
