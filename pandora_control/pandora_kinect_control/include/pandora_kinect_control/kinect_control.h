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
#ifndef PANDORA_KINECT_CONTROL_KINECT_CONTROL_H
#define PANDORA_KINECT_CONTROL_KINECT_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <actionlib/server/simple_action_server.h>
#include <pandora_kinect_control/MoveKinectAction.h>

namespace pandora_control
{
  enum
  {
    START = 0,
    LEFT = 1,
    CENTER = 2,
    RIGHT = 3
  };

  class PandoraMoveKinectActionServer
  {
    private:
    ros::NodeHandle nodeHandle_;
      actionlib::SimpleActionServer<
        pandora_kinect_control::MoveKinectAction> actionServer_;
      std::string actionName_;

      ros::Publisher kinect_pitch_publisher;
      ros::Publisher kinect_yaw_publisher;

      ros::Subscriber _compassSubscriber;
      void compassCallback(
        const sensor_msgs::ImuConstPtr& msg);
      int position_;
      int command_;

      ros::Timer timer_;

      void goalCallback();

      void preemptCallback();

      void timerCallback(const ros::TimerEvent&);
    public:
      PandoraMoveKinectActionServer(
        std::string name,
        ros::NodeHandle nodeHandle_);

      ~PandoraMoveKinectActionServer(void);
  };
}  // namespace pandora_control
#endif  // PANDORA_KINECT_CONTROL_KINECT_CONTROL_H
