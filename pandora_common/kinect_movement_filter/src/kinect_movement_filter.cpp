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
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "ros/ros.h"

namespace pandora_common
{
  class KinectMovementFilter
  {
    private:
      ros::NodeHandle nodeHandle_;
      ros::Subscriber jointStateSubscriber_;
      ros::Subscriber imageSubscriber_;
      ros::Subscriber depthImageSubscriber_;
      ros::Subscriber pointCloudSubscriber_;
      ros::Subscriber pitchCommandSubscriber_;
      ros::Subscriber yawCommandSubscriber_;

      ros::Publisher imagePublisher_;
      ros::Publisher depthImagePublisher_;
      ros::Publisher pointCloudPublisher_;

      double pitchCommand_;
      double yawCommand_;
      double pitchError_;
      double yawError_;
      double errorThreshold_;

      std::string imageTopic_;
      std::string depthTopic_;
      std::string pointCloudTopic_;

      void pitchCommandCallback(const std_msgs::Float64ConstPtr& msg);
      void yawCommandCallback(const std_msgs::Float64ConstPtr& msg);
      void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg);
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);
      void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
      void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
    public:
      KinectMovementFilter();
      ~KinectMovementFilter();
  };

  KinectMovementFilter::KinectMovementFilter()
  {
    if ( nodeHandle_.hasParam("/kinect_movement_filter/error_threshold") )
    {
      nodeHandle_.getParam("/kinect_movement_filter/error_threshold", errorThreshold_);
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter]: Got parameter errorThreshold : " <<
        errorThreshold_);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter] : " <<
        "Parameter errorThreshold not found. Using Default 0.012");
      errorThreshold_ = 0.012;
    }

    if ( nodeHandle_.hasParam("/kinect_movement_filter/image_topic") )
    {
      nodeHandle_.getParam("/kinect_movement_filter/image_topic", imageTopic_);
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter]: Got parameter image_topic : " <<
        imageTopic_);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter] : " <<
        "Parameter image_topic not found. Using Default /kinect/rgb/image_raw");
      imageTopic_ = "/kinect/rgb/image_raw";
    }

    if ( nodeHandle_.hasParam("/kinect_movement_filter/depth_topic") )
    {
      nodeHandle_.getParam("/kinect_movement_filter/depth_topic", depthTopic_);
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter]: Got parameter depth_topic : " <<
        depthTopic_);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter] : " <<
        "Parameter depth_topic not found. Using Default /kinect/depth/image");
      depthTopic_ = "/kinect/depth/image";
    }

    if ( nodeHandle_.hasParam("/kinect_movement_filter/point_cloud_topic") )
    {
      nodeHandle_.getParam("/kinect_movement_filter/point_cloud_topic", pointCloudTopic_);
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter]: Got parameter point_cloud_topic : " <<
        pointCloudTopic_);
    }
    else
    {
      ROS_DEBUG_STREAM(
        "[kinect_movement_filter] : " <<
        "Parameter point_cloud_topic not found. " <<
        "Using Default /kinect/depth_registered/points");
      pointCloudTopic_ = "/kinect/depth_registered/points";
    }

    pitchCommand_ = 0;
    yawCommand_ = 0;
    pitchError_ = errorThreshold_;
    yawError_ = errorThreshold_;

    pitchCommandSubscriber_ = nodeHandle_.subscribe(
      "/kinect_pitch_controller/command",
      1,
      &KinectMovementFilter::pitchCommandCallback,
      this);

    yawCommandSubscriber_ = nodeHandle_.subscribe(
      "/kinect_yaw_controller/command",
      1,
      &KinectMovementFilter::yawCommandCallback,
      this);

    jointStateSubscriber_ = nodeHandle_.subscribe(
      "/joint_states",
      1,
      &KinectMovementFilter::jointStatesCallback,
      this);

    imageSubscriber_ = nodeHandle_.subscribe(
      imageTopic_,
      1,
      &KinectMovementFilter::imageCallback,
      this);

    depthImageSubscriber_ = nodeHandle_.subscribe(
      depthTopic_,
      1,
      &KinectMovementFilter::depthImageCallback,
      this);

    pointCloudSubscriber_ = nodeHandle_.subscribe(
      pointCloudTopic_,
      1,
      &KinectMovementFilter::pointCloudCallback,
      this);

    imagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(
      imageTopic_ + "/still",
      1);

    depthImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(
      depthTopic_ + "/still",
      1);

    pointCloudPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(
      pointCloudTopic_ + "/still",
      1);
  }

  KinectMovementFilter::~KinectMovementFilter()
  {
  }

  void KinectMovementFilter::pitchCommandCallback(
    const std_msgs::Float64ConstPtr& msg)
  {
    pitchCommand_ = msg->data;
  }

  void KinectMovementFilter::yawCommandCallback(
    const std_msgs::Float64ConstPtr& msg)
  {
    yawCommand_ = msg->data;
  }

  void KinectMovementFilter::jointStatesCallback(
    const sensor_msgs::JointStateConstPtr& msg)
  {
    for (int ii = 0; ii < msg->name.size(); ii++)
    {
      if (msg->name[ii] == "kinect_pitch_joint")
      {
        pitchError_ = msg->position[ii] - pitchCommand_;
      }
      if (msg->name[ii] == "kinect_yaw_joint")
      {
        yawError_ = msg->position[ii] - yawCommand_;
      }
    }
  }

  void KinectMovementFilter::imageCallback(
    const sensor_msgs::ImageConstPtr& msg)
  {
    if (fabs(pitchError_) < fabs(errorThreshold_) &&
      fabs(yawError_) < fabs(errorThreshold_))
    {
      imagePublisher_.publish(*msg);
    }
  }

  void KinectMovementFilter::depthImageCallback(
    const sensor_msgs::ImageConstPtr& msg)
  {
    if (fabs(pitchError_) < fabs(errorThreshold_) &&
      fabs(yawError_) < fabs(errorThreshold_))
    {
      depthImagePublisher_.publish(*msg);
    }
  }

  void KinectMovementFilter::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if (fabs(pitchError_) < fabs(errorThreshold_) &&
      fabs(yawError_) < fabs(errorThreshold_))
    {
      pointCloudPublisher_.publish(*msg);
    }
  }
}  // namespace pandora_common

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_movement_filter");
  pandora_common::KinectMovementFilter kinectMovementFilter;
  ros::spin();
}
