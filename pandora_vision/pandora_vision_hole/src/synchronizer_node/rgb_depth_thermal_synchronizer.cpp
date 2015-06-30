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
 * Authors: Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include "synchronizer_node/rgb_depth_thermal_synchronizer.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief The constructor
   **/
  RgbDTSynchronizer::RgbDTSynchronizer(void)
  {
    getTopicNames();

    queue_ = 5;

    // If thermal mode is enabled in launch file message filters is on and
    // the two input messages are synchronized packed and sent.
    // If not only the pointcloud is sent for further usage
    if (mode_)
    {
      inputPointCloudSubscriber_ = new pcSubscriber(nodeHandle_,
        inputPointCloudTopic_, queue_);

      inputThermalCameraSubscriber_ = new thermalSubscriber(nodeHandle_,
        inputThermalCameraTopic_, queue_);

      synchronizer_ = new message_filters::Synchronizer<mySyncPolicy>
        (mySyncPolicy(queue_),
         *inputPointCloudSubscriber_,
         *inputThermalCameraSubscriber_);

      synchronizer_->registerCallback(
        boost::bind(&RgbDTSynchronizer::synchronizedCallback,
          this, _1, _2));
    }
    else
    {
      pointCloudSubscriber_ = nodeHandle_.subscribe(inputPointCloudTopic_, 1,
        &RgbDTSynchronizer::pointCloudCallback, this);
    }

    synchronizedMsgPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::SynchronizedMsg>
      (synchronizedMsgTopic_, 1);

    ROS_INFO("[RGBDT Synchronizer node] Initiated");
  }

  /**
    @brief Default destructor
    @return void
   **/
  RgbDTSynchronizer::~RgbDTSynchronizer(void)
  {
    ROS_INFO("[RGBDT Synchronizer node] Terminated");
  }

  /**
    @brief The synchronized callback called when the input messages from kinect
    and thermal camera are acquired. A custom synchronized message is filled and
    sent to synchronizer node for further usage.
    @param[in] pcMsg [const sensor_msgs::PointCloud2::ConstPtr&]
    The input pointcloud from kinect
    @param[in] thermalMsg[const distrib_msgs::flirLeptonMsg::ConstPtr&]
    The input thermal message
    @return void
   **/
  void RgbDTSynchronizer::synchronizedCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pcMsg,
    const distrib_msgs::flirLeptonMsg::ConstPtr& thermalMsg)
  {
    // ROS_INFO("[RGBDT Synchronizer node] Synchronized callback called");

    // The synchronized message that is sent to synchronizer node
    pandora_vision_msgs::SynchronizedMsg msg;

    msg.pc = *pcMsg;
    // The synchronized message header stamp
    msg.header.stamp = ros::Time::now();

    // The pointcloud2
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

    // pcl_conversions::toPCL(*pcMsg, *cloud);
    // pcl_conversions::fromPCL(*cloud, msg.pc);

    // The thermal Image
    msg.thermalInfo.header = thermalMsg->thermalImage.header;
    msg.thermalInfo = thermalMsg->thermalImage;

    msg.thermalInfo.encoding = "mono8";
    msg.thermalInfo.height = 60;
    msg.thermalInfo.width = 80;
    msg.thermalInfo.step = 80 * sizeof(uint8_t);

    // The temperatures multiarray
    msg.temperatures = thermalMsg->temperatures;

    synchronizedMsgPublisher_.publish(msg);
  }

  /**
    @brief This callback is called when thermal mode is off. A custom synchronized
    message is sent to synchronizer node. This time we fill only the pointcloud.
    @param[in] pointCloudMsg [const sensor_msgs::PointCloud2::ConstPtr&]
    The input pointcloud2
    @return void
   **/
  void RgbDTSynchronizer::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& pointCloudMsg)
  {
    // ROS_INFO("[RGBDT Synchronizer node] PointCloud callback called");

    // The synchronized message that is sent to synchronizer node
    pandora_vision_msgs::SynchronizedMsg msg;

    msg.pc = *pointCloudMsg;
    // The synchronized message header stamp
    msg.header.stamp = ros::Time::now();

    synchronizedMsgPublisher_.publish(msg);
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the rgb_depth_thermal_synchronizer node
    @param void
    @return void
   **/
  void RgbDTSynchronizer::getTopicNames(void)
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // This variable indicates the mode in which Hole-Package is running
    // If is set to true -> Rgb-D-T mode
    // Else Rgb-D mode
    if (nodeHandle_.getParam("/hole_detector/thermal", mode_))
    {
      ROS_INFO("[RGBDT Synchronizer Node] Packages Mode has been found");
    }
    else
    {
      ROS_ERROR("[RGBDT Synchronizer] Could not find Packages Mode");
    }

    // Read the name of the topic from where the rgb_depth_thermal_synchronizer
    // node acquires the input pointcloud2
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/input_topic",
        inputPointCloudTopic_))
    {
      ROS_INFO(
        "[RGBDT Synchronizer Node] Subscribed to the input kinect topic");
    }
    else
    {
      ROS_ERROR(
        "[RGBDT Synchronizer Node] Could not find topic kinect topic");
    }

    // Read the name of the topic from where the rgb_depth_thermalsynchronizer
    // node acquires the input thermal message
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/input_thermal_topic",
        inputThermalCameraTopic_))
    {
      ROS_INFO(
        "[RGBDT Synchronizer Node] Subscribed to the input thermal camera topic");
    }
    else
    {
      ROS_ERROR(
        "[RGBDT Synchronizer Node] Could not find thermal camera topic");
    }

    // Read the name of the topic that the rgb_depth_thermal synchronizer node
    // will be publishing the synchronized message to synchronizer node
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/synchronized_topic",
        synchronizedMsgTopic_))
    {
      // Make the topic's name absolute
      synchronizedMsgTopic_ = ns + "/" + synchronizedMsgTopic_;

      ROS_INFO(
        "[RGBDT Synchronizer Node] Advertising to synchronizer node");
    }
    else
    {
      ROS_ERROR(
        "[RGBDT Synchronizer Node] Could not find topic for synchronizer node");
    }
  }

}  // namespace pandora_vision
