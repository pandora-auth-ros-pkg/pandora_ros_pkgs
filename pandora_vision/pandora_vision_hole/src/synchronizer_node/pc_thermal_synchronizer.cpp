/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors:
 *   Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>
#include <boost/algorithm/string.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>

#include "distrib_msgs/flirLeptonMsg.h"
#include "pandora_vision_msgs/SynchronizedMsg.h"

#include "synchronizer_node/pc_thermal_synchronizer.h"

PLUGINLIB_EXPORT_CLASS(pandora_vision::pandora_vision_hole::PcThermalSynchronizer, nodelet::Nodelet)

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  /**
    @brief The constructor
   **/
  PcThermalSynchronizer::
  PcThermalSynchronizer() {}

  PcThermalSynchronizer::
  ~PcThermalSynchronizer()
  {
    NODELET_INFO("[%s] Terminated", nodeName_.c_str());
  }

  void
  PcThermalSynchronizer::
  onInit()
  {
    // Take NodeHandlers from nodelet manager
    nh_ = this->getNodeHandle();
    private_nh_ = this->getPrivateNodeHandle();
    nodeName_ = boost::to_upper_copy<std::string>(this->getName());

    getTopicNames();

    private_nh_.param("synchronizer_queue", queue_, 5);

    // If thermal mode is enabled in launch file message filters is on and
    // the two input messages are synchronized packed and sent.
    // If not only the pointcloud is sent for further usage
    inputPointCloudSubscriberPtr_.reset( new PcSubscriber(nh_,
          inputPointCloudTopic_, queue_) );
    inputThermalCameraSubscriberPtr_.reset( new ThermalSubscriber(nh_,
          inputThermalCameraTopic_, queue_) );

    synchronizerPtr_.reset( new ApprTimePcThermalSynchronizer(
          ApprTimePcThermalPolicy(queue_),
          *inputPointCloudSubscriberPtr_,
          *inputThermalCameraSubscriberPtr_) );
    synchronizerPtr_->registerCallback(
        boost::bind(&PcThermalSynchronizer::synchronizedCallback, this, _1, _2));

    synchronizedMsgPublisher_ = nh_.advertise<pandora_vision_msgs::SynchronizedMsg>
      (synchronizedMsgTopic_, 1);

    NODELET_INFO("[%s] Initiated", nodeName_.c_str());
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
  void PcThermalSynchronizer::synchronizedCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcMsg,
      const distrib_msgs::flirLeptonMsgConstPtr& thermalMsg)
  {
    NODELET_INFO("[%s] Pointcloud-Thermal synchronizer callback called", nodeName_.c_str());

    // The synchronized message that is sent to synchronizer node
    pandora_vision_msgs::SynchronizedMsgPtr msgPtr( new pandora_vision_msgs::SynchronizedMsg );

    msgPtr->pc = *pcMsg;
    msgPtr->header.stamp = ros::Time::now();  // mby needs to change, write frame_id too
    msgPtr->thermalImage = thermalMsg->thermalImage;
    msgPtr->temperatures = thermalMsg->temperatures;

    synchronizedMsgPublisher_.publish(msgPtr);
    // @note: Do not change the msgPtr after publishing!
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the rgb_depth_thermal_synchronizer node
    @param void
    @return void
   **/
  void PcThermalSynchronizer::getTopicNames(void)
  {
    // Read the name of the topic from where the rgb_depth_thermal_synchronizer
    // node acquires the input pointcloud2
    if (!private_nh_.getParam("subscribed_topics/pc_topic", inputPointCloudTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic kinect topic", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read the name of the topic from where the rgb_depth_thermalsynchronizer
    // node acquires the input thermal message
    if (!private_nh_.getParam("subscribed_topics/thermal_topic", inputThermalCameraTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal camera topic", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read the name of the topic that the rgb_depth_thermal synchronizer node
    // will be publishing the synchronized message to synchronizer node
    if (!private_nh_.getParam("published_topics/synchronized_topic", synchronizedMsgTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic for synchronizer node", nodeName_.c_str());
      ROS_BREAK();
    }
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
