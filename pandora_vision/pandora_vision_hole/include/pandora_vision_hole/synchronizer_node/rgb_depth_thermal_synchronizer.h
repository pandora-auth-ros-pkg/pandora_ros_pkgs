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

#ifndef PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_THERMAL_SYNCHRONIZER_H
#define PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_THERMAL_SYNCHRONIZER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include "distrib_msgs/flirLeptonMsg.h"
#include "pandora_vision_msgs/SynchronizedMsg.h"
#include "utils/defines.h"
#include <pcl_conversions/pcl_conversions.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pcSubscriber;
  typedef message_filters::Subscriber<distrib_msgs::flirLeptonMsg> thermalSubscriber;
  typedef message_filters::sync_policies::ApproximateTime
    <sensor_msgs::PointCloud2, distrib_msgs::flirLeptonMsg> mySyncPolicy;

  /**
    @class RgbDTSynchronizer
    @brief Responsible for the synchronization of PointCloud2 messages from
    kinect and flirLeptonMsg from flir-Lepton camera.
   **/
  class RgbDTSynchronizer
  {
   public:
    /**
      @brief The constructor
      **/
    RgbDTSynchronizer(void);

    /**
      @brief The default constructor
      **/
    virtual ~RgbDTSynchronizer(void);

   private:
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
    void synchronizedCallback(
        const sensor_msgs::PointCloud2::ConstPtr& pcMsg,
        const distrib_msgs::flirLeptonMsg::ConstPtr& thermalMsg);

    /**
      @brief This callback is called when thermal mode is off. A custom synchronized
      message is sent to synchronizer node. This time we fill only the pointcloud.
      @param[in] pointCloudMsg [const sensor_msgs::PointCloud2::ConstPtr&]
      The input pointcloud2
      @return void
      **/
    void pointCloudCallback(
        const sensor_msgs::PointCloud2::ConstPtr& pointCloudMsg);

    /**
      @brief Acquires topics' names needed to be subscribed to and advertise
      to by the rgb_depth_thermal_synchronizer node
      @param void
      @return void
      **/
    void getTopicNames(void);


   private:
    // The ROS node handle
    ros::NodeHandle nodeHandle_;

    // The message_filters subscriber to kinect that aquires the PointCloud2
    // message
    pcSubscriber *inputPointCloudSubscriber_;

    // The name of the topic from where the PointCloud2 message is acquired
    std::string inputPointCloudTopic_;

    // The message_filters subscriber to flir-lepton camera that aquires
    // the flirLeptonMsg message
    thermalSubscriber *inputThermalCameraSubscriber_;

    message_filters::Synchronizer<mySyncPolicy> *synchronizer_;

    // The name of the topic from where the flirLeptonMsg message is acquired
    std::string inputThermalCameraTopic_;

    // Subscriber to kinect's pointcloud if thermal mode is off
    ros::Subscriber pointCloudSubscriber_;

    // The publisher which will advertise the synchronized message to
    // rgb_depth_synchronizer node
    ros::Publisher synchronizedMsgPublisher_;

    // The name of the topic to which the synchronized message will be
    // published
    std::string synchronizedMsgTopic_;

    // The queue size of the approximate time synch method
    int queue_;

    // The mode in which the package is running
    // If true Thermal process is enabled, else only Rgb-D.
    bool mode_;
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_SYNCHRONIZER_NODE_RGB_DEPTH_THERMAL_SYNCHRONIZER_H
