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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Angelos Triantafyllidis
 *********************************************************************/

#include <string>
#include <boost/algorithm/string.hpp>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>

#include "distrib_msgs/FlirLeptonMsg.h"

#include "hole_fusion_node/utils/defines.h"
#include "hole_fusion_node/utils/noise_elimination.h"
#include "hole_fusion_node/utils/message_conversions.h"
#include "synchronizer/pc_thermal_synchronizer.h"

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
#ifdef DEBUG_TIME
    invocationTime_ = 0.0;
    meanProcessingTime_ = 0.0;
    ticks_ = 0;
#endif
    // Take NodeHandlers from nodelet manager
    nh_ = this->getNodeHandle();
    private_nh_ = this->getPrivateNodeHandle();
    nodeName_ = boost::to_upper_copy<std::string>(this->getName());

    nh_.param("thermal_mode", thermalMode_, true);
    nh_.param("rgbd_mode", rgbdMode_, true);
    nh_.param("rgbdt_mode", rgbdtMode_, true);
    private_nh_.param("simulating", simulating_, false);

    // The synchronizer node starts off in life locked, waiting for the
    // hole fusion node to unlock him
    holeFusionLocked_ = true;

    // The synchronizer node starts off in life locked for thermal standalone
    // procedure , waiting for the thermal cropper node to unlock him.
    thermalLocked_ = true;

    // Acquire the names of topics which the synchronizer node will be having
    // transactionary affairs with
    getTopicNames();

    if (simulating_)
    {
      // Acquire the information about the input point cloud that cannot be
      // acquired from the point cloud message.
      // The parameters concerned are needed only if in simulation mode
      getSimulationInfo();
    }

/******************************************************************************
 *                                Synchronizer                                *
 ******************************************************************************/

    private_nh_.param("synchronizer_queue", queue_, 5);

    // If thermal mode is enabled in launch file message filters is on and
    // the two input messages are synchronized packed and sent.
    // If not only the pointcloud is sent for further usage
    if (rgbdtMode_ || thermalMode_)
    {
      syncPointCloudSubscriberPtr_.reset( new PcSubscriber(nh_,
            inputPointCloudTopic_, queue_) );
      syncThermalCameraSubscriberPtr_.reset( new ThermalSubscriber(nh_,
            syncThermalCameraTopic_, queue_) );

      synchronizerPtr_.reset( new ApprTimePcThermalSynchronizer(
            ApprTimePcThermalPolicy(queue_),
            *syncPointCloudSubscriberPtr_,
            *syncThermalCameraSubscriberPtr_) );
      synchronizerPtr_->registerCallback(
          boost::bind(&PcThermalSynchronizer::syncPointCloudThermalCallback, this, _1, _2));
    }
    else if (rgbdMode_)
    {
      inputPointCloudSubscriber_ = nh_.subscribe(inputPointCloudTopic_, 1,
          &PcThermalSynchronizer::inputPointCloudCallback, this);
    }

/******************************************************************************
 *                                Subscribers                                 *
 ******************************************************************************/

    if (rgbdMode_ || rgbdtMode_)
    {
      // Subscribe to the hole_fusion lock/unlock topic
      unlockHoleFusionSubscriber_ = nh_.subscribe(unlockHoleFusionTopic_, 1,
        &PcThermalSynchronizer::unlockHoleFusionCallback, this);
    }
    if (thermalMode_)
    {
      // Subscribe to the topic where the thermal node requests synchronizer
      // to act.
      unlockThermalSubscriber_ = nh_.subscribe(
        unlockThermalTopic_, 1,
        &PcThermalSynchronizer::unlockThermalCallback, this);
    }
    // Subscribe to the topic where the Hole Fusion node requests from the
    // synchronizer node to subscribe to the input point cloud topic
    subscribeToInputPointCloudSubscriber_ = nh_.subscribe(
      subscribeToInputPointCloudTopic_, 1,
      &PcThermalSynchronizer::subscribeToInputPointCloudCallback, this);

    // Subscribe to the topic where the Hole Fusion node requests from the
    // synchronizer node to leave its subscription to the
    // input point cloud topic
    leaveSubscriptionToInputPointCloudSubscriber_ = nh_.subscribe(
      leaveSubscriptionToInputPointCloudTopic_, 1,
      &PcThermalSynchronizer::leaveSubscriptionToInputPointCloudCallback, this);

/******************************************************************************
 *                                 Publishers                                 *
 ******************************************************************************/

    if (rgbdMode_ || rgbdtMode_)
    {
      // Advertise the synchronized point cloud
      synchronizedPointCloudPublisher_ = nh_.advertise
        <PointCloud>(synchronizedPointCloudTopic_, 1);
    }

    if (rgbdMode_ || rgbdtMode_)
    {
      // Advertise the synchronized depth image
      synchronizedDepthImagePublisher_ = nh_.advertise
        <sensor_msgs::Image>(synchronizedDepthImageTopic_, 1);

      // Advertise the synchronized rgb image
      synchronizedRgbImagePublisher_ = nh_.advertise
        <sensor_msgs::Image>(synchronizedRgbImageTopic_, 1);
    }

    if (rgbdtMode_ || thermalMode_)
    {
      // Advertise the synchronized thermal image and its index
      synchronizedThermalImagePublisher_ = nh_.advertise
        <distrib_msgs::FlirLeptonMsg>
        (synchronizedThermalImageTopic_, 1);

      thermalOutputReceiverPublisher_ = nh_.advertise
        <std_msgs::String>
        (thermalOutputReceiverTopic_, 1);
    }

    enhancedImagePublisher_ = nh_.advertise<pandora_vision_msgs::EnhancedImage>(
        enhancedImageTopic_, 1);

    if (thermalMode_)
    {
      enhancedImageCropperPublisher_ = nh_.advertise<pandora_vision_msgs::EnhancedImage>(
          enhancedImageCropperTopic_, 1);
    }

    std::string modes;
    if (rgbdMode_)
      modes += "rgbd ";
    if (rgbdtMode_)
      modes += "rgbdt ";
    if (thermalMode_)
      modes += "thermal";
    NODELET_INFO("[%s] Initiated %s", nodeName_.c_str(), modes.c_str());
  }

  void
  PcThermalSynchronizer::
  inputPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pcMsg)
  {
    sensor_msgs::ImagePtr rgbImageMessagePtr, depthImageMessagePtr;
    PointCloudPtr pointCloudPtr;
    initCallback(pointCloudPtr, rgbImageMessagePtr, depthImageMessagePtr, pcMsg);

    boost::shared_ptr<pandora_vision_msgs::EnhancedImage> enhancedImagePtr(
        new pandora_vision_msgs::EnhancedImage );
    enhancedImagePtr->header = pcMsg->header;
    enhancedImagePtr->rgbImage = *rgbImageMessagePtr;
    enhancedImagePtr->depthImage = *depthImageMessagePtr;
    enhancedImagePtr->isDepth = (hole_fusion::Parameters::Depth::interpolation_method == 0);

    enhancedImagePublisher_.publish(enhancedImagePtr);

    if (!holeFusionLocked_)
    {
      NODELET_INFO("[%s] RGBD Callback", nodeName_.c_str());
      holeFusionLocked_ = true;
      synchronizedPointCloudPublisher_.publish(pointCloudPtr);
      synchronizedRgbImagePublisher_.publish(rgbImageMessagePtr);
      synchronizedDepthImagePublisher_.publish(depthImageMessagePtr);

      // inputPointCloudSubscriber_.shutdown();
    }
  }

  /**
    @brief The synchronized callback for the point cloud
    obtained by the depth sensor and the thermal info by flir camera.

    If the synchronizer node is unlocked, it extracts a depth image from
    the input point cloud's depth measurements, an RGB image from the colour
    measurements of the input point cloud and thermal info.
    Then publishes these images
    to their respective recipients. Finally, the input point cloud is
    published directly to the hole fusion node.
    @param[in] synchronizedMessage [pandora_vision_msgs::SynchronizedMsg&]
    the input synchronized thermal and pc info
    @return void
   **/
  void
  PcThermalSynchronizer::
  syncPointCloudThermalCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcMsg,
      const distrib_msgs::FlirLeptonMsgConstPtr& thermalMsg)
  {
    sensor_msgs::ImagePtr rgbImageMessagePtr, depthImageMessagePtr;
    PointCloudPtr pointCloudPtr;
    initCallback(pointCloudPtr, rgbImageMessagePtr, depthImageMessagePtr, pcMsg);

    boost::shared_ptr<pandora_vision_msgs::EnhancedImage> enhancedImagePtr(
        new pandora_vision_msgs::EnhancedImage );
    enhancedImagePtr->header = pcMsg->header;
    enhancedImagePtr->rgbImage = *rgbImageMessagePtr;
    enhancedImagePtr->depthImage = *depthImageMessagePtr;
    enhancedImagePtr->isDepth = (hole_fusion::Parameters::Depth::interpolation_method == 0);

    enhancedImagePublisher_.publish(enhancedImagePtr);

    if (!thermalLocked_ || !holeFusionLocked_)
    {
      NODELET_INFO("[%s] RGBDT Callback", nodeName_.c_str());
      std_msgs::StringPtr thermalIndex( new std_msgs::String );

      if (thermalMode_ && !thermalLocked_)
      {
        thermalIndex->data = thermalIndex->data + "thermal";
      }

      if ((rgbdMode_ || rgbdtMode_) && !holeFusionLocked_)
      {
        if (rgbdtMode_)
          thermalIndex->data = thermalIndex->data + "hole";

        synchronizedPointCloudPublisher_.publish(pointCloudPtr);
        synchronizedRgbImagePublisher_.publish(rgbImageMessagePtr);
        synchronizedDepthImagePublisher_.publish(depthImageMessagePtr);
      }

      if ((thermalMode_ && !thermalLocked_) || (rgbdtMode_ && !holeFusionLocked_))
      {
        distrib_msgs::FlirLeptonMsg::Ptr thermalMsgPtr( new distrib_msgs::FlirLeptonMsg );
        *thermalMsgPtr = *thermalMsg;

        synchronizedThermalImagePublisher_.publish(thermalMsgPtr);
        thermalOutputReceiverPublisher_.publish(thermalIndex);
      }

      if (thermalMode_ && !thermalLocked_)
      {
        enhancedImageCropperPublisher_.publish(enhancedImagePtr);
      }

      if (!thermalLocked_)
        thermalLocked_ = true;
      if (!holeFusionLocked_)
        holeFusionLocked_ = true;

      // syncPointCloudSubscriberPtr_->unsubscribe();
      // syncThermalCameraSubscriberPtr_->unsubscribe();
    }
  }

  /**
    @brief The callback executed when the Hole Fusion node requests
    from the synchronizer node to leave its subscription to the
    input pointcloud2 and flir topics.
    This happens when the state of the hole detector package is set
    to "off" so as to minimize processing resources.
    @param[in] msg [const std_msgs::Empty&] An empty message used to
    trigger the callback
    @return void
   **/
  void
  PcThermalSynchronizer::
  leaveSubscriptionToInputPointCloudCallback(const std_msgs::EmptyConstPtr& msg)
  {
    if (rgbdtMode_ || thermalMode_)
    {
      // Shutdown the input thermal subscriber
      syncPointCloudSubscriberPtr_->unsubscribe();
      syncThermalCameraSubscriberPtr_->unsubscribe();
    }
    else if (rgbdMode_)
    {
      inputPointCloudSubscriber_.shutdown();
    }
  }

  /**
    @brief The callback executed when the Hole Fusion node requests
    from the synchronizer node to subscribe to the input pointcloud2 and flir.
    This happens when the hole detector is in an "off" state, where the
    synchronizer node is not subscribed to the input point cloud(and flir) and
    transitions to an "on" state, where the synchronizer node needs to be
    subscribed to the input point cloud topic in order for the hole detector
    to function.
    @param[in] msg [const std_msgs::Empty&] An empty message used to
    trigger the callback
    @return void
   **/
  void
  PcThermalSynchronizer::
  subscribeToInputPointCloudCallback(const std_msgs::EmptyConstPtr& msg)
  {
    if (rgbdtMode_ || thermalMode_)
    {
      syncPointCloudSubscriberPtr_->subscribe();
      syncThermalCameraSubscriberPtr_->subscribe();
      synchronizerPtr_->init();
    }
    else if (rgbdMode_)
    {
      inputPointCloudSubscriber_ = nh_.subscribe(inputPointCloudTopic_, 1,
          &PcThermalSynchronizer::inputPointCloudCallback, this);
    }
  }

  /**
    @brief The callback for the hole_fusion node request for the
    lock/unlock of the rgb_depth_synchronizer node.

    The synchronizer node, when subscribed to the input point cloud topic,
    receives in a second as many callbacks as the number of depth sensor's
    point cloud publications per second (currently 25). Because its
    function is to synchronize the input of the depth and rgb nodes,
    it needs to stay locked for the time that these nodes process their
    input. When both these nodes have finished processing their input
    images and the hole fusion node has processed the point cloud sent
    to him by the synchronizer node, the synchronizer
    (who has been locked all this time), can now be unlocked and therefore
    receive a new point cloud.
    @param[in] lockMsg [const std_msgs::Empty] An empty message used to
    trigger the callback
    @return void
   **/
  void
  PcThermalSynchronizer::
  unlockHoleFusionCallback(const std_msgs::EmptyConstPtr& lockMsg)
  {
    holeFusionLocked_ = false;
    // if (rgbdtMode_ || thermalMode_)
    // {
    //   syncPointCloudSubscriberPtr_->subscribe();
    //   syncThermalCameraSubscriberPtr_->subscribe();
    // }
    // else if (rgbdMode_)
    // {
    //   inputPointCloudSubscriber_ = nh_.subscribe(inputPointCloudTopic_, 1,
    //       &PcThermalSynchronizer::inputPointCloudCallback, this);
    // }
  }

  /**
    @brief The callback from thermal node. Set's a lock variable that is
    responsible for the message that is sent to thermal node from
    synchronizer node.
    @paramp[in] lockMsg [const std_msgs::Empty&] An empty message used to
    trigger the callback for thermal procedure.
   **/
  void
  PcThermalSynchronizer::
  unlockThermalCallback(const std_msgs::EmptyConstPtr& lockMsg)
  {
    thermalLocked_ = false;
    // if (rgbdtMode_ || thermalMode_)
    // {
    //   syncPointCloudSubscriberPtr_->subscribe();
    //   syncThermalCameraSubscriberPtr_->subscribe();
    // }
  }

  void
  PcThermalSynchronizer::
  initCallback(
      PointCloudPtr& pcPtr,
      sensor_msgs::ImagePtr& rgbImageMessagePtr,
      sensor_msgs::ImagePtr& depthImageMessagePtr,
      const sensor_msgs::PointCloud2ConstPtr& pcMsg)
  {
    // Exctract the pointcloud from the message and convert it
    // to PointCloud<T>::Ptr type.
    // pcl::PCLPointCloud2 pcl_pc;
    // pcl_conversions::toPCL(*pcMsg, pcl_pc);
    pcPtr = boost::make_shared<PointCloud>();
    pcl::fromROSMsg(*pcMsg, *pcPtr);

    if (simulating_)
    {
      // The input point cloud is unorganized, in other words,
      // simulation is running. Variables are needed to be set in order for
      // the point cloud to be functionally exploitable.
      if (pcPtr->height == 1)
      {
        // The point cloud's height
        pcPtr->height = hole_fusion::Parameters::Image::HEIGHT;

        // The point cloud's width
        pcPtr->width = hole_fusion::Parameters::Image::WIDTH;
      }
    }

    // Extract the RGB image from the point cloud

    // cv::Mat rgbImage = hole_fusion::MessageConversions::convertPointCloudMessageToImage(
    //   pcPtr, CV_8UC3);
    // cv_bridge::CvImagePtr rgbImageConverter( new cv_bridge::CvImage() );
    // rgbImageConverter->header = pcMsg->header;
    // rgbImageConverter->encoding = sensor_msgs::image_encodings::BGR8;
    // rgbImageConverter->image = rgbImage;
    // rgbImageMessagePtr = rgbImageConverter->toImageMsg();

    rgbImageMessagePtr = boost::make_shared<sensor_msgs::Image>();
    pcl::toROSMsg(*pcMsg, *rgbImageMessagePtr);
    rgbImageMessagePtr->header = pcMsg->header;

    // Extract the depth image from the point cloud
    cv::Mat depthImage = hole_fusion::MessageConversions::convertPointCloudMessageToImage(
        pcPtr, CV_32FC1);
    cv::Mat interpolatedDepthImage;
    hole_fusion::NoiseElimination::performNoiseElimination(depthImage, &interpolatedDepthImage);

#ifdef DEBUG_SHOW
    if (hole_fusion::Parameters::Debug::show_depth_image)
    {
      hole_fusion::Visualization::showScaled("Interpolated Depth image", interpolatedDepthImage, 1);
    }
#endif

    cv_bridge::CvImagePtr depthImageConverter(new cv_bridge::CvImage());
    depthImageConverter->header = pcMsg->header;
    depthImageConverter->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    depthImageConverter->image = interpolatedDepthImage;
    depthImageMessagePtr = depthImageConverter->toImageMsg();

    // depthImageMessagePtr = boost::make_shared<sensor_msgs::Image>();
    // MessageConversions::toROSDepthMsg(*pcMsg, *depthImageMessagePtr);
    // depthImageMessagePtr->header = pcMsg->header;
  }


  /**
    @brief Variables regarding the point cloud are needed to be set in
    simulation mode: the point cloud's heigth, width and point step.
    This method reads them and sets their respective Parameters.
    @param void
    @return void
   **/
  void
  PcThermalSynchronizer::
  getSimulationInfo()
  {
    // Read "height" from the nodehandle
    if (!private_nh_.getParam("height", hole_fusion::Parameters::Image::HEIGHT))
    {
      NODELET_FATAL(
        "[%s] Input dimension height failed to be read", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read "width" from the nodehandle
    if (!private_nh_.getParam("width", hole_fusion::Parameters::Image::WIDTH))
    {
      NODELET_FATAL(
        "[%s] Input dimension width failed to be read", nodeName_.c_str());
      ROS_BREAK();
    }
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the synchronizer node
    @param void
    @return void
   **/
  void
  PcThermalSynchronizer::
  getTopicNames()
  {
    // Read the name of the topic from where the rgb_depth_thermal_synchronizer
    // node acquires the input pointcloud2
    if (!private_nh_.getParam("subscribed_topics/pc_topic", inputPointCloudTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic kinect topic", nodeName_.c_str());
      ROS_BREAK();
    }

    if (rgbdMode_ || rgbdtMode_)
    {
      // Read the name of the topic that the Hole Fusion node uses to unlock
      // the synchronizer node
      if (!private_nh_.getParam("subscribed_topics/hole_fusion_unlock_topic",
            unlockHoleFusionTopic_))
      {
        NODELET_FATAL(
          "[%s] Could not find topic hole fusion unlock_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }
    if (rgbdtMode_ || thermalMode_)
    {
      // Read the name of the topic from where the rgb_depth_thermalsynchronizer
      // node acquires the input thermal message
      if (!private_nh_.getParam("subscribed_topics/thermal_topic", syncThermalCameraTopic_))
      {
        NODELET_FATAL("[%s] Could not find thermal camera topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }
    if (thermalMode_)
    {
      // not to thermal standalone node.
      if (!private_nh_.getParam("subscribed_topics/thermal_unlock_topic",
            unlockThermalTopic_))
      {
        NODELET_FATAL(
          "[%s] Could not find topic thermal unlock_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }

    // Read the name of the topic that the Hole Fusion node uses to request from
    // the synchronizer node to subscribe to the input point cloud
    if (!private_nh_.getParam("subscribed_topics/subscribe_to_input",
          subscribeToInputPointCloudTopic_))
    {
      NODELET_FATAL(
        "[%s] Could not find subscribe to input command topic", nodeName_.c_str());
      ROS_BREAK();
    }
    // Read the name of the topic that the Hole Fusion node uses to request from
    // the synchronizer node to leave its subscription to the input point cloud
    if (!private_nh_.getParam("subscribed_topics/leave_subscription_to_input",
          leaveSubscriptionToInputPointCloudTopic_))
    {
      NODELET_FATAL(
        "[%s] Could not find leave subcription to input command topic", nodeName_.c_str());
      ROS_BREAK();
    }

    if (rgbdMode_ || rgbdtMode_)
    {
      // Read the name of the topic that the synchronizer node will be publishing
      // the input point cloud to
      if (!private_nh_.getParam("published_topics/point_cloud_internal_topic",
            synchronizedPointCloudTopic_))
      {
        NODELET_FATAL(
            "[%s] Could not find topic point_cloud_internal_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }

    if (rgbdMode_ || rgbdtMode_)
    {
      // Read the name of the topic that the synchronizer node will be publishing
      // the depth image extracted from the input point cloud to
      if (!private_nh_.getParam("published_topics/depth_image_topic",
            synchronizedDepthImageTopic_))
      {
        NODELET_FATAL(
          "[%s] Could not find topic depth_image_topic", nodeName_.c_str());
        ROS_BREAK();
      }
      // Read the name of the topic that the synchronizer node will be publishing
      // the depth image extracted from the input point cloud to
      if (!private_nh_.getParam("published_topics/rgb_image_topic",
            synchronizedRgbImageTopic_))
      {
        NODELET_FATAL(
          "[%s] Could not find topic rgb_image_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }

    if (rgbdtMode_ || thermalMode_)
    {
      // Read the name of the topic that the synchronizer node will be publishing
      // the thermal info extracted from flir camera
      if (!private_nh_.getParam("published_topics/thermal_image_topic",
            synchronizedThermalImageTopic_))
      {
        NODELET_FATAL(
          "[%s] Could not find topic thermal_image_topic", nodeName_.c_str());
        ROS_BREAK();
      }
      // Read the name of the topic that the synchronizer node will be publishing
      // the rgb and depth image to thermal cropper node
      if (!private_nh_.getParam("published_topics/thermal_output_receiver_topic",
            thermalOutputReceiverTopic_))
      {
        NODELET_FATAL("[%s] Could not find topic thermal_output_receiver_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }

    if (!private_nh_.getParam("published_topics/enhanced_image_topic",
          enhancedImageTopic_))
    {
      NODELET_FATAL("[%s] Could not find topic enhanced_image_topic", nodeName_.c_str());
      ROS_BREAK();
    }

    if (thermalMode_)
    {
      if (!private_nh_.getParam("published_topics/enhanced_image_cropper_topic",
            enhancedImageCropperTopic_))
      {
        NODELET_FATAL("[%s] Could not find topic enhanced_image_cropper_topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
