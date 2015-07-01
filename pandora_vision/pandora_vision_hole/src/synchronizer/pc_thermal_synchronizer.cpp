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
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include "distrib_msgs/FlirLeptonMsg.h"

#include "utils/defines.h"
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

    // The synchronizer node starts off in life locked, waiting for the
    // hole fusion node to unlock him
    holeFusionLocked_ = true;

    // The synchronizer node starts off in life locked for thermal standalone
    // procedure , waiting for the thermal cropper node to unlock him.
    thermalLocked_ = true;

    // Acquire the names of topics which the synchronizer node will be having
    // transactionary affairs with
    getTopicNames();

    // Acquire the information about the input point cloud that cannot be
    // acquired from the point cloud message.
    // The parameters concerned are needed only if in simulation mode
    getSimulationInfo();

/******************************************************************************
 *                                Synchronizer                                *
 ******************************************************************************/

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
        boost::bind(&PcThermalSynchronizer::inputPointCloudThermalCallback, this, _1, _2));

/******************************************************************************
 *                                Subscribers                                 *
 ******************************************************************************/

    // Subscribe to the hole_fusion lock/unlock topic
    unlockHoleFusionSubscriber_ = nh_.subscribe(unlockHoleFusionTopic_, 1,
      &PcThermalSynchronizer::unlockHoleFusionCallback, this);

    // Subscribe to the topic where the thermal node requests synchronizer
    // to act.
    unlockThermalSubscriber_ = nh_.subscribe(
      unlockThermalTopic_, 1,
      &PcThermalSynchronizer::unlockThermalCallback, this);

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

    // Advertise the synchronized point cloud
    synchronizedPointCloudPublisher_ = nh_.advertise
      <PointCloud>(synchronizedPointCloudTopic_, 1);

    // Advertise the synchronized depth image
    synchronizedDepthImagePublisher_ = nh_.advertise
      <sensor_msgs::Image>(synchronizedDepthImageTopic_, 1);

    // Advertise the synchronized rgb image
    synchronizedRgbImagePublisher_ = nh_.advertise
      <sensor_msgs::Image>(synchronizedRgbImageTopic_, 1);

    // Advertise the synchronized thermal image and its index
    synchronizedThermalImagePublisher_ = nh_.advertise
      <distrib_msgs::FlirLeptonMsg>
      (synchronizedThermalImageTopic_, 1);

    thermalOutputReceiverPublisher_ = nh_.advertise
      <std_msgs::String>
      (thermalOutputReceiverTopic_, 1);

    NODELET_INFO("[%s] Initiated", nodeName_.c_str());
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
  inputPointCloudThermalCallback(
      const sensor_msgs::PointCloud2ConstPtr& pcMsg,
      const distrib_msgs::FlirLeptonMsgConstPtr& thermalMsg)
  {
    // This block is responsible to sent the thermal image to thermal node.
    // This way thermal node runs autonomously but it also provides its
    // information to hole fusion node.
    if (!thermalLocked_ || !holeFusionLocked_)
    {
      //  This variable informs thermal node if it must publish to hole fusion
      //  to thermal cropper node or both.
      //  If it is set as "thermal" --> post to thermal cropper node.
      //  If it is set as "hole" --> post to hole fusion node.
      //  If it is set as "thermalhole" --> post to both.
      std_msgs::StringPtr thermalIndex( new std_msgs::String );

      // Exctract the pointcloud from the message and convert it
      // to PointCloud<T>::Ptr type.
      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*pcMsg, pcl_pc);
      PointCloudPtr pointCloud, copiedPc;
      copiedPc.reset( new PointCloud );
      pcl::fromPCLPointCloud2(pcl_pc, *copiedPc);
      pointCloud = copiedPc;

      if (simulating_)
      {
        // For simulation purposes, the width and height parameters of the
        // point cloud must be set. Copy the input point cloud message to another
        // one so that these can be set manually if and when needed
        pointCloud.reset( new PointCloud );
        pcl::copyPointCloud(*copiedPc, *pointCloud);

        // The input point cloud is unorganized, in other words,
        // simulation is running. Variables are needed to be set in order for
        // the point cloud to be functionally exploitable.
        if (pointCloud->height == 1)
        {
          // The point cloud's height
          pointCloud->height = Parameters::Image::HEIGHT;

          // The point cloud's width
          pointCloud->width = Parameters::Image::WIDTH;
        }
      }

      // Extract the RGB image from the point cloud
      cv::Mat rgbImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloud, CV_8UC3);
      // Convert the rgbImage to a ROS message
      cv_bridge::CvImagePtr rgbImageMessagePtr( new cv_bridge::CvImage() );
      rgbImageMessagePtr->encoding = sensor_msgs::image_encodings::BGR8;
      rgbImageMessagePtr->image = rgbImage;

      // Extract the depth image from the point cloud
      cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloud, CV_32FC1);
      // Convert the depthImage to a ROS message
      cv_bridge::CvImagePtr depthImageMessagePtr(new cv_bridge::CvImage());
      depthImageMessagePtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depthImageMessagePtr->image = depthImage;

      // This condition is crucial for the synchronization of thermal, rgb and
      // depth images to be sent for further processing. This block is
      // is responsible for the autonomous thermal procedure and sends
      // an enhanced message to thermal cropper node.
      if (!thermalLocked_)
      {
        // Lock the thermal procedure; aka prevent
        // the execution of this if-block without the explicit request
        // of the thermal cropper node.This way we can synchronize the rgb depth
        // and thermal images sent to thermal cropper.
        thermalLocked_ = true;
        // Change the thermalIndex_ variable to inform thermal node
        thermalIndex->data = thermalIndex->data + "thermal";
      }

      // This condition is crucial for the synchronization of thermal, rgb and
      // depth images to be sent for further processing. This block is
      // is responsible for hole exctraction procedure and sends rgb and depth
      // images to their respective nodes.
      if (!holeFusionLocked_)
      {
        // Lock the rgb_depth_thermal_synchronizer node; aka prevent
        // the execution of this if-block without the explicit request
        // of the hole fusion node.
        holeFusionLocked_ = true;

#ifdef DEBUG_TIME
        ROS_INFO_NAMED(PKG_NAME, "Synchronizer unlocked");

        ROS_INFO_NAMED(PKG_NAME,
          "=================================================");

        double t = ros::Time::now().toSec() - invocationTime_;

        ROS_INFO_NAMED(PKG_NAME,
          "Previous synchronizer invocation before %fs", t);

        // Increment the number of this node's invocations
        ticks_++;

        if (ticks_ > 1)
        {
          meanProcessingTime_ += t;
        }

        ROS_INFO_NAMED(PKG_NAME,
          "Mean processing time :                  %fs",
          (meanProcessingTime_ / (ticks_ - 1)));

        ROS_INFO_NAMED(PKG_NAME,
          "=================================================");

        invocationTime_ = ros::Time::now().toSec();

        Timer::start("synchronizedCallback", "", true);
#endif

        // Publish the synchronized rgb image
        synchronizedRgbImagePublisher_.publish(rgbImageMessagePtr->toImageMsg());

        // Publish the synchronized depth image
        synchronizedDepthImagePublisher_.publish(depthImageMessagePtr->toImageMsg());

        // Publish the synchronized point cloud
        synchronizedPointCloudPublisher_.publish(pointCloud);

        // Change the thermalIndex_ variable to inform thermal node
        thermalIndex->data = thermalIndex->data + "hole";

#ifdef DEBUG_TIME
        Timer::tick("synchronizedCallback");
        Timer::printAllMeansTree();
#endif
      }

      // Publish the synchronized thermal message to thermal node
      synchronizedThermalImagePublisher_.publish(thermalMsg);
      thermalOutputReceiverPublisher_.publish(thermalIndex);

      inputPointCloudSubscriberPtr_->unsubscribe();
      inputThermalCameraSubscriberPtr_->unsubscribe();
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
    // Shutdown the input thermal subscriber
    inputPointCloudSubscriberPtr_->unsubscribe();
    inputThermalCameraSubscriberPtr_->unsubscribe();
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
    inputPointCloudSubscriberPtr_->subscribe();
    inputThermalCameraSubscriberPtr_->subscribe();
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
    inputPointCloudSubscriberPtr_->subscribe();
    inputThermalCameraSubscriberPtr_->subscribe();
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
    inputPointCloudSubscriberPtr_->subscribe();
    inputThermalCameraSubscriberPtr_->subscribe();
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
    // Read "simulating" from the nodehandle
    if (!private_nh_.getParam("simulating", simulating_))
    {
      NODELET_FATAL(
        "[%s] Simulating mode failed to be read", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read "height" from the nodehandle
    if (!private_nh_.getParam("height", Parameters::Image::HEIGHT))
    {
      NODELET_FATAL(
        "[%s] Input dimension height failed to be read", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read "width" from the nodehandle
    if (!private_nh_.getParam("width", Parameters::Image::WIDTH))
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

    // Read the name of the topic from where the rgb_depth_thermalsynchronizer
    // node acquires the input thermal message
    if (!private_nh_.getParam("subscribed_topics/thermal_topic", inputThermalCameraTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal camera topic", nodeName_.c_str());
      ROS_BREAK();
    }

    // This variable indicates the mode in which Hole-Package is running
    // If is set to true -> Rgb-D-T mode
    // Else Rgb-D mode
    if (!nh_.getParam("thermal", thermalMode_))
    {
      NODELET_FATAL("[%s] Could not find Packages Mode", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read the name of the topic that the Hole Fusion node uses to unlock
    // the synchronizer node
    if (!private_nh_.getParam("subscribed_topics/hole_fusion_unlock_topic",
          unlockHoleFusionTopic_))
    {
      NODELET_FATAL(
        "[%s] Could not find topic hole fusion unlock_topic", nodeName_.c_str());
      ROS_BREAK();
    }

    // Read the name of the topic that the synchronizer node uses to publish or
    // not to thermal standalone node.
    if (!private_nh_.getParam("subscribed_topics/thermal_unlock_synchronizer_topic",
          unlockThermalTopic_))
    {
      NODELET_FATAL(
        "[%s] Could not find topic thermal unlock_topic", nodeName_.c_str());
      ROS_BREAK();
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

    // Read the name of the topic that the synchronizer node will be publishing
    // the input point cloud to
    if (!private_nh_.getParam("published_topics/point_cloud_internal_topic",
          synchronizedPointCloudTopic_))
    {
      NODELET_FATAL(
          "[%s] Could not find topic point_cloud_internal_topic", nodeName_.c_str());
      ROS_BREAK();
    }

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

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
