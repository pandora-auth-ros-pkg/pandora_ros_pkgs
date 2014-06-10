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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "synchronizer_node/rgb_depth_synchronizer.h"

namespace pandora_vision
{
  /**
    @brief The constructor
   **/
  RgbDepthSynchronizer::RgbDepthSynchronizer(void)
    : invocationTime_(0.0), meanProcessingTime_(0.0), ticks_(0)
  {
    #ifdef DEBUG_TIME
    Timer::start("RgbDepthSynchronizer");
    #endif

    isLocked_ = true;

    // Acquire the names of topics which the synchronizer node will be having
    // transactionary affairs with
    getTopicNames();

    // Acquire the information about the input point cloud that cannot be
    // acquired from the point cloud message.
    // The parameters concerned are needed only if in simulation mode
    getSimulationDimensions();

    // Subscribe to the RGB point cloud topic
    inputPointCloudSubscriber_ = nodeHandle_.subscribe(inputPointCloudTopic_, 1,
      &RgbDepthSynchronizer::inputPointCloudCallback, this);

    // Subscribe to the hole_fusion lock/unlock topic
    unlockSubscriber_ = nodeHandle_.subscribe(unlockTopic_, 1,
      &RgbDepthSynchronizer::unlockCallback, this);

    // Subscribe to the topic where the Hole Fusion node requests from the
    // synchronizer node to subscribe to the input point cloud topic
    subscribeToInputPointCloudSubscriber_ = nodeHandle_.subscribe(
      subscribeToInputPointCloudTopic_, 1,
      &RgbDepthSynchronizer::subscribeToInputPointCloudCallback, this);

    // Subscribe to the topic where the Hole Fusion node requests from the
    // synchronizer node to leave its subscription to the
    // input point cloud topic
    leaveSubscriptionToInputPointCloudSubscriber_ = nodeHandle_.subscribe(
      leaveSubscriptionToInputPointCloudTopic_, 1,
      &RgbDepthSynchronizer::leaveSubscriptionToInputPointCloudCallback, this);


    // Advertise the synchronized point cloud
    synchronizedPointCloudPublisher_ = nodeHandle_.advertise
      <PointCloud>(synchronizedPointCloudTopic_, 1000);

    // Advertise the synchronized depth image
    synchronizedDepthImagePublisher_ = nodeHandle_.advertise
      <sensor_msgs::Image>(synchronizedDepthImageTopic_, 1000);

    // Advertise the synchronized rgb image
    synchronizedRGBImagePublisher_ = nodeHandle_.advertise
      <sensor_msgs::Image>(synchronizedRgbImageTopic_, 1000);

    ROS_INFO_NAMED("hole_detector", "[Synchronizer node] Initiated");

    #ifdef DEBUG_TIME
    Timer::tick("RgbDepthSynchronizer");
    #endif
  }



  /**
    @brief Default destructor
    @return void
   **/
  RgbDepthSynchronizer::~RgbDepthSynchronizer(void)
  {
    ROS_INFO_NAMED("hole_detector", "[Synchronizer node] Terminated");
  }



  /**
    @brief Variables regarding the point cloud are needed to be set in
    simulation mode: the point cloud's heigth, width and point step.
    This method reads them and sets their respective Parameters.
    @param void
    @return void
   **/
  void RgbDepthSynchronizer::getSimulationDimensions()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read "height" from the nodehandle
    if (nodeHandle_.hasParam(ns + "/synchronizer_node/height"))
    {
      if (nodeHandle_.getParam(ns + "/synchronizer_node/height",
          Parameters::Image::HEIGHT))
      {
        ROS_INFO_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension height read");
      }
      else
      {
        ROS_ERROR_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension height failed to be read");
      }
    }

    // Read "width" from the nodehandle
    if (nodeHandle_.hasParam(ns + "/synchronizer_node/width"))
    {
      if (nodeHandle_.getParam(ns + "/synchronizer_node/width",
          Parameters::Image::WIDTH))
      {
        ROS_INFO_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension width read");
      }
      else
      {
        ROS_ERROR_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension width failed to be read");
      }
    }

    // Read "point_step" from the nodehandle
    if (nodeHandle_.hasParam(ns + "/synchronizer_node/point_step"))
    {
      if (nodeHandle_.getParam(ns + "/synchronizer_node/point_step",
          Parameters::Image::POINT_STEP))
      {
        ROS_INFO_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension point_step read");
      }
      else
      {
        ROS_ERROR_NAMED("hole_detector",
          "[Synchronizer Node] Input dimension point_step failed to be read");
      }
    }
  }



  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the synchronizer node
    @param void
    @return void
   **/
  void RgbDepthSynchronizer::getTopicNames ()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the synchronizer node acquires the
    // input point cloud
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/input_topic",
        inputPointCloudTopic_ ))
    {
      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Subscribed to the input point cloud");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic input_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to unlock
    // the synchronizer node
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/unlock_topic",
        unlockTopic_))
    {
      // Make the topic's name absolute
      unlockTopic_ = ns + "/" + unlockTopic_;

      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Subscribed to the unlock topic");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic unlock_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to request from
    // the synchronizer node to subscribe to the input point cloud
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/subscribe_to_input",
        subscribeToInputPointCloudTopic_))
    {
      // Make the topic's name absolute
      subscribeToInputPointCloudTopic_ =
        ns + "/" + subscribeToInputPointCloudTopic_;

      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Subscribed to the Hole Fusion input cloud"
        "subscription request topic");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic ");
    }

    // Read the name of the topic that the Hole Fusion node uses to request from
    // the synchronizer node to leave its subscription to the input point cloud
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/leave_subscription_to_input",
        leaveSubscriptionToInputPointCloudTopic_))
    {
      // Make the topic's name absolute
      leaveSubscriptionToInputPointCloudTopic_ =
        ns + "/" + leaveSubscriptionToInputPointCloudTopic_;

      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Subscribed to the Hole Fusion input cloud"
        "subscription leave request topic");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic ");
    }

    // Read the name of the topic that the synchronizer node will be publishing
    // the input point cloud to
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/point_cloud_internal_topic",
        synchronizedPointCloudTopic_))
    {
      // Make the topic's name absolute
      synchronizedPointCloudTopic_ = ns + "/" + synchronizedPointCloudTopic_;

      ROS_INFO_NAMED("hole_detector", "[Synchronizer Node] "
        "Advertising to the internal point cloud topic");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector", "[Synchronizer Node] "
        "Could not find topic point_cloud_internal_topic");
    }

    // Read the name of the topic that the synchronizer node will be publishing
    // the depth image extracted from the input point cloud to
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/depth_image_topic",
        synchronizedDepthImageTopic_))
    {
      // Make the topic's name absolute
      synchronizedDepthImageTopic_ = ns + "/" + synchronizedDepthImageTopic_;

      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Advertising to the internal depth image");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic depth_image_topic");
    }

    // Read the name of the topic that the synchronizer node will be publishing
    // the depth image extracted from the input point cloud to
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/rgb_image_topic",
        synchronizedRgbImageTopic_))
    {
      // Make the topic's name absolute
      synchronizedRgbImageTopic_ = ns + "/" + synchronizedRgbImageTopic_;

      ROS_INFO_NAMED("hole_detector",
        "[Synchronizer Node] Advertising to the internal rgb image");
    }
    else
    {
      ROS_ERROR_NAMED("hole_detector",
        "[Synchronizer Node] Could not find topic rgb_image_topic");
    }
  }



  /**
    @brief The synchronized callback for the point cloud
    obtained by the depth sensor.
    @param[in] pointCloudMessage [const PointCloudPtr&]
    The input point cloud
    @return void
   **/
  void RgbDepthSynchronizer::inputPointCloudCallback(
    const PointCloudPtr& pointCloudMessage)
  {
    if (!isLocked_)
    {
      // Lock the rgb_depth_synchronizer node; aka prevent the execution
      // of this if-block without the explicit request of the hole fusion node
      isLocked_ = true;

      #ifdef DEBUG_TIME
      ROS_INFO_NAMED("hole_detector", "Synchronizer unlocked");

      ROS_INFO_NAMED("hole_detector",
        "=================================================");

      double t = ros::Time::now().toSec() - invocationTime_;

      ROS_INFO_NAMED("hole_detector",
        "Previous synchronizer invocation before %fs", t);

      // Increment the number of this node's invocations
      ticks_++;

      if (ticks_ > 1)
      {
        meanProcessingTime_ += t;
      }

      ROS_INFO_NAMED("hole_detector",
        "Mean processing time :                  %fs",
        (meanProcessingTime_ / (ticks_ - 1)));

      ROS_INFO_NAMED("hole_detector",
        "=================================================");

      invocationTime_ = ros::Time::now().toSec();

      Timer::start("synchronizedCallback", "", true);
      #endif

      // For simulation purposes, the width and height parameters of the
      // point cloud must be set. Copy the input point cloud message to another
      // one so that these can be set manually if and when needed
      PointCloudPtr pointCloud(new PointCloud);
      pcl::copyPointCloud(*pointCloudMessage, *pointCloud);

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


      // Extract the RGB image from the point cloud
      cv::Mat rgbImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloud, CV_8UC3);

      // Convert the rgbImage to a ROS message
      cv_bridge::CvImagePtr rgbImageMessagePtr(new cv_bridge::CvImage());

      rgbImageMessagePtr->encoding = sensor_msgs::image_encodings::BGR8;
      rgbImageMessagePtr->image = rgbImage;

      // Publish the synchronized rgb image
      synchronizedRGBImagePublisher_.publish(rgbImageMessagePtr->toImageMsg());


      // Extract the depth image from the point cloud
      cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloud, CV_32FC1);

      // Convert the depthImage to a ROS message
      cv_bridge::CvImagePtr depthImageMessagePtr(new cv_bridge::CvImage());

      depthImageMessagePtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depthImageMessagePtr->image = depthImage;

      // Publish the synchronized depth image
      synchronizedDepthImagePublisher_.publish(
        depthImageMessagePtr->toImageMsg());


      // Publish the synchronized point cloud
      synchronizedPointCloudPublisher_.publish(pointCloud);

      #ifdef DEBUG_TIME
      Timer::tick("synchronizedCallback");
      Timer::printAllMeansTree();
      #endif
    }
  }



  /**
    @brief The callback executed when the Hole Fusion node requests
    from the synchronizer node to leave its subscription to the
    input point cloud
    @param[in] msg [const std_msgs::Empty&] An empty message used to
    trigger the callback
    @return void
   **/
  void RgbDepthSynchronizer::leaveSubscriptionToInputPointCloudCallback(
    const std_msgs::Empty& msg)
  {
    // Shutdown the input point cloud subscriber
    inputPointCloudSubscriber_.shutdown();
  }



  /**
    @brief The callback executed when the Hole Fusion node requests
    from the synchronizer node to subscribe to the input point cloud
    @param[in] msg [const std_msgs::Empty&] An empty message used to
    trigger the callback
    @return void
   **/
  void RgbDepthSynchronizer::subscribeToInputPointCloudCallback(
    const std_msgs::Empty& msg)
  {
    // Subscribe to the input point cloud topic
    inputPointCloudSubscriber_ = nodeHandle_.subscribe(inputPointCloudTopic_, 1,
      &RgbDepthSynchronizer::inputPointCloudCallback, this);
  }



  /**
    @brief The callback for the hole_fusion node request for the
    lock/unlock of the rgb_depth_synchronizer node
    @param[in] lockMsg [const std_msgs::Empty] An empty message
    @return void
   **/
  void RgbDepthSynchronizer::unlockCallback(const std_msgs::Empty& lockMsg)
  {
    #ifdef DEBUG_TIME
    Timer::start("holeFusionCallback");
    #endif

    isLocked_ = false;

    #ifdef DEBUG_TIME
    Timer::tick("holeFusionCallback");
    #endif
  }

} // namespace pandora_vision
