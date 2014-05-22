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

    // Subscribe to the RGB point cloud topic
    inputPointCloudSubscriber_ = nodeHandle_.subscribe(inputPointCloudTopic_, 1,
      &RgbDepthSynchronizer::synchronizedCallback, this);

    // Subscribe to the hole_fusion lock/unlock topic
    unlockSubscriber_ = nodeHandle_.subscribe(unlockTopic_, 1,
      &RgbDepthSynchronizer::holeFusionCallback, this);


    // Advertise the synchronized point cloud
    synchronizedPointCloudPublisher_ = nodeHandle_.advertise
      <sensor_msgs::PointCloud2>(synchronizedPointCloudTopic_, 1000);

    // Advertise the synchronized depth image
    synchronizedDepthImagePublisher_ = nodeHandle_.advertise
      <sensor_msgs::Image>(synchronizedDepthImageTopic_, 1000);

    // Advertise the synchronized rgb image
    synchronizedRGBImagePublisher_ = nodeHandle_.advertise
      <sensor_msgs::Image>(synchronizedRgbImageTopic_, 1000);

    ROS_INFO("[Synchronizer node] Initiated");

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
    ROS_INFO("[Synchronizer node] Terminated");
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
      #ifdef DEBUG_SHOW
      ROS_INFO ("[Synchronizer Node] Subscribed to the input point cloud");
      #endif
    }
    else
    {
      ROS_ERROR ("[Synchronizer Node] Could not find topic input_topic");
    }

    // Read the name of the topic that the Hole Fusion node uses to unlock
    // the synchronizer node
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/subscribed_topics/unlock_topic",
        unlockTopic_))
    {
      // Make the topic's name absolute
      unlockTopic_ = ns + "/" + unlockTopic_;

      #ifdef DEBUG_SHOW
      ROS_INFO ("[Synchronizer Node] Subscribed to the unlock topic");
      #endif
    }
    else
    {
      ROS_ERROR ("[Synchronizer Node] Could not find topic unlock_topic");
    }

    // Read the name of the topic that the synchronizer node will be publishing
    // the input point cloud to
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/point_cloud_internal_topic",
        synchronizedPointCloudTopic_))
    {
      // Make the topic's name absolute
      synchronizedPointCloudTopic_ = ns + "/" + synchronizedPointCloudTopic_;

      #ifdef DEBUG_SHOW
      ROS_INFO ("[Synchronizer Node] "
        "Advertising to the internal point cloud topic");
      #endif
    }
    else
    {
      ROS_ERROR ("[Synchronizer Node] "
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

      #ifdef DEBUG_SHOW
      ROS_INFO ("[Synchronizer Node] Advertising to the internal depth image");
      #endif
    }
    else
    {
      ROS_ERROR ("[Synchronizer Node] Could not find topic depth_image_topic");
    }

    // Read the name of the topic that the synchronizer node will be publishing
    // the depth image extracted from the input point cloud to
    if (nodeHandle_.getParam(
        ns + "/synchronizer_node/published_topics/rgb_image_topic",
        synchronizedRgbImageTopic_))
    {
      // Make the topic's name absolute
      synchronizedRgbImageTopic_ = ns + "/" + synchronizedRgbImageTopic_;

      #ifdef DEBUG_SHOW
      ROS_INFO ("[Synchronizer Node] Advertising to the internal rgb image");
      #endif
    }
    else
    {
      ROS_ERROR ("[Synchronizer Node] Could not find topic rgb_image_topic");
    }
  }



  /**
    @brief The synchronized callback for the point cloud and rgb image
    obtained by the depth sensor.
    @param[in] pointCloudMessage [const sensor_msgs::PointCloud2ConstPtr&]
    The input point cloud
    @return void
   **/
  void RgbDepthSynchronizer::synchronizedCallback(
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage)
  {
    if (!isLocked_)
    {
      // Lock the rgb_depth_synchronizer node; aka prevent the execution
      // of this if-block without the explicit request of the hole_fusion node
      isLocked_ = true;

      #ifdef DEBUG_SHOW
      ROS_INFO("Synchronizer unlocked");
      #endif

      #ifdef DEBUG_TIME
      ROS_ERROR("================================================");

      double t = ros::Time::now().toSec() - invocationTime_;

      ROS_ERROR("Previous synchronizer invocation before %fs", t);

      // Increment the number of this node's invocations
      ticks_++;

      if (ticks_ > 1)
      {
        meanProcessingTime_ += t;
      }

      ROS_ERROR("Mean processing time :                  %fs",
        (meanProcessingTime_ / (ticks_ - 1)));

      ROS_ERROR("================================================");

      invocationTime_ = ros::Time::now().toSec();

      Timer::start("synchronizedCallback", "", true);
      #endif

      // For simulation purposes, the width and height parameters of the
      // point cloud must be set.
      sensor_msgs::PointCloud2 pointCloud(*pointCloudMessage);

      // The input point cloud is unorganized, in other words,
      // simulation is running. Variables are needed to be set in order for
      // the point cloud to be functionally exploitable.
      // See http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
      if (pointCloud.height == 1)
      {
        // The namespace dictated in the launch file
        std::string ns = nodeHandle_.getNamespace();

        // Read "height" from the nodehandle
        int height;
        if (nodeHandle_.getParam(ns + "/height", height))
        {
          pointCloud.height = height;
        }
        else
        {
          pointCloud.height = Parameters::Image::HEIGHT;
        }

        // Read "width" from the nodehandle
        int width;
        if (nodeHandle_.getParam(ns + "/width", width))
        {
          pointCloud.width = width;
        }
        else
        {
          pointCloud.width= Parameters::Image::WIDTH;
        }

        // Read "point_step" from the nodehandle
        int point_step;
        if (nodeHandle_.getParam(ns + "/point_step", point_step))
        {
          pointCloud.point_step = point_step;
        }
        else
        {
          pointCloud.point_step = Parameters::Image::POINT_STEP;
        }

        pointCloud.row_step = pointCloud.width * pointCloud.point_step;
      }

      // Take a pointer on the constructed point cloud
      const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg =
        boost::make_shared<sensor_msgs::PointCloud2>(pointCloud);


      // Extract the RGB image from the point cloud
      cv::Mat rgbImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloudMsg, CV_8UC3);

      // Convert the rgbImage to a ROS message
      cv_bridge::CvImagePtr rgbImageMessagePtr(new cv_bridge::CvImage());

      rgbImageMessagePtr->header = pointCloudMsg->header;
      rgbImageMessagePtr->encoding = sensor_msgs::image_encodings::BGR8;
      rgbImageMessagePtr->image = rgbImage;

      // Publish the synchronized rgb image
      synchronizedRGBImagePublisher_.publish(rgbImageMessagePtr->toImageMsg());


      // Extract the depth image from the point cloud
      cv::Mat depthImage = MessageConversions::convertPointCloudMessageToImage(
        pointCloudMsg, CV_32FC1);

      // Convert the depthImage to a ROS message
      cv_bridge::CvImagePtr depthImageMessagePtr(new cv_bridge::CvImage());

      depthImageMessagePtr->header = pointCloudMsg->header;
      depthImageMessagePtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depthImageMessagePtr->image = depthImage;

      // Publish the synchronized depth image
      synchronizedDepthImagePublisher_.publish(
        depthImageMessagePtr->toImageMsg());


      // Publish the synchronized point cloud
      synchronizedPointCloudPublisher_.publish(pointCloudMsg);

      #ifdef DEBUG_TIME
      Timer::tick("synchronizedCallback");
      Timer::printAllMeansTree();
      #endif
    }
  }



  /**
    @brief The callback for the hole_fusion node request for the
    lock/unlock of the rgb_depth_synchronizer node
    @param[in] lockMsg [const std_msgs::Empty] An empty message
    @return void
   **/
  void RgbDepthSynchronizer::holeFusionCallback(const std_msgs::Empty& lockMsg)
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
