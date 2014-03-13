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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#ifndef KINECT_NODE_H
#define KINECT_NODE_H

#include "pandora_vision_kinect/hole_detector.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace vision
{
  class PandoraKinect
  {
    private:
      //!< The ROS node handle
      ros::NodeHandle     _nodeHandle;
      //!< The RGB image acquired from Kinect
      cv::Mat             _kinectFrame;
      //!< The Depth image acquired from Kinect
      cv::Mat             _kinectDepthFrame;
      //!< Timestamp of RGB image
      ros::Time           _kinectFrameTimestamp;
      //!< Timestamp of depth image
      ros::Time           _kinectDepthFrameTimestamp;
      //!< Subscriber of Kinect point cloud
      ros::Subscriber     _inputCloudSubscriber;
      //!< Subscriber of kinect RGB image
      ros::Subscriber     _inputImageSubscriber;
      //!< Subscriber of Kinect depth image
      ros::Subscriber     _inputDepthImageSubscriber;
      //!< ROS publisher for computed 2D planes
      ros::Publisher      _planePublisher;

      Parameters params;

      HoleDetector hd; //!< Just to try out the particle filter


      /**
        @brief Callback for the point cloud
        @param msg [const sensor_msgs::PointCloud2ConstPtr&] The point cloud
        message
        @return void
       **/
      void inputCloudCallback
        (const sensor_msgs::PointCloud2ConstPtr& msg);

      /**
        @brief Callback for the RGB image
        @param msg [const sensor_msgs::ImageConstPtr&] The RGB image
        @return void
       **/
      void inputImageCallback
        (const sensor_msgs::ImageConstPtr& msg);

      /**
        @brief Callback for the depth image
        @param msg [const sensor_msgs::ImageConstPtr&] The depth image
        @return void
       **/
      void inputDepthImageCallback
        (const sensor_msgs::ImageConstPtr& msg);

      /**
        @brief Stores a ensemble of point clouds in pcd images
        @param in_cloud [const std::vector<PointCloudXYZPtr>] The point clouds
        @return void
       **/
      void storePointCloudVectorToImages
        (const std::vector<PointCloudXYZPtr> in_cloud);

      /**
        @brief Publishes the planes to /vision/kinect/planes topic
        @param cloudVector [const std::vector<PointCloudXYZPtr>] The point
        clouds containing the planes
        @return void
       **/
      void publishPlanes
        (const std::vector<PointCloudXYZPtr> cloudVector);


    public:

      /**
        @brief Default constructor. Initiates communications, loads parameters.
        @return void
       **/
      PandoraKinect(void);

      /**
        @brief Default destructor
        @return void
       **/
      ~PandoraKinect(void);
  };
}

#endif
