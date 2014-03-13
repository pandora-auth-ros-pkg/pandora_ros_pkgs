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

#include "pandora_vision_kinect/kinect.h"

namespace vision
{

  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  PandoraKinect::PandoraKinect(void)
  {
    ros::Duration(0.5).sleep();

    _inputCloudSubscriber = _nodeHandle.subscribe("/camera/depth/points", 1,
        &PandoraKinect::inputCloudCallback, this);

    _inputImageSubscriber = _nodeHandle.subscribe("/kinect/image", 1,
        &PandoraKinect::inputImageCallback, this);

    //~ _inputDepthImageSubscriber  =
      //~ _nodeHandle.subscribe("/camera/depth/image",
        //~ 1, &PandoraKinect::inputDepthImageCallback, this);

    _planePublisher = _nodeHandle.advertise<PointCloudXYZ>
      ("/vision/kinect/planes", 1000);
  }



  /**
    @brief Default destructor
    @return void
   **/
  PandoraKinect::~PandoraKinect(void) {}



  /**
    @brief Callback for the RGB image
    @param msg [const sensor_msgs::ImageConstPtr&] The RGB image
    @return void
  **/
  void PandoraKinect::inputImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _kinectFrame= in_msg->image.clone();
    _kinectFrameTimestamp = msg->header.stamp;

    if (_kinectFrame.empty())
    {
      ROS_ERROR("[kinectNode] : No more Frames");
      return;
    }
  }



  /**
    @brief Callback for the depth image
    @param msg [const sensor_msgs::ImageConstPtr&] The depth image
    @return void
   **/
  void PandoraKinect::inputDepthImageCallback
    (const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr in_msg;

      try
      {
        in_msg = cv_bridge::toCvCopy
          (msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

      _kinectDepthFrame = in_msg->image.clone();
      _kinectDepthFrameTimestamp = msg->header.stamp;
      if (_kinectDepthFrame.empty())
      {
        ROS_ERROR("[kinectNode] : No more Frames");
        return;
      }

    //!< each pixel has one value of intensity in the range of 0-255.\
    127 for unknown depth.

    double min;
    double max;

    cv::minMaxIdx(_kinectDepthFrame, &min, &max);
    cv::Mat adjMap;
    cv::convertScaleAbs(_kinectDepthFrame, _kinectDepthFrame, 255 / max);
    cv::imshow("Out", _kinectDepthFrame);
    cv::waitKey(1);

    //std::cerr << adjMap << std::endl << std::endl;

    /*
    // each pixel has one value; the distance from the sensor to the point in
    // the depth scene. 0 for unknown distance
    cv::namedWindow("Depth image from kinect", cv::WINDOW_AUTOSIZE);
    cv::imshow("Depth image from kinect", _kinectDepthFrame);

    std::cerr << _kinectDepthFrame << std::endl << std::endl;
    */

  }



  /**
    @brief Callback for the point cloud
    @param msg [const sensor_msgs::PointCloud2ConstPtr&] The point cloud
    message
    @return void
   **/
  void PandoraKinect::inputCloudCallback
    (const sensor_msgs::PointCloud2ConstPtr& msg)
  {

    PointCloud pointCloud;
    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr\
     to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*msg, pointCloud);

    //!< convert the point cloud from pcl::PCLPointCloud2 to pcl::PointCLoud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudXYZ
      (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZ);


    const PointCloudXYZPtr originalPointCloud (new PointCloudXYZ());
    copyPointCloud(*pointCloudXYZ, *originalPointCloud);

    //!< zArray is a 2D float array each element of which is the input
    //!< pointcloud -extracted z.
    //!< It holds floats in meters.
    //!< This array will be used to produce an opencv image similar to
    //!< the depth image obtained from /camera/depth/image.

    float zArray [pointCloudXYZ->height][pointCloudXYZ->width];

    for (unsigned int row = 0; row < pointCloudXYZ->height; ++row)
    {
      for (unsigned int col = 0; col < pointCloudXYZ->width; ++col)
      {
        zArray[row][col] =
          pointCloudXYZ->points[col + pointCloudXYZ->width * row].z;

        //!< if element is nan make it a zero
        if (zArray[row][col] != zArray[row][col])
        {
          zArray[row][col] = 0.0;
        }
      }
    }

    //!< convert the array to an opencv image depthImage now holds \
    the depth image
    cv::Mat depthImage(pointCloudXYZ->height, pointCloudXYZ->width,
      CV_32FC1, zArray);

    //!< Finds possible holes
    HoleFilters::HolesConveyor holes = HoleDetector::findHoles(depthImage,
        pointCloudXYZ);

    return;
  }



  /**
    @brief Stores a ensemble of point clouds in pcd images
    @param in_cloud [const std::vector<PointCloudXYZPtr>] The point clouds
    @return void
   **/
  void PandoraKinect::storePointCloudVectorToImages
    (const std::vector<PointCloudXYZPtr> in_vector)
  {
      for (int i = 0; i < in_vector.size(); i++)
      {
        pcl::io::savePCDFileASCII
          (boost::to_string(i) + "_.pcd", *in_vector[i]);
      }
  }



  /**
  @brief Publishes the planes to /vision/kinect/planes topic
  @param cloudVector [const std::vector<PointCloudXYZPtr>] The point clouds\
  containing the planes
  @return void
  **/
  void PandoraKinect::publishPlanes
    (const std::vector<PointCloudXYZPtr> cloudVector)
  {

    PointCloudXYZPtr aggregatedPlanes (new PointCloudXYZ);

    for (unsigned int i = 0; i < cloudVector.size(); i++)
    {
      *aggregatedPlanes += *cloudVector[i];
    }

    aggregatedPlanes->header.frame_id = cloudVector[0]->header.frame_id;
    aggregatedPlanes->header.stamp = cloudVector[0]->header.stamp;
    _planePublisher.publish(aggregatedPlanes);
  }

}
