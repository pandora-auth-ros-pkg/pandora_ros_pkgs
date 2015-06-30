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
 *   Alexandros Philotheou
 *   Manos Tsardoulias
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>

#include "pandora_vision_common/pandora_vision_utilities/pointcloud_to_image_converter.h"

namespace pandora_vision
{
  PointCloudToImageConverter::PointCloudToImageConverter()
  {
  }

  PointCloudToImageConverter::~PointCloudToImageConverter()
  {
  }

  cv::Mat PointCloudToImageConverter::convertPclToImage(const sensor_msgs::PointCloud2ConstPtr& pclPtr,
      int encoding)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg<pcl::PointXYZRGB>(*pclPtr, *pointCloud);

    // Prepare the output image
    cv::Mat image(pointCloud->height, pointCloud->width, encoding);

    // For the depth image
    if (encoding == CV_32FC1)
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row) {
        for (unsigned int col = 0; col < pointCloud->width; ++col) {
          image.at<float>(row, col) =
            pointCloud->points[col + pointCloud->width * row].z;

          // if element is nan make it a zero
          if (image.at<float>(row, col) != image.at<float>(row, col))
          {
            image.at<float>(row, col) = 0.0;
          }
        }
      }
    }
    else if (encoding == CV_8UC3)  // For the rgb image
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row) {
        for (unsigned int col = 0; col < pointCloud->width; ++col) {
          image.at<unsigned char>(row, 3 * col + 2) =
            pointCloud->points[col + pointCloud->width * row].r;
          image.at<unsigned char>(row, 3 * col + 1) =
            pointCloud->points[col + pointCloud->width * row].g;
          image.at<unsigned char>(row, 3 * col + 0) =
            pointCloud->points[col + pointCloud->width * row].b;
        }
      }
    }
    return image;
  }
}  // namespace pandora_vision
