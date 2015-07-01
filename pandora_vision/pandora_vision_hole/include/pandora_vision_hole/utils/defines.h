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

#ifndef PANDORA_VISION_HOLE_UTILS_DEFINES_H
#define PANDORA_VISION_HOLE_UTILS_DEFINES_H

#include <iostream>
#include <fstream>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include "utils/timer.h"

#define DEBUG_SHOW
#define DEBUG_TIME

//  Transforms a float number to string
#define TOSTR(x)      static_cast< std::ostringstream & >( \
  (std::ostringstream() << std::dec << x)).str()

//  Takes the file name from an absolute path
#define LPATH(x)      x.substr(x.find_last_of("/") + 1 , \
  x.size() - x.find_last_of("/") - 1)

//  Shortcut to std::string
#define STR(x)        std::string(x)

//  Available processing modes:
//  In RGBD_MODE, depth-based filters can be utilized along side RGB-based
//  filters in order to ascerain the validity of candidate holes.
//  In RGB_ONLY_MODE, depth analysis is not possible due to the overwhelming
//  amount of noise present in the input depth image, and so the validity
//  of candidate holes can only be tested through RGB-based filters.
#define RGBD_MODE 0
#define RGB_ONLY_MODE 1


//  The string identifier of the hole detector package.
//  It must comply with the sub-namespace set in the package's launchers
#define PKG_NAME "hole_detector"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{

  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
  typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudXYZConstPtr;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudConstPtr;

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_UTILS_DEFINES_H
