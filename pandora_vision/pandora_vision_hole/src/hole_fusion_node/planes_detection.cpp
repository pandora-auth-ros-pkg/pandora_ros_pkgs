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

#include "hole_fusion_node/planes_detection.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Applies a voxel grid filtering
    (http://pointclouds.org/documentation/tutorials/voxel_grid.php)
    @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud to filter
    @return PointCloudXYZPtr A pointer to the filtered cloud
   **/
  PointCloudXYZPtr PlanesDetection::applyVoxelGridFilter(
    const PointCloudXYZPtr& cloudIn)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyVoxelGridFilter", "locatePlanes");
    #endif

    // The output filtered cloud
    PointCloudXYZPtr cloudOut (new PointCloudXYZ());

    pcl::VoxelGrid <pcl::PointXYZ> sor;
    sor.setInputCloud(cloudIn);

    float leafSize = Parameters::HoleFusion::Planes::filter_leaf_size;
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*cloudOut);

    #ifdef DEBUG_TIME
    Timer::tick("applyVoxelGridFilter");
    #endif

    return cloudOut;
  }



  /**
    @brief Identify the planes in a point cloud and return the number of
    detected planes.
    @param[in] inputCloud [const PointCloudXYZPtr&] The point cloud whose
    planes we wish to locate
    @param[in] applyVoxelFilter [const bool&] Apply the voxel filter or not
    on the input cloud.
    @param[out] inliersVector [std::vector<pcl::PointIndices::Ptr>*]
    A vector of pointers to a pcl::PointIndices struct where point indices
    are stored. In pandora's context, it is used to calculate the maximum
    ratio of points lying on a plane
    @return The number of planes detected in inputCloud.
   **/
  int PlanesDetection::locatePlanes(const PointCloudXYZPtr& inputCloud,
    const bool& applyVoxelFilter,
    std::vector<pcl::PointIndices::Ptr>* inliersVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("locatePlanes", "checkHolesRectangleOutline");
    #endif

    PointCloudXYZPtr inCloud (new PointCloudXYZ);

    // Apply voxel filtering
    if (applyVoxelFilter)
    {
      inCloud = applyVoxelGridFilter(inputCloud);
    }
    else
    {
      copyPointCloud(*inputCloud, *inCloud);
    }

    // The vector of planar point clouds
    std::vector<PointCloudXYZPtr> planesVectorOut;

    // The coefficients of the planes
    std::vector<pcl::ModelCoefficients> coefficientsVectorOut;

    // Locate planes
    locatePlanesUsingSACSegmentation(inCloud,
      &planesVectorOut, &coefficientsVectorOut, inliersVector);

    #ifdef DEBUG_TIME
    Timer::tick("locatePlanes");
    #endif

    return planesVectorOut.size();
  }



  /**
    @brief Locates planes using the SAC segmentation
    (as stated in http://www.pointclouds.org/documentation/tutorials/
    extract_indices.php#extract-indices)
    @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose planes we
    wish to locate
    @param[out] planesVector [std::vector<PointCloudXYZPtr>*]
    The output vector of pointers to point cloud planes
    @param[out] coefficientsVector[std::vector<pcl::ModelCoefficients>*]
    The output vector of coefficients of each plane detected
    @param[out] inliersVector[std::vector<pcl::PointIndices::Ptr>*]
    The inliers for each plane
    @return void
   **/
  void PlanesDetection::locatePlanesUsingSACSegmentation(
    const PointCloudXYZPtr& cloudIn,
    std::vector<PointCloudXYZPtr>* planesVector,
    std::vector<pcl::ModelCoefficients>* coefficientsVector,
    std::vector<pcl::PointIndices::Ptr>* inliersVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("locatePlanesUsingSACSegmentation", "locatePlanes");
    #endif

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Optional
    seg.setOptimizeCoefficients(true);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(Parameters::HoleFusion::Planes::max_iterations);

    // Maybe a value needs to be set dynamically here, depending on
    // the distance of the kinect to the plane.
    seg.setDistanceThreshold(
      Parameters::HoleFusion::Planes::point_to_plane_distance_threshold);

    // Copy the input cloud to a point cloud that we will be processing
    PointCloudXYZPtr pointCloudProcessed (new PointCloudXYZ);
    pcl::copyPointCloud(*cloudIn, *pointCloudProcessed);

    int i = 0;
    int nr_points = static_cast<int> (pointCloudProcessed->points.size());

    // While 100 x num_points_to_exclude % of the original
    // cloud is still there
    while (pointCloudProcessed->points.size() >
      Parameters::HoleFusion::Planes::num_points_to_exclude * nr_points)
    {
      // The plane's coefficients
      pcl::ModelCoefficients coefficients;

      // The plane's inliers
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(pointCloudProcessed);
      seg.segment(*inliers, coefficients);

      // Add the coefficients and inliers to their respective vectors
      coefficientsVector->push_back(coefficients);
      inliersVector->push_back(inliers);

      if (inliers->indices.size () == 0)
      {
        ROS_DEBUG_NAMED(PKG_NAME,
          "Could not estimate a planar model for the given dataset.");

        break;
      }

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      // Extract the inliers
      extract.setInputCloud(pointCloudProcessed);
      extract.setIndices(inliers);

      // Remove the plane found from pointCloudProcessed and place it in
      // cloud_p. pointCloudProcessed goes unaffected.
      extract.setNegative(false);

      // The inliers of the largest planar component create cloud_p
      PointCloudXYZPtr cloud_p (new PointCloudXYZ);
      extract.filter(*cloud_p);

      // Push back the point cloud of the plane found
      planesVector->push_back(cloud_p);

      // We want to extract the rest of the points that were found to lie on
      // a plane
      extract.setNegative(true);

      // In short: cloud_f = pointCloudProcessed - cloud_p
      PointCloudXYZPtr cloud_f (new PointCloudXYZ);
      extract.filter(*cloud_f);

      // pointCloudProcessed is now without cloud_p, that is,
      // without the points that were
      // found to lie on the largest planar component of pointCloudProcessed
      pcl::copyPointCloud(*cloud_f, *pointCloudProcessed);

      // Increment the number of planes found
      i++;
    }

    #ifdef DEBUG_TIME
    Timer::tick("locatePlanesUsingSACSegmentation");
    #endif
  }

}  // namespace pandora_vision
