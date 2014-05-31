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

#ifndef HOLE_FUSION_NODE_PLANES_DETECTION_H
#define HOLE_FUSION_NODE_PLANES_DETECTION_H

#include "utils/parameters.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class PlanesDetection
    @brief Provides methods for plane detection and extraction
   **/
  class PlanesDetection
  {
    public:

      /**
        @brief Applies a voxel grid filtering
        (http://pointclouds.org/documentation/tutorials/voxel_grid.php)
        @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud to filter
        @return PointCloudXYZPtr A pointer to the filtered cloud
       **/
      static PointCloudXYZPtr applyVoxelGridFilter
        (const PointCloudXYZPtr& cloudIn);

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
      static int locatePlanes(const PointCloudXYZPtr& inputCloud,
        const bool& applyVoxelFilter,
        std::vector<pcl::PointIndices::Ptr>* inliersVector);

      /**
        @brief Locates planes using the SAC segmentation
        (as stated in http://www.pointclouds.org/documentation/tutorials/
        extract_indices.php#extract-indices)
        @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose
        planes we wish to locate
        @param[out] planesVectorOut [std::vector<PointCloudXYZPtr>*]
        The output vector of pointers to point cloud planes
        @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
        The output vector of coefficients of each plane detected
        @param[out] inliersVectorOut [std::vector<pcl::PointIndices::Ptr>*]
        The inliers for each plane
        @return void
       **/
      static void locatePlanesUsingSACSegmentation
        (const PointCloudXYZPtr& cloudIn,
         std::vector<PointCloudXYZPtr>* planesVector,
         std::vector<pcl::ModelCoefficients>* coefficientsVector,
         std::vector<pcl::PointIndices::Ptr>* inliersVector);

  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_PLANES_DETECTION_H
