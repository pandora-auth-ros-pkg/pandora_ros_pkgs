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

#include "hole_fusion_node/planes_detection.h"

namespace pandora_vision
{
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

    if (applyVoxelFilter)
    {
      inCloud = applyVoxelGridFilter (inputCloud);
    }
    else
    {
      copyPointCloud(*inputCloud, *inCloud);
    }

    std::vector<PointCloudXYZPtr> planesVectorOut;
    std::vector<pcl::ModelCoefficients> coefficientsVectorOut;

    if (Parameters::segmentation_method == 0)
    {
      locatePlanesUsingSACSegmentation(inCloud,
          &planesVectorOut, &coefficientsVectorOut, inliersVector);
    }
    else if (Parameters::segmentation_method == 1)
    {
      locatePlanesUsingNormalsSACSegmentation(inCloud,
          &planesVectorOut, &coefficientsVectorOut, inliersVector);
    }

    #ifdef DEBUG_TIME
    Timer::tick("locatePlanes");
    #endif

    return planesVectorOut.size();
  }



  /**
    @brief Identify the planes in a point cloud and return a vector
    cointaining pointers to them.
    @param[in] inputCloud [const PointCloudXYZPtr&] The point cloud whose planes
    we wish to locate
    @param[in] applyVoxelFilter [const bool&] Apply the voxel filter or not on
    the input cloud.
    @param[out] planesVectorOut
    [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
    the output vector of pointers to planes
    @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*] The
    output vector of coefficients of each plane detected
    @return void
   **/
  void PlanesDetection::locatePlanes(const PointCloudXYZPtr& inputCloud,
    const bool& applyVoxelFilter,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planesVectorOut,
    std::vector<pcl::ModelCoefficients>* coefficientsVectorOut)
  {
    #ifdef DEBUG_TIME
    Timer::start("locatePlanes (void)", "checkHolesRectangleOutline");
    #endif

    double start_time = pcl::getTime();
    PointCloudXYZPtr inCloud (new PointCloudXYZ);

    if (applyVoxelFilter)
    {
      // std::cerr << "PointCloud before filtering: "
      // << inputCloud->width
      // << " x "
      // << inputCloud->height
      // << " data points."
      // << std::endl;

      inCloud = applyVoxelGridFilter (inputCloud);

      // std::cerr << "PointCloud after filtering: "
      // << inCloud->width
      // << " x "
      // << inCloud->height
      // << " data points ("
      // << pcl::getFieldsList (*inCloud)
      // << ")." << std::endl;
    }
    else
    {
      copyPointCloud(*inputCloud, *inCloud);
    }


    std::vector<pcl::PointIndices::Ptr> inliersVector;

    if (Parameters::segmentation_method == 0)
    {
      locatePlanesUsingSACSegmentation(inCloud,
        planesVectorOut, coefficientsVectorOut, &inliersVector);
    }
    else if (Parameters::segmentation_method == 1)
    {
      locatePlanesUsingNormalsSACSegmentation(inCloud,
        planesVectorOut, coefficientsVectorOut, &inliersVector);
    }
    //double end_time = pcl::getTime ();
    // std::cout << "Time taken from cloud input to planes' locating: "
    // << double (end_time - start_time)
    // << " sec."
    // << std::endl;
    #ifdef DEBUG_TIME
    Timer::tick("locatePlanes (void)");
    #endif
  }



  /**
    @brief Locates planes using the SACS segmentation
    (as stated in http://www.pointclouds.org/documentation/tutorials/
    extract_indices.php#extract-indices)
    @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose planes we
    wish to locate
    @param[out] planesVectorOut
    [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
    the output vector of pointers to planes
    @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
    The output vector of coefficients of each plane detected
    @param[out] inliersVectorOut [std::vector<pcl::PointIndices::Ptr>*]
    The inliers for each plane
    @return void
   **/
  void PlanesDetection::locatePlanesUsingSACSegmentation (
    const PointCloudXYZPtr& cloudIn,
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*  planesVectorOut,
    std::vector<pcl::ModelCoefficients>* coefficientsVectorOut,
    std::vector<pcl::PointIndices::Ptr>* inliersVectorOut)
  {
    #ifdef DEBUG_TIME
    Timer::start("locatePlanesUsingSACSegmentation", "locatePlanes");
    #endif

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    //!< Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //!< Optional
    seg.setOptimizeCoefficients (true);
    //!< Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (Parameters::max_iterations);

    //!< Maybe a value needs to be set dynamically here, depending on
    //!< the distance of the kinect to the plane.
    seg.setDistanceThreshold(
      Parameters::point_to_plane_distance_threshold);

    //!< Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f
      (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ps;
    std::vector<pcl::ModelCoefficients> coefficientsVector;
    std::vector<pcl::PointIndices::Ptr> inliersVector;

    int i = 0;
    int nr_points = static_cast<int> (cloudIn->points.size());
    //!< While 100 x num_points_to_exclude % of the original
    //!< cloud is still there
    while (cloudIn->points.size () >
      Parameters::num_points_to_exclude * nr_points)
    {
      //!< Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloudIn);
      seg.segment (*inliers, coefficients);

      if (inliers->indices.size () == 0)
      {
        std::cerr
          << "Could not estimate a planar model for the given dataset."
          << std::endl;
        break;
      }

      //!< Extract the inliers
      extract.setInputCloud (cloudIn);
      extract.setIndices (inliers);
      //!< Remove the plane found from cloudIn and place it in
      //!< loud_p. cloudIn goes unaffected.
      extract.setNegative (false);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p
        (new pcl::PointCloud<pcl::PointXYZ>);
      extract.filter (*cloud_p);
      // std::cerr << "PointCloud representing the planar component: "
      // << cloud_p->width
      // << " x "
      // << cloud_p->height
      // << " data points."
      // << std::endl;

      cloud_ps.push_back(cloud_p);
      coefficientsVector.push_back(coefficients);
      inliersVector.push_back(inliers);

      //!< Create the filtering object. Remove the plane \
      //!< found from cloudIn.
      //!< cloud_f = cloudIn - cloud_p. cloudIn goes unaffected.
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloudIn = *cloud_f;

      i++;


      //!< If the number of planes found so far exceeds the number one,
      //!< return. We are only interested in holes that lie on one plane.

      if (i > 1)
      {
        return;
      }
    }

    *planesVectorOut = cloud_ps;
    *coefficientsVectorOut = coefficientsVector;
    *inliersVectorOut = inliersVector;
    // std::cerr << "Total number of planes found: "
    // << cloud_ps.size()
    // << std::endl;
    #ifdef DEBUG_TIME
    Timer::tick("locatePlanesUsingSACSegmentation");
    #endif
  }



  /**
    @brief Locates planes using the normals SACS segmentation
    (http://pi-robot-ros-pkg.googlecode.com/svn-history/r303/trunk/pi_pcl/src/
    cylinder.cpp)
    @param[in] cloudIn [const PointCloudXYZPtr&] The point cloud whose planes we
    wish to locate
    @param[out] planesVectorOut
    [std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>*]
    The output vector of pointers to planes
    @param[out] coefficientsVectorOut [std::vector<pcl::ModelCoefficients>*]
    The output[out] vector of coefficients of each plane detected
    @param[out] inliersVectorOut [std::vector<pcl::PointIndices::Ptr>*]
    The inliers for each plane
    @return void
    **/
  void PlanesDetection::locatePlanesUsingNormalsSACSegmentation
    (const PointCloudXYZPtr& cloudIn,
     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planesVectorOut,
     std::vector<pcl::ModelCoefficients>* coefficientsVectorOut,
     std::vector<pcl::PointIndices::Ptr>* inliersVectorOut)
  {
    #ifdef DEBUG_TIME
    Timer::start("locatePlanesUsingNormalsSACSegmentation", "locatePlanes");
    #endif

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    //!< Create the segmentation object
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals
      (new pcl::PointCloud<pcl::Normal>);

    //!< Optional
    seg.setOptimizeCoefficients (true);
    //!< Mandatory
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.01);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (Parameters::max_iterations);

    //!< Maybe a value needs to be set dynamically here,
    //!< depending on the distance of the kinect to the plane.
    seg.setDistanceThreshold(
      Parameters::point_to_plane_distance_threshold);

    //!< Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f
      (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_ps;
    std::vector<pcl::ModelCoefficients> coefficientsVector;
    std::vector<pcl::PointIndices::Ptr> inliersVector;

    //!< Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud (cloudIn);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    int i = 0;
    int nr_points = static_cast<int> (cloudIn->points.size());
    //!< While 100 x num_points_to_exclude % of the original
    //!< cloud is still there
    while (cloudIn->points.size () >
        Parameters::num_points_to_exclude * nr_points)
    {
      //!< Segment the largest planar component from the
      //!< remaining cloud
      seg.setInputCloud (cloudIn);
      seg.setInputNormals(cloud_normals);
      seg.segment (*inliers, coefficients);

      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset."
          << std::endl;
        break;
      }

      //!< Extract the inliers
      extract.setInputCloud (cloudIn);
      extract.setIndices (inliers);
      //!< Remove the plane found from cloudIn and place it in cloud_p.
      //!< cloudIn goes unaffected.
      extract.setNegative (false);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p
        (new pcl::PointCloud<pcl::PointXYZ>);
      extract.filter (*cloud_p);
      // std::cerr << "PointCloud representing the planar component: "
      // << cloud_p->width
      // << " x "
      // << cloud_p->height
      // << " data points."
      // << std::endl;

      cloud_ps.push_back(cloud_p);
      coefficientsVector.push_back(coefficients);
      inliersVector.push_back(inliers);

      /** Create the filtering object
        Remove the plane found from cloudIn.
        cloud_f = cloudIn - cloud_p. cloudIn goes unaffected.
       **/
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloudIn = *cloud_f;

      //!< Create the filtering object
      extract_normals.setNegative (true);
      extract_normals.setInputCloud (cloud_normals);
      extract_normals.setIndices (inliers);
      extract_normals.filter (*cloud_normals);

      i++;

       //!< If the number of planes found so far exceeds the number one,
       //!< return. We are only interested in holes that lie on one plane.

      if (i > 1)
      {
        return;
      }
    }

    *planesVectorOut = cloud_ps;
    *coefficientsVectorOut = coefficientsVector;
    *inliersVectorOut = inliersVector;
    // std::cerr << "Total number of planes found: "
    // << cloud_ps.size()
    // << std::endl;
    #ifdef DEBUG_TIME
    Timer::tick("locatePlanesUsingNormalsSACSegmentation");
    #endif
  }



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

    PointCloudXYZPtr cloudOut (new PointCloudXYZ());

    pcl::VoxelGrid <pcl::PointXYZ> sor;
    sor.setInputCloud (cloudIn);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloudOut);

    #ifdef DEBUG_TIME
    Timer::tick("applyVoxelGridFilter");
    #endif

    return cloudOut;
  }

} // namespace pandora_vision
