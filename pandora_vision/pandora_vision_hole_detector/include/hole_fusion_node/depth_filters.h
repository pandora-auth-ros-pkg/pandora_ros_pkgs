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

#ifndef HOLE_FUSION_NODE_DEPTH_FILTERS_H
#define HOLE_FUSION_NODE_DEPTH_FILTERS_H

#include <math.h>
#include "utils/holes_conveyor.h"
#include "utils/blob_detection.h"
#include "utils/edge_detection.h"
#include "utils/visualization.h"
#include "hole_fusion_node/planes_detection.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleDetector
    @brief Provides the functionalities for detecting holes [functional]
   **/
  class DepthFilters
  {
    public:

      /**
        @brief Checks for valid holes just by depth difference between the c
        enter of the blob and the edges of the bounding box
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @param[in] inflationSize [const in&t] The number of pixels by which the
        bounding rectange will be inflated
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @return void
       **/
      static void checkHolesDepthDiff(
        const cv::Mat& depthImage,
        const HolesConveyor& conveyor,
        std::vector<std::string>* msgs,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Checks for valid holes by area / depth comparison
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @return void
       **/
      static void checkHolesDepthArea(
        const cv::Mat& depthImage,
        const HolesConveyor& conveyor,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Brushfire from a blobs's outline to its bounding box
        with an inflation size (inflates the rectangle by inflationSize pixels).
        If the points between the blob's outline and the inflated rectangle
        lie on one plane, this blob is a hole. Although all
        planes are considered valid, the @param probabilitiesVector hint
        to the validity of the candidate hole through this filter
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
        The original point cloud acquired from the depth sensor
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in] inflationsize [const int&] Grow the rectangle by
        inflationsize as to acquire more points to check for plane existence.
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @return void
       **/
      static void checkHolesBrushfireOutlineToRectangle(
        const cv::Mat& inImage,
        const PointCloudXYZPtr& initialPointCloud,
        const HolesConveyor& conveyor,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief  Given the bounding box of a blob, inflate it.
        All the points that lie on the (edges of the) rectangle should
        also lie on exactly one plane for the blob to be a hole. Although all
        planes are considered valid, the @param probabilitiesVector hint
        to the validity of the candidate hole through this filter
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
        The original point cloud,  uninterpolated, undistorted.
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in] inflationSize [cosnt int&] grow the rectangle by inflationSize
        as to acquire more points to check for plane existence.
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @return void
       **/
      static void checkHolesRectangleOutline(
        const cv::Mat& inImage,
        const PointCloudXYZPtr& initialPointCloud,
        const HolesConveyor& conveyor,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief Checks the homogenity of the gradient of depth in an area
        enclosed by @param inOutlines
        @param[in] interpolatedDepthImage [const cv::Mat&] The input depth image
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[out] msgs [std::vector<std::string>*] Debug messages
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @return void
       **/
      static void checkHolesDepthHomogenity(
        const cv::Mat& interpolatedDepthImage,
        const HolesConveyor& conveyor,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Apply a cascade-like hole checker. Each filter applied is
        attached to an order which relates to the sequence of the overall
        filter execution.
        @param[in] interpolatedDepthImage [const cv::Mat&] The denoised depth
        image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
        The undistorted input point cloud
        @param[in] conveyor [const HolesConveyor&] A struct that
        contains the final valid holes
        @param[out] probabilitiesVector [std::vector<std::vector<float> >*]
        A 2D vector of probabilities hinting to the certainty degree with
        which each candidate hole is associated for every
        active filter executed.
        While the returned set may be reduced in size,
        the size of this vector is the same throughout and equal to the number
        of active filters by the number of keypoints found and
        published by the rgb node.
        @return void
       **/
      static void checkHoles(
        const cv::Mat& interpolatedDepthImage,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& initialPointCloud,
        const HolesConveyor& conveyor,
        std::vector<std::vector<float> >* probabilitiesVector);

      /**
        @brief Apply a cascade-like hole checker. Each filter applied
        is attached to an order which relates to the sequence of the overall
        filter execution.
        @param[in] method [const unsigned int&] The filter identifier to execute
        @param[in] img [const cv::Mat&] The input depth image
        @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&] The
        original point cloud that corresponds to the input depth image
        @param[in] conveyor [const HolesConveyor&] The structure
        that holds the final holes' data
        @param[in] inflationSize [const int&] The amount of pixels by which each
        bounding box is inflated
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree with which each
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node.
        @param[in][out] imgs [std::vector<cv::Mat>*] A vector of images which
        shows the holes that are considered valid by each filter
        @param[in][out] msgs [std::vector<std::string>*] Debug messages
        @return void
       **/
      static void applyFilter(
        const unsigned int& method,
        const cv::Mat& img,
        const PointCloudXYZPtr& pointCloud,
        const HolesConveyor& conveyor,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector,
        std::vector<cv::Mat>* imgs,
        std::vector<std::string>* msgs);
  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_DEPTH_FILTERS_H
