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
 * Author: Alexandros Philotheou
 *********************************************************************/

#ifndef HOLE_FUSION_NODE_FILTERS_H
#define HOLE_FUSION_NODE_FILTERS_H

#include <math.h>
#include "hole_fusion_node/planes_detection.h"
#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/rgb_filters.h"
#include "utils/outline_detection.h"
#include "utils/edge_detection.h"
#include "utils/histogram.h"
#include "utils/holes_conveyor.h"
#include "utils/morphological_operators.h"
#include "utils/parameters.h"
#include "utils/visualization.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class Filters
    @brief Provides checks and pertinent methods as a means to validate holes
    through all available filtering methods
   **/
  class Filters
  {
    public:

      /**
        @brief Applies a specific active filter, either from an RGB
        or a Depth sources.
        @param[in] conveyor [const HolesConveyor&]
        The conveyor of candidateholes
        @param[in] filteringMethod [const int&]
        Each filter's identifier
        @param[in] depthImage [const cv::Mat&]
        The interpolated depth image
        @param[in] rgbImage [const cv::Mat&]
        The rgb image
        @param[in] inHistogram [const cv::MatND&]
        The model histogram's H and S component
        @param[in] pointCloud [const PointCloudPtr&]
        The original point cloud that corresponds to the input depth image
        @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points's indices;
        each point is internal to its respective hole
        @param[in] holesMasksImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points inside each hole's outline
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of the inflated rectangle that corresponds to a specific
        hole inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding rectangle.
        Used primarily because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[in] intermediatePointsSetVector
        [const std::vector<std::set<unsigned int> >& ] A vector that holds for
        each hole a set of points' indices; these points are the points
        between the hole's outline and its bounding rectangle
        @param[in] intermediatePointsImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points outside each hole's outline,
        but inside its bounding rectangle
        @param[out] probabilitiesVector [std::vector<float>*]
        A vector of probabilities hinting to the certainty degree with which
        each candidate hole is associated. While the returned set may be
        reduced in size, the size of this vector is the same throughout and
        equal to the number of keypoints found and processed by the
        Hole Fusion node.
        @param[out] imgs [std::vector<cv::Mat>*]
        A vector of images which shows the holes that are considered
        valid by each filter
        @param[out] msgs [std::vector<std::string>*]
        Debug messages
        @return void
       **/
      static void applyFilter(
        const HolesConveyor& conveyor,
        const int& filteringMethod,
        const cv::Mat& depthImage,
        const cv::Mat& rgbImage,
        const cv::MatND& inHistogram,
        const PointCloudPtr& pointCloud,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        const std::vector<cv::Mat>& holesMasksImageVector,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
        const std::vector<cv::Mat>& intermediatePointsImageVector,
        std::vector<float>* probabilitiesVector,
        std::vector<cv::Mat>* imgs,
        std::vector<std::string>* msgs);

      /**
        @brief Applies all active filters, from both RGB and Depth sources.
        The order of execution is derived from the dynamic reconfigure
        facility.
        @param[in] conveyor [const HolesConveyor&]
        The conveyor of candidateholes
        @param[in] filteringMode [const int&]
        The filtering mode used: If RGBD_MODE, depth analysis is possible,
        and depth-based filters will be utilized.
        If RGB_ONLY_MODE, depth-based filters cannot be utilized,
        so validation of candidate holes can only be made using
        RGB-based filters.
        @param[in] depthImage [const cv::Mat&]
        The interpolated depth image
        @param[in] rgbImage [const cv::Mat&]
        The rgb image
        @param[in] inHistogram [const cv::MatND&]
        The model histogram's H and S component
        @param[in] pointCloud [const PointCloudPtr&]
        The original point cloud that corresponds to the input depth image
        @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points's indices;
        each point is internal to its respective hole
        @param[in] holesMasksImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points inside each hole's outline
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of the inflated rectangle that corresponds to a specific
        hole inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding rectangle.
        Used primarily because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[in] intermediatePointsSetVector
        [const std::vector<std::set<unsigned int> >& ] A vector that holds for
        each hole a set of points' indices; these points are the points
        between the hole's outline and its bounding rectangle
        @param[in] intermediatePointsImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points outside each hole's outline,
        but inside its bounding rectangle
        @param[out] probabilitiesVector [std::vector<float>*]
        A vector of probabilities hinting to the certainty degree with which
        each candidate hole is associated. While the returned set may be
        reduced in size, the size of this vector is the same throughout and
        equal to the number of keypoints found and processed by the
        Hole Fusion node.
        @return void
       **/
      static void applyFilters(
        const HolesConveyor& conveyor,
        const int& interpolationMethod,
        const cv::Mat& depthImage,
        const cv::Mat& rgbImage,
        const cv::MatND& inHistogram,
        const PointCloudPtr& pointCloud,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        const std::vector<cv::Mat>& holesMasksImageVector,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
        const std::vector<cv::Mat>& intermediatePointsImageVector,
        std::vector<std::vector<float> >* probabilitiesVector);

  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_FILTERS_H
