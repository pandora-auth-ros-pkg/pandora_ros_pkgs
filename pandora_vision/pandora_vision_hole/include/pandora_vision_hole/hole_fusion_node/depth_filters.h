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

#ifndef PANDORA_VISION_HOLE_HOLE_FUSION_NODE_DEPTH_FILTERS_H
#define PANDORA_VISION_HOLE_HOLE_FUSION_NODE_DEPTH_FILTERS_H

#include <math.h>
#include "hole_fusion_node/utils/holes_conveyor.h"
#include "hole_fusion_node/utils/outline_discovery.h"
#include "hole_fusion_node/utils/edge_detection.h"
#include "hole_fusion_node/utils/visualization.h"
#include "hole_fusion_node/planes_detection.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace hole_fusion
{
  /**
    @class DepthFilters
    @brief Provides checks and pertinent methods as a means to validate holes
    through a depth-based analysis
   **/
  class DepthFilters
  {
    public:
      /**
        @brief Checks for valid holes by area / depth comparison
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] holesMasksSetVector
        [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points; each point is internal to its
        respective hole
        @param[out] msgs [std::vector<std::string>*] Messages for debug
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
        const HolesConveyor& conveyor,
        const cv::Mat& depthImage,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Checks for valid holes just by the depth difference between
        the keypoint of the blob and the edges of its bounding box
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of the inflated rectangle that corresponds to a specific
        hole inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding inflated
        rectangle.
        Used because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[out] msgs [std::vector<std::string>*] Messages for debug reasons
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
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Checks the homogeneity of the gradient of an interpolated
        depth image in areas denoted by the points inside the
        holesMasksSetVector vector
        @param[in] conveyor [const HolesConveyor&] The candidate holes
        @param[in] interpolatedDepthImage [const cv::Mat&] The input
        interpolated depth image
        @param[in] holesMasksSetVector
        [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points; each point is internal to its
        respective hole
        @param[out] msgs [std::vector<std::string>*] Debug messages
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @return void
       **/
      static void checkHolesDepthHomogeneity(
        const HolesConveyor& conveyor,
        const cv::Mat& interpolatedDepthImage,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

      /**
        @brief If the intermediate points (points between a hole's outline
        and its bounding rectangle) for each hole lie on one plane,
        this hole is considered valid. Although plane constitution alone is
        considered valid, the @param probabilitiesVector hint to the
        validity of the candidate hole through this filter
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud [const PointCloudPtr&]
        The point cloud acquired from the depth sensor, interpolated
        @param[in] intermediatePointsSetVector
        [const std::vector<std::set<unsigned int> >& ] A vector that holds for
        each hole a set of points;
        these points are the points between the hole's outline and its
        bounding rectangle
        @param[in] inflatedRectanglesIndices [const std::vector<int>&] Because
        each hole's bounding rectangle may be inflated, and thus not all holes
        possess a bounding rectangle by this process, in this vector is stored
        the indices of the holes whose inflated bounding box is inside the
        image's bounds
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @param[out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesOutlineToRectanglePlaneConstitution(
        const cv::Mat& inImage,
        const PointCloudPtr& initialPointCloud,
        const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief All the points that lie on the (edges of the) rectangle should
        also lie on exactly one plane for the blob to be a hole. Although all
        planes are considered valid, the @param probabilitiesVector hint
        to the validity of the candidate hole through this filter
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud [const PointCloudPtr&]
        The point cloud acquired from the depth sensor, interpolated
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of the inflated rectangle that corresponds to a specific
        hole inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding inflated
        rectangle.
        Used because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities, each position of which hints to the certainty degree
        with which the associated candidate hole is associated.
        While the returned set may be reduced in size, the size of this vector
        is the same throughout and equal to the number of keypoints found and
        published by the rgb node
        @param[out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesRectangleEdgesPlaneConstitution(
        const cv::Mat& inImage,
        const PointCloudPtr& initialPointCloud,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);
  };

}  // namespace hole_fusion
}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_HOLE_FUSION_NODE_DEPTH_FILTERS_H
