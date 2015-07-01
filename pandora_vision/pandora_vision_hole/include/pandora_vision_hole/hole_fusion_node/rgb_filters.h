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

#ifndef PANDORA_VISION_HOLE_HOLE_FUSION_NODE_RGB_FILTERS_H
#define PANDORA_VISION_HOLE_HOLE_FUSION_NODE_RGB_FILTERS_H

#include <math.h>
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
namespace pandora_vision_hole
{
  /**
    @class RgbFilters
    @brief Provides checks and pertinent methods as a means to validate holes
    through RGB-image-based analysis
   **/
  class RgbFilters
  {
    public:
      /**
        @brief Checks for colour homogeneity in a region where points are
        constrained inside each hole. The colors of the image are reduced
        and their different values inside each candidate hole are counted
        and averaged against the maximum number of colours possible.
        @param[in] inImage [const cv::Mat&] The RGB image in CV_8UC3 format
        @param[in] holesMasksImageVector [const std::vector<cv::Mat>&] A vector
        containing the masks needed to produce the histograms of the points
        inside each hole's outline
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @param[in,out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesColorHomogeneity(
        const cv::Mat& inImage,
        const std::vector<cv::Mat>& holesMasksVector,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief Checks for difference in mean value of luminosity between
        (1) the pixels in between a hole's bounding box edges and the points
        outside the hole's outline and
        (2) the points inside the hole's outline.
        @param[in] inImage [const cv::Mat&] The RGB image in CV_8UC3 format
        @param[in] holesMasksSetVector
        [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points's indices;
        each point is internal to its respective hole
        @param[in] intermediatePointsSetVector
        [const std::vector<std::set<unsigned int> >& ] A vector that holds
        for each hole a set of points;
        these points are the points between the hole's outline and its
        bounding rectangle
        @param[in] rectanglesIndices [const std::vector<int>&] A vector that
        is used to identify a hole's corresponding rectangle. Used primarily
        because the rectangles used are inflated rectangles; not all holes
        possess an inflated rectangle
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @param[out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesLuminosityDiff(
        const cv::Mat& inImage,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
        const std::vector<int>& rectanglesIndices,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief Given a set of keypoints, their respective outline and
        bounding box points, and a model histogram, this filter looks for near
        equation between the histograms of the points between the blob's
        outline and the bounding box's edges and the model histogram,
        and for major difference between the
        histograms of the bounding box and the points inside the outline of the
        blob.
        @param[in] inImage [const cv::Mat&] The RGB image in CV_8UC3 format
        @param[in] inHistogram [const std::vector<cv::MatND>&]
        The vector of model histograms
        @param[in] holesMasksImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points inside each hole's outline
        @param[in] intermediatePointsImageVector [const std::vector<cv::Mat>&]
        A vector containing masks of the points outside each hole's outline,
        but inside its bounding rectangle
        @param[in] rectanglesIndices [const std::vector<int>&] A vector that
        is used to identify a hole's corresponding rectangle. Used primarily
        because the rectangles used are inflated rectangles; not all holes
        possess an inflated rectangle
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @param[in,out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesTextureDiff(
        const cv::Mat& inImage,
        const std::vector<cv::MatND>& inHistogram,
        const std::vector<cv::Mat>& holesMasksImageVector,
        const std::vector<cv::Mat>& intermediatePointsImageVector,
        const std::vector<int>& rectanglesIndices,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);

      /**
        @brief Given a set of keypoints, their respective outline and
        bounding box points, and a model histogram, this filter creates the
        back project of the @param inImage based on @param inHistogram and
        exports a vector of probabilities, that is a vector of how probable it
        is for a candidate hole's points between the blob's outline points
        and the bounding box's edges to have a high probability
        in the back project image, and for the points inside the candidate
        hole's outline to have a low probability in the back project image
        @param[in] inImage [const cv::Mat&] The RGB image in CV_8UC3 format
        @param[in] inHistogram [const std::vector<cv::MatND>&]
        The vector of model histograms
        @param[in] holesMasksSetVector
        [const std::vector<std::set<unsigned int> >&]
        A vector that holds sets of points; each point is internal to its
        respective hole
        @param[in] intermediatePointsSetVector
        [const std::vector<std::set<unsigned int> >& ] A vector that holds
        for each hole a set of points' indices;
        these points are the points between the hole's outline and its
        bounding rectangle
        @param[in] rectanglesIndices [const std::vector<int>&] A vector that
        is used to identify a hole's corresponding rectangle. Used primarily
        because the rectangles used are inflated rectangles; not all holes
        possess an inflated rectangle
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @param[in,out] msgs [std::vector<std::string>*] Messages for
        debug reasons
        @return void
       **/
      static void checkHolesTextureBackProject(
        const cv::Mat& inImage,
        const std::vector<cv::MatND>& inHistogram,
        const std::vector<std::set<unsigned int> >& holesMasksSetVector,
        const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
        const std::vector<int>& rectanglesIndices,
        std::vector<float>* probabilitiesVector,
        std::vector<std::string>* msgs);
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_HOLE_FUSION_NODE_RGB_FILTERS_H
