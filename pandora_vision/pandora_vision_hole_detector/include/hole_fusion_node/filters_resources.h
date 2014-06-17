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

#ifndef HOLE_FUSION_NODE_FILTERS_RESOURCES_H
#define HOLE_FUSION_NODE_FILTERS_RESOURCES_H

#include "utils/defines.h"
#include "utils/blob_detection.h"
#include "utils/holes_conveyor.h"
#include "utils/parameters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class FiltersResources
    @brief Provides methods for obtaining hole-related resources
   **/
  class FiltersResources
  {
    public:

      /**
        @brief Each Depth and RGB filter requires the construction of a set
        of vectors which uses to determine the validity of each hole.
        The total number of vectors is finite; every filter uses vectors from
        this pool of vectors. This method centrally constructs the necessary
        vectors in runtime, depending on which filters are commanded to run
        @param[in] conveyor [const HolesConeveyor&] The candidate holes
        from which each element of the vector will be constructed
        @param[in] image [const cv::Mat&] An image needed for its size
        @param[in] inflationSize [const int&] The bounding rectangles
        inflation size
        @param[in] interpolationMethod [const int&] The interpolation method
        of the depth image. If its value is other than zero, the depth filters
        cannot be applied.
        @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
        A vector containing an image (the mask) for each hole
        @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
        A vector that holds sets of points's indices; each set holds the
        indices of the points inside the outline of each hole
        @param[out] inflatedRectanglesVector
        [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
        vertices of the in-image-bounds inflated rectangles
        @param[out] inflatedRectanglesIndices [std::vector<int>*]
        The vector that holds the indices of the original holes whose
        inflated bounding rectangles is within the image's bounds.
        @param[out] intermediatePointsImageVector [std::vector<cv::Mat>*]
        A vector that holds the image of the intermediate points between
        a hole's outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
        A vector that holds the intermediate points' indices between a hole's
        outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @return void
       **/
      static void createCheckerRequiredVectors(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const int& inflationSize,
        const int& interpolationMethod,
        std::vector<cv::Mat>* holesMasksImageVector,
        std::vector<std::set<unsigned int> >* holesMasksSetVector,
        std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
        std::vector<int>* inflatedRectanglesIndices,
        std::vector<cv::Mat>* intermediatePointsImageVector,
        std::vector<std::set<unsigned int> >* intermediatePointsSetVector);

      /**
        @brief Some hole checkers require the construction of a hole's mask,
        that is, the pixels inside the hole; either in cv::Mat form or in
        a set form which contains points' indices.
        Construct each form here; this method makes it possible to brushfire
        once for every hole, instead of twice, if the image and set vectors
        are needed
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image required only for the
        masks' size
        @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
        A vector containing an image (the mask) for each hole
        @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
        A vector that holds sets of points;
        each set holds the inside points of each hole
        @return void
       **/
      static void createHolesMasksVectors(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        std::vector<cv::Mat>* holesMasksImageVector,
        std::vector<std::set<unsigned int> >* holesMasksSetVector);

      /**
        @brief Some hole checkers require the construction of a hole's mask,
        that is, the pixels inside the hole with a value of
        value 255 while the background pixels are with 0 value.
        Construct each mask here, instead of in each checker.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image required only for the
        masks' size
        @param[out] holesMasksImageVector [std::vector<cv::Mat>*]
        A vector containing an image (the mask) for each hole
        @return void
       **/
      static void createHolesMasksImageVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        std::vector<cv::Mat>* holesMasksImageVector);

      /**
        @brief Some hole checkers require access to a hole's inside points,
        Construct a set of points for each hole, and store all of them in
        a vector
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image required to access
        each hole
        @param[out] holesMasksSetVector [std::vector<unsigned int>*] A vector
        that holds sets of points; each set holds the inside points of each hole
        @return void
       **/
      static void createHolesMasksSetVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        std::vector<std::set<unsigned int> >* holesMasksSetVector);

      /**
        @brief Some checkers require the construction of a hole's inflated
        rectangle in order to validate a hole. Construct each mask here.
        Each vector element contains the four vertices of the inflated
        rectangle. A hole's bounding rectangle is inflated by a standard size;
        inflated rectangles that go beyond the image's bounds are discarded,
        that is, the output vector contains the indices of the original
        keypoints whose inflated bounding rectangles is within the image's
        bounds.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflationSize [const int&] The bounding rectangles
        inflation size in pixels
        @param[out] inflatedRectanglesVector
        [std::vector<std::vector<cv::Point2f> >*] The vector that holds the
        vertices of the in-image-bounds inflated rectangles
        @param[out] inflatedRectanglesIndices [std::vector<int>*]
        The vector that holes the indices of the original holes whose
        inflated bounding rectangles is within the image's bounds.
        @return void
       **/
      static void createInflatedRectanglesVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const int& inflationSize,
        std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
        std::vector<int>* inflatedRectanglesIndices);

      /**
        @brief Some hole checkers require the construction of a hole's mask
        for the points between a hole's outline and its inflated bounding
        rectangle, either in cv::Mat form or in a set form which contains
        points' indices.
        Construct each form here; this method makes it possible to brushfire
        once for every hole, instead of twice, if the image and set vectors
        are needed
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of each rectangle that corresponds to a specific hole
        inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding rectangle.
        Used primarily because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        A vector that holds the image of the intermediate points between
        a hole's outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
        A vector that holds the intermediate points' indices between a hole's
        outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @return void
       **/
      static void createIntermediateHolesPointsVectors(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<cv::Mat>* intermediatePointsImageVector,
        std::vector<std::set<unsigned int> >* intermediatePointsSetVector);

      /**
        @brief For each hole, this function finds the points between the hole's
        outline and the rectangle (inflated or not) that corrensponds to it.
        These points are then stored in an image
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of each rectangle that corresponds to a specific hole
        inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&]
        A vector that is used to identify a hole's corresponding rectangle.
        Used primarily because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[out] intermediatePointsImageVector [std::vector<cv::Mat>*]
        A vector that holds the image of the intermediate points between
        a hole's outline and its bounding box, for each hole whose identifier
        exists in the @param inflatedRectanglesIndices vector
        @return void
       **/
      static void createIntermediateHolesPointsImageVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<cv::Mat>* intermediatePointsImageVector);

      /**
        @brief For each hole, this function finds the points between the hole's
        outline and the rectangle (inflated or not) that corrensponds to it.
        These points are then stored in a std::set of ints.
        @param[in] conveyor [const HolesConveyor&] The conveyor of holes
        @param[in] image [const cv::Mat&] An image needed only for
        its size
        @param[in] inflatedRectanglesVector
        [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
        the vertices of each rectangle that corresponds to a specific hole
        inside the coveyor
        @param[in] inflatedRectanglesIndices [const std::vector<int>&] A vector
        that is used to identify a hole's corresponding rectangle.
        Used primarily because the rectangles used are inflated rectangles;
        not all holes possess an inflated rectangle
        @param[out] intermediatePointsSetVector
        [std::vector<std::set<unsigned int> >*]
        A vector that holds the intermediate points' indices for each hole
        whose identifier exists in the @param inflatedRectanglesIndices vector
        @return void
       **/
      static void createIntermediateHolesPointsSetVector(
        const HolesConveyor& conveyor,
        const cv::Mat& image,
        const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
        const std::vector<int>& inflatedRectanglesIndices,
        std::vector<std::set<unsigned int> >* intermediatePointsSetVector);

  };

} // namespace pandora_vision

#endif  // HOLE_FUSION_NODE_FILTERS_RESOURCES_H
