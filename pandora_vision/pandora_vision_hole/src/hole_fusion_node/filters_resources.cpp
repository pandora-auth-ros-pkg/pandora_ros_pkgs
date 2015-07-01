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

#include "hole_fusion_node/filters_resources.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
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
    @param[in] filteringMode [const int&]
    The filtering mode used: If RGBD_MODE, depth analysis is possible,
    and depth-based filters' resources will be utilized.
    If RGB_ONLY_MODE, depth-based filters cannot be utilized,
    so validation of candidate holes can only be made using
    RGB-based filters, so only resources needed by those will be
    needed to be constructed.
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
    A vector that holds the intermediate points' between a hole's outline
    and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void FiltersResources::createCheckerRequiredVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const int& inflationSize,
    const int& filteringMode,
    std::vector<cv::Mat>* holesMasksImageVector,
    std::vector<std::set<unsigned int> >* holesMasksSetVector,
    std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
    std::vector<int>* inflatedRectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCheckerRequiredVectors", "filterHoles");
    #endif

    // Indicate the necessity of creating particular resources
    bool enable_holesMasksImageVector = false;
    bool enable_holesMasksSetVector = false;
    bool enable_inflatedRectanglesVectorAndIndices = false;
    bool enable_intermediatePointsImageVector = false;
    bool enable_intermediatePointsSetVector = false;

    // If the conditions permit for the depth filters to be applied,
    // create their resources
    if (filteringMode == RGBD_MODE)
    {
      // The depth diff filter requires only the contruction of the vectors
      // that have to do with the inflation of holes' rectangles
      if (Parameters::Filters::DepthDiff::priority > 0)
      {
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The depth/area filter requires only the construction of sets that
      // hold the indices of points inside holes' outlines
      if (Parameters::Filters::DepthArea::priority > 0)
      {
        enable_holesMasksSetVector = true;
      }

      // The intermediate points plane constitution filter requires exactly
      // the construction of vectors pertaining to holes' inflation and
      // and a vector of sets of indices of points between holes' outline and
      // their respective (inflated) bounding rectangle
      if (Parameters::Filters::IntermediatePointsPlaneConstitution::priority > 0)
      {
        enable_intermediatePointsSetVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The outline of rectangle plane constitution filter requires
      // the construction of vectors pertaining to holes' inflation
      if (Parameters::Filters::RectanglePlaneConstitution::priority > 0)
      {
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The depth homogeneity filter requires the construction of sets of
      // points' indices; these points are the ones inside holes' outlines
      if (Parameters::Filters::DepthHomogeneity::priority > 0)
      {
        enable_holesMasksSetVector = true;
      }

      // The color homogeneity filter requires a vector of holes' masks
      // that will be used to extract their histograms
      if (Parameters::Filters::ColourHomogeneity::rgbd_priority > 0)
      {
        enable_holesMasksImageVector = true;
      }

      // The luminosity diff filter requires the set of points' indices
      // that are inside a hole's outline,
      // the set of points' indices that are outside a hole's outline
      // but inside its (inflated) bounding rectangle
      // and the inflated rectangles and indices of the respective
      // valid keypoints
      if (Parameters::Filters::LuminosityDiff::rgbd_priority > 0)
      {
        enable_holesMasksSetVector = true;
        enable_intermediatePointsSetVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The texture diff filter requires the construction of an image mask
      // vector for the points inside holes' outline and of image and set
      // masks for the points outside holes' outline but inside their (inflated)
      // bounding box
      // as it checks for texture metrics difference between the
      // histograms of the points inside a hole's outline and outside
      // the hole's outline but inside its (inflated) bounding rectangle
      if (Parameters::Filters::TextureDiff::rgbd_priority > 0)
      {
        enable_holesMasksImageVector = true;
        enable_intermediatePointsImageVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The texture backproject filter uses two sets: they respectively contain
      // the indices of points inside holes' outlines and the indices of points
      // outside holes' outlines but inside their (inflated) bounding rectangle.
      // Hence, we also need the construction of inflated rectangles' vectors
      if (Parameters::Filters::TextureBackprojection::rgbd_priority > 0)
      {
        enable_holesMasksSetVector = true;
        enable_intermediatePointsSetVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }
    }
    // Depth-based filters cannot be applied. Create only the necessary
    // resources needed by the RGB-based filters, according to the *_urgent
    // order
    else if (filteringMode == RGB_ONLY_MODE)
    {
      // The color homogeneity filter requires a vector of holes' masks
      // that will be used to extract their histograms
      if (Parameters::Filters::ColourHomogeneity::rgb_priority > 0)
      {
        enable_holesMasksImageVector = true;
      }

      // The luminosity diff filter requires the set of points' indices
      // that are inside a hole's outline,
      // the set of points' indices that are outside a hole's outline
      // but inside its (inflated) bounding rectangle
      // and the inflated rectangles and indices of the respective
      // valid keypoints
      if (Parameters::Filters::LuminosityDiff::rgb_priority > 0)
      {
        enable_holesMasksSetVector = true;
        enable_intermediatePointsSetVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The texture diff filter requires the construction of an image mask
      // vector for the points inside holes' outline and of image and set
      // masks for the points outside holes' outline but inside their (inflated)
      // bounding box
      // as it checks for texture metrics difference between the
      // histograms of the points inside a hole's outline and outside
      // the hole's outline but inside its (inflated) bounding rectangle
      if (Parameters::Filters::TextureDiff::rgb_priority > 0)
      {
        enable_holesMasksImageVector = true;
        enable_intermediatePointsImageVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }

      // The texture backproject filter uses two sets: they respectively contain
      // the indices of points inside holes' outlines and the indices of points
      // outside holes' outlines but inside their (inflated) bounding rectangle.
      // Hence, we also need the construction of inflated rectangles' vectors
      if (Parameters::Filters::TextureBackprojection::rgb_priority > 0)
      {
        enable_holesMasksSetVector = true;
        enable_intermediatePointsSetVector = true;
        enable_inflatedRectanglesVectorAndIndices = true;
      }
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME, "[Hole Fusion node] Resource creation failure");
    }

    // Create the necessary resources

    // The generation of image masks presupposes the generation of set masks
    // within method createHolesMasksImageVector
    if (enable_holesMasksImageVector && !enable_holesMasksSetVector)
    {
      createHolesMasksImageVector(conveyor, image, holesMasksImageVector);
    }

    if (enable_holesMasksSetVector && !enable_holesMasksImageVector)
    {
      createHolesMasksSetVector(conveyor, image, holesMasksSetVector);
    }

    // Generate both types of masks
    if (enable_holesMasksSetVector && enable_holesMasksImageVector)
    {
      createHolesMasksVectors(conveyor, image,
        holesMasksImageVector, holesMasksSetVector);
    }

    if (enable_inflatedRectanglesVectorAndIndices)
    {
      createInflatedRectanglesVector(conveyor,
        image,
        inflationSize,
        inflatedRectanglesVector,
        inflatedRectanglesIndices);
    }

    // The generation of image masks presupposes the generation of set masks
    // within method createIntermediateHolesPointsImageVector
    if (enable_intermediatePointsImageVector &&
      !enable_intermediatePointsSetVector)
    {
      // The intermediate points images vector depends on the
      // inflated rectangles vectors, which has been created previously
      createIntermediateHolesPointsImageVector(conveyor,
        image,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsImageVector);
    }

    if (enable_intermediatePointsSetVector &&
      !enable_intermediatePointsImageVector)
    {
      // The intermediate points set vector depends on the
      // inflated rectangles vectors, which has been created previously
      createIntermediateHolesPointsSetVector(conveyor,
        image,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsSetVector);
    }

    if (enable_intermediatePointsSetVector &&
      enable_intermediatePointsImageVector)
    {
      // The intermediate points set vector depends on the
      // inflated rectangles vectors, which has been created previously
      createIntermediateHolesPointsVectors(conveyor,
        image,
        *inflatedRectanglesVector,
        *inflatedRectanglesIndices,
        intermediatePointsImageVector,
        intermediatePointsSetVector);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createCheckerRequiredVectors");
    #endif
  }



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
  void FiltersResources::createHolesMasksVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<cv::Mat>* holesMasksImageVector,
    std::vector<std::set<unsigned int> >* holesMasksSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksVectors", "createCheckerRequiredVectors");
    #endif

    // Create the masks' set initially
    createHolesMasksSetVector(conveyor, image, holesMasksSetVector);

    // Draw each mask set onto an image
    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      // The current hole's image mask
      cv::Mat holeMask = cv::Mat::zeros(image.size(), CV_8UC1);

      // A pointer to the hole mask image
      unsigned char* ptr = holeMask.ptr();

      // Draw the current hole's mask
      for (std::set<unsigned int>::iterator it =
        (*holesMasksSetVector)[i].begin();
        it != (*holesMasksSetVector)[i].end(); it++)
      {
        ptr[*it] = 255;
      }

      holesMasksImageVector->push_back(holeMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksVectors");
    #endif
  }



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
  void FiltersResources::createHolesMasksImageVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<cv::Mat>* holesMasksImageVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksImageVector", "createCheckerRequiredVectors");
    #endif

    // Create the masks' set initially
    std::vector<std::set<unsigned int> > holesMasksSetVector;
    createHolesMasksSetVector(conveyor, image, &holesMasksSetVector);

    // Draw each mask set onto an image
    for (unsigned int i = 0; i < conveyor.size(); i++)
    {
      // The current hole's image mask
      cv::Mat holeMaskImage = cv::Mat::zeros(image.size(), CV_8UC1);

      // A pointer to the hole mask image
      unsigned char* ptr = holeMaskImage .ptr();

      // Draw the current hole's mask
      for (std::set<unsigned int>::iterator it = holesMasksSetVector[i].begin();
        it != holesMasksSetVector[i].end(); it++)
      {
        ptr[*it] = 255;
      }

      holesMasksImageVector->push_back(holeMaskImage);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksImageVector");
    #endif
  }



  /**
    @brief Some hole checkers require access to a hole's inside points,
    Construct a set of points for each hole, and store all of them in
    a vector
    @param[in] conveyor [const HolesConveyor&] The conveyor of holes
    @param[in] image [const cv::Mat&] An image required to access
    each hole
    @param[out] holesMasksSetVector [std::vector<std::set<unsigned int> >*]
    A vector that holds sets of points;
    each set holds the inside points of each hole
    @return void
   **/
  void FiltersResources::createHolesMasksSetVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    std::vector<std::set<unsigned int> >* holesMasksSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createHolesMasksSetVector", "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < conveyor.size(); i++)
    {
      // The image on which the i-th hole's outline will be drawn
      cv::Mat holeMask = cv::Mat::zeros(image.size(), CV_8UC1);

      // Draw the outline points of the i-th hole onto holeMask
      for (unsigned int j = 0; j < conveyor.holes[i].outline.size(); j++)
      {
        holeMask.at<unsigned char>(
          conveyor.holes[i].outline[j].y,
          conveyor.holes[i].outline[j].x) = 255;
      }

      // The set of indices of points inside the i-th hole's outline
      std::set<unsigned int> holeMaskSet;

      // The point from which the floodfill will begin
      cv::Point2f seedPoint(
        conveyor.holes[i].keypoint.pt.x, conveyor.holes[i].keypoint.pt.y);

      // Fill the inside of the i-th hole
      cv::floodFill(holeMask, seedPoint, cv::Scalar(255));

      // Take a pointer on the mask image
      unsigned char* ptr = holeMask.ptr();

      // The points with non-zero value are the ones inside the i-th hole.
      // Insert them into the desired set.
      for (int rows = 0; rows < holeMask.rows; rows++)
      {
        for (int cols = 0; cols < holeMask.cols; cols++)
        {
          if (ptr[rows * holeMask.cols + cols] != 0)
          {
            holeMaskSet.insert(rows * holeMask.cols + cols);
          }
        }
      }

      holesMasksSetVector->push_back(holeMaskSet);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createHolesMasksSetVector");
    #endif
  }



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
  void FiltersResources::createInflatedRectanglesVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const int& inflationSize,
    std::vector<std::vector<cv::Point2f> >* inflatedRectanglesVector,
    std::vector<int>* inflatedRectanglesIndices)
  {
    #ifdef DEBUG_TIME
    Timer::start("createInflatedRectanglesVector",
      "createCheckerRequiredVectors");
    #endif

    // Store the vertices of the inside-of-image-bounds inflated bounding
    // rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point2f> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (int i = 0; i < conveyor.size(); i++)
    {
      std::vector<cv::Point2f> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.holes[i].keypoint.pt.y;
        key_x = conveyor.holes[i].keypoint.pt.x;

        vert_y = conveyor.holes[i].rectangle[j].y;
        vert_x = conveyor.holes[i].rectangle[j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        // check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < image.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < image.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point2f(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      }  // end for rectangle's points

      // If one or more vertices are out of bounds discard the whole
      // inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        inflatedRectanglesIndices->push_back(i);
        inflatedRectanglesVector->push_back(inflatedVertices);
      }
    }  // end for each hole

    #ifdef DEBUG_TIME
    Timer::tick("createInflatedRectanglesVector");
    #endif
  }



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
    @param[in] inflationSize [const int&] The bounding rectangles
    inflation size in pixels
    A vector that holds the image of the intermediate points between
    a hole's outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
    A vector that holds the intermediate points' indices between a hole's
    outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void FiltersResources::createIntermediateHolesPointsVectors(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
    const std::vector<int>& inflatedRectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsVectors",
      "createCheckerRequiredVectors");
    #endif

    // Create the masks' set initially
    createIntermediateHolesPointsSetVector(
      conveyor,
      image,
      inflatedRectanglesVector,
      inflatedRectanglesIndices,
      intermediatePointsSetVector);

    for (int i = 0; i < inflatedRectanglesVector.size(); i++)
    {
      // The current hole's intermediate points mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      // A pointer to the hole mask image
      unsigned char* ptr = intermediatePointsMask.ptr();

      // Draw the intermediate points' mask
      for (std::set<unsigned int>::iterator it =
        (*intermediatePointsSetVector)[i].begin();
        it != (*intermediatePointsSetVector)[i].end(); it++)
      {
        ptr[*it] = 255;
      }

      intermediatePointsImageVector->push_back(intermediatePointsMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsVectors");
    #endif
  }



  /**
    @brief For each hole, this function finds the points between the hole's
    outline and the rectangle (inflated or not) that corrensponds to it.
    These points are then stored in an image.
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
  void FiltersResources::createIntermediateHolesPointsImageVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
    const std::vector<int>& inflatedRectanglesIndices,
    std::vector<cv::Mat>* intermediatePointsImageVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsImageVector",
      "createCheckerRequiredVectors");
    #endif

    // Create the masks' set initially
    std::vector<std::set<unsigned int> > intermediatePointsSetVector;
    createIntermediateHolesPointsSetVector(
      conveyor,
      image,
      inflatedRectanglesVector,
      inflatedRectanglesIndices,
      &intermediatePointsSetVector);

    for (int i = 0; i < inflatedRectanglesVector.size(); i++)
    {
      // The current hole's intermediate points mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      // A pointer to the hole mask image
      unsigned char* ptr = intermediatePointsMask.ptr();

      // Draw the intermediate points' mask
      for (std::set<unsigned int>::iterator it =
        intermediatePointsSetVector[i].begin();
        it != intermediatePointsSetVector[i].end(); it++)
      {
        ptr[*it] = 255;
      }

      intermediatePointsImageVector->push_back(intermediatePointsMask);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsImageVector");
    #endif
  }



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
    @param[in] inflatedRectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[out] intermediatePointsSetVector [std::vector<std::set<int> >*]
    A vector that holds the intermediate points' indices between a hole's
    outline and its bounding box, for each hole whose identifier
    exists in the @param inflatedRectanglesIndices vector
    @return void
   **/
  void FiltersResources::createIntermediateHolesPointsSetVector(
    const HolesConveyor& conveyor,
    const cv::Mat& image,
    const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
    const std::vector<int>& inflatedRectanglesIndices,
    std::vector<std::set<unsigned int> >* intermediatePointsSetVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("createIntermediateHolesPointsSetVector",
      "createCheckerRequiredVectors");
    #endif

    for (int i = 0; i < inflatedRectanglesVector.size(); i++)
    {
      // The current hole's mask
      cv::Mat intermediatePointsMask = cv::Mat::zeros(image.size(), CV_8UC1);

      // An image whose non-zero value pixels are the ones inside the
      // hole's outline
      cv::Mat holeOutlineFilledImage = cv::Mat::zeros(image.size(), CV_8UC1);


      // Draw the outline of the i-th hole onto holeOutlineFilledImage
      for (unsigned int j = 0;
        j < conveyor.holes[inflatedRectanglesIndices[i]].outline.size(); j++)
      {
        holeOutlineFilledImage.at<unsigned char>(
          conveyor.holes[inflatedRectanglesIndices[i]].outline[j].y,
          conveyor.holes[inflatedRectanglesIndices[i]].outline[j].x) = 255;
      }

      // The set of indices of points inside the i-th hole's outline
      std::set<unsigned int> holeOutlineFilledSet;

      // The brushfire start point is the hole's seedPoint
      cv::Point2f seedPoint(
        conveyor.holes[inflatedRectanglesIndices[i]].keypoint.pt.x,
        conveyor.holes[inflatedRectanglesIndices[i]].keypoint.pt.y);

      // floodFill from the seedPoint to the hole's outline
      // to obtain the points inside it
      cv::floodFill(holeOutlineFilledImage, seedPoint, cv::Scalar(255));

      // Take a pointer on the hole's mask image
      unsigned char* ptr = holeOutlineFilledImage.ptr();

      // The points with non-zero value are the ones inside the i-th hole.
      // Insert them into the desired set.
      for (int rows = 0; rows < holeOutlineFilledImage.rows; rows++)
      {
        for (int cols = 0; cols < holeOutlineFilledImage.cols; cols++)
        {
          if (ptr[rows * holeOutlineFilledImage.cols + cols] != 0)
          {
            holeOutlineFilledSet.insert(
              rows * holeOutlineFilledImage.cols + cols);
          }
        }
      }

      // holeOutlineFilledSet is now constructed


      // An image whose non-zero value pixels are the ones inside the
      // hole's bounding rectangle
      cv::Mat rectangleOutlineFilledImage =
        cv::Mat::zeros(image.size(), CV_8UC1);

      // Draw the bounding rectangle of the i-th hole onto
      // rectangleOutlineFilledImage
      for (unsigned int j = 0; j < inflatedRectanglesVector[i].size(); j++)
      {
        cv::line(rectangleOutlineFilledImage,
          inflatedRectanglesVector[i][j],
          inflatedRectanglesVector[i][(j + 1)
          % inflatedRectanglesVector[i].size()],
          cv::Scalar(255, 0, 0), 1, 8);
      }

      // The set of indices of points inside the i-th hole's
      // bounding rectangle
      std::set<unsigned int> rectangleOutlineFilledSet;

      // floodFill from the seedPoint to the hole's bounding rectangle
      // to obtain the points inside it
      cv::floodFill(rectangleOutlineFilledImage, seedPoint, cv::Scalar(255));

      // Take a pointer on the rectangle's mask image
      ptr = rectangleOutlineFilledImage.ptr();

      // The points with non-zero value are the ones inside the i-th hole.
      // Insert them into the desired set.
      for (int rows = 0; rows < rectangleOutlineFilledImage.rows; rows++)
      {
        for (int cols = 0; cols < rectangleOutlineFilledImage.cols; cols++)
        {
          if (ptr[rows * rectangleOutlineFilledImage.cols + cols] != 0)
          {
            rectangleOutlineFilledSet.insert(
              rows * rectangleOutlineFilledImage.cols + cols);
          }
        }
      }
      // rectangleOutlineFilledSet is now constructed


      // The final set of points' indices
      std::set<unsigned int> intermediatePointsSet;

      std::set_difference(rectangleOutlineFilledSet.begin(),
        rectangleOutlineFilledSet.end(),
        holeOutlineFilledSet.begin(),
        holeOutlineFilledSet.end(),
        std::inserter(intermediatePointsSet, intermediatePointsSet.end()));

      intermediatePointsSetVector->push_back(intermediatePointsSet);
    }

    #ifdef DEBUG_TIME
    Timer::tick("createIntermediateHolesPointsSetVector");
    #endif
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
