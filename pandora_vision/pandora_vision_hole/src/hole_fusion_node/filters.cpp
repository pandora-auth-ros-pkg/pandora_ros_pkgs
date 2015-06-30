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

#include "hole_fusion_node/filters.h"

namespace pandora_vision
{
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
    @param[in] inHistogram [const std::vector<cv::MatND>&]
    The vector of model histograms
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
  void Filters::applyFilter(
    const HolesConveyor& conveyor,
    const int& filteringMethod,
    const cv::Mat& depthImage,
    const cv::Mat& rgbImage,
    const std::vector<cv::MatND>& inHistogram,
    const PointCloudPtr& pointCloud,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<cv::Mat>& holesMasksImageVector,
    const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
    const std::vector<int>& inflatedRectanglesIndices,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    const std::vector<cv::Mat>& intermediatePointsImageVector,
    std::vector<float>* probabilitiesVector,
    std::vector<cv::Mat>* imgs,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyFilter", "applyFilters");
    #endif

    std::string windowMsg;
    std::vector<std::string> finalMsgs;
    std::vector<std::string> msgs_;

    // Initialize structures
    finalMsgs.clear();
    msgs_.clear();

    switch (filteringMethod)
    {
      // Filter #1 (Color homogeneity inside blob)------------------------------
      case 1 :
        {
          RgbFilters::checkHolesColorHomogeneity(
            rgbImage,
            holesMasksImageVector,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Color homogeneity";
          break;
        }
        // Filter #2 (Luminosity difference)------------------------------------
        // Check for luminosity difference between the points that constitute
        // the blob's bounding box and the points inside the blob's outline
      case 2 :
        {
          RgbFilters::checkHolesLuminosityDiff(
            rgbImage,
            holesMasksSetVector,
            intermediatePointsSetVector,
            inflatedRectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Luminosity difference";
          break;
        }
        // Filter #3 (Texture difference)---------------------------------------
      case 3 :
        {
          RgbFilters::checkHolesTextureDiff(
            rgbImage,
            inHistogram,
            holesMasksImageVector,
            intermediatePointsImageVector,
            inflatedRectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Texture difference";
          break;
        }
        // Filter #4 (Back project model histogram)-----------------------------
      case 4 :
        {
          RgbFilters::checkHolesTextureBackProject(
            rgbImage,
            inHistogram,
            holesMasksSetVector,
            intermediatePointsSetVector,
            inflatedRectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Texture back project";
          break;
        }
      // Filter #5 (through difference of depth)--------------------------------
      case 5 :
        {
          DepthFilters::checkHolesDepthDiff(
            depthImage,
            conveyor,
            inflatedRectanglesVector,
            inflatedRectanglesIndices,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Depth difference";
          break;
        }
        // Filter #6------------------------------------------------------------
        // Inflate the bounding boxes by an inflation size.
        // For a blob to be at least a potential hole, all the points that
        // constitute the inflated rectangle should lie on exactly one plane.
      case 6 :
        {
          DepthFilters::checkHolesRectangleEdgesPlaneConstitution(
            depthImage,
            pointCloud,
            inflatedRectanglesVector,
            inflatedRectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Outline of rectangle on plane";
          break;
        }
        // Filter #7 (depth & area comparison)----------------------------------
      case 7 :
        {
          DepthFilters::checkHolesDepthArea(
            conveyor,
            depthImage,
            holesMasksSetVector,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Area / Depth";
          break;
        }
        // Filter #8------------------------------------------------------------
        // Brushfire from blob outline to blob bounding box
        // with an inflation size (inflates the rectangle by x pixels).
        // If the points between the blob's outline and the inflated rectangle
        // lie on one plane, this blob is a hole.
      case 8 :
        {
          DepthFilters::checkHolesOutlineToRectanglePlaneConstitution(
            depthImage,
            pointCloud,
            intermediatePointsSetVector,
            inflatedRectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Points around blob to plane";
          break;
        }
        // Filter #9 (Depth homogeneity)----------------------------------------
        // All holes are considered valid except for those that are edgeless
        // inside the area denoted by the conveyor->outlines points
      case 9 :
        {
          DepthFilters::checkHolesDepthHomogeneity(
            conveyor,
            depthImage,
            holesMasksSetVector,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Depth homogeneity";
          break;
        }
    }

    for (int i = 0; i < conveyor.size(); i++)
    {
      if (msgs_.size() == conveyor.size())
      {
        finalMsgs.push_back(msgs_[i]);
      }
    }

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_check_holes)  // Debug
    {
      std::string msg = LPATH(STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" ") + windowMsg;
      msgs->push_back(msg);

      if (filteringMethod < 6)
      {
        cv::Mat depthFiltersImage;
        depthFiltersImage = Visualization::showHoles(
          windowMsg.c_str(),
          depthImage,
          conveyor,
          -1,
          finalMsgs);

        imgs->push_back(depthFiltersImage);
      }
      else
      {
        cv::Mat rgbFiltersImage;
        rgbFiltersImage = Visualization::showHoles(
          windowMsg.c_str(),
          rgbImage,
          conveyor,
          -1,
          finalMsgs);

        imgs->push_back(rgbFiltersImage);
      }
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("applyFilter");
    #endif
  }



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
    @param[in] inHistogram [const std::vector<cv::MatND>&]
    The vector of model histograms
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
  void Filters::applyFilters(
    const HolesConveyor& conveyor,
    const int& filteringMode,
    const cv::Mat& depthImage,
    const cv::Mat& rgbImage,
    const std::vector<cv::MatND>& inHistogram,
    const PointCloudPtr& pointCloud,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<cv::Mat>& holesMasksImageVector,
    const std::vector<std::vector<cv::Point2f> >& inflatedRectanglesVector,
    const std::vector<int>& inflatedRectanglesIndices,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    const std::vector<cv::Mat>& intermediatePointsImageVector,
    std::vector<std::vector<float> >* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyFilters", "filterHoles");
    #endif

    // A mapping of the filters' execution order to an identifier for each
    // filter
    std::map<int, int> filtersOrder;

    // The filtering mode permission to application of depth analysis
    // condition.
    // Active Depth and RGB filters will both be applied
    if (filteringMode == RGBD_MODE)
    {
      if (Parameters::Filters::ColourHomogeneity::rgbd_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::ColourHomogeneity::rgbd_priority] = 1;
      }

      if (Parameters::Filters::LuminosityDiff::rgbd_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::LuminosityDiff::rgbd_priority] = 2;
      }

      if (Parameters::Filters::TextureDiff::rgbd_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::TextureDiff::rgbd_priority] = 3;
      }

      if (Parameters::Filters::TextureBackprojection::rgbd_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::TextureBackprojection::rgbd_priority] = 4;
      }

      if (Parameters::Filters::DepthDiff::priority > 0)
      {
        filtersOrder[
          Parameters::Filters::DepthDiff::priority] = 5;
      }

      if (Parameters::Filters::RectanglePlaneConstitution::priority > 0)
      {
        filtersOrder[
          Parameters::Filters::RectanglePlaneConstitution::priority] = 6;
      }

      if (Parameters::Filters::DepthArea::priority > 0)
      {
        filtersOrder[
          Parameters::Filters::DepthArea::priority] = 7;
      }

      if (Parameters::Filters::IntermediatePointsPlaneConstitution::priority > 0)
      {
        filtersOrder[
          Parameters::Filters::IntermediatePointsPlaneConstitution::priority] = 8;
      }

      if (Parameters::Filters::DepthHomogeneity::priority > 0)
      {
        filtersOrder[
          Parameters::Filters::DepthHomogeneity::priority] = 9;
      }
    }
    // Depth filtering cannot be applied, hence only RGB filters will
    // be utilized
    else if (filteringMode == RGB_ONLY_MODE)
    {
      if (Parameters::Filters::ColourHomogeneity::rgb_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::ColourHomogeneity::rgb_priority] = 1;
      }

      if (Parameters::Filters::LuminosityDiff::rgb_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::LuminosityDiff::rgb_priority] = 2;
      }

      if (Parameters::Filters::TextureDiff::rgb_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::TextureDiff::rgb_priority] = 3;
      }

      if (Parameters::Filters::TextureBackprojection::rgb_priority > 0)
      {
        filtersOrder[
          Parameters::Filters::TextureBackprojection::rgb_priority] = 4;
      }
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME, "[Hole Fusion node] Filtering process failure");
    }

    // Debugging images and messages of validity probabilities
    // per candidate hole
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;

    // Apply each active filter, depending on the interpolation method
    int counter = 0;
    for (std::map<int, int>::iterator o_it = filtersOrder.begin();
      o_it != filtersOrder.end(); ++o_it)
    {
      applyFilter(
        conveyor,
        o_it->second,
        depthImage,
        rgbImage,
        inHistogram,
        pointCloud,
        holesMasksSetVector,
        holesMasksImageVector,
        inflatedRectanglesVector,
        inflatedRectanglesIndices,
        intermediatePointsSetVector,
        intermediatePointsImageVector,
        &probabilitiesVector->at(counter),
        &imgs,
        &msgs);

      counter++;
    }  // o_it iterator ends

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_check_holes)  // Debug
    {
      Visualization::multipleShow("CheckHoles function", imgs, msgs,
        Parameters::Debug::show_check_holes_size, 1);
    }
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("applyFilters");
    #endif
  }

}  // namespace pandora_vision
