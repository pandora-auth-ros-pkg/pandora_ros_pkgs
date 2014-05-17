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

#include "hole_fusion_node/rgb_filters.h"

namespace pandora_vision
{
  /**
    @brief Checks for color homogenity in a region where points are
    constrained inside each hole. A candidate hole is considered valid
    if its H-V histogram has above a certain number of bins occupied.
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inImage [const cv::Mat&] The RGB image in unscaled format
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
  void RgbFilters::checkHolesColorHomogeneity(
    const HolesConveyor& conveyor,
    const cv::Mat& inImage,
    const std::vector<cv::Mat>& holesMasksImageVector,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesColorHomogeneity", "applyFilter");
    #endif

    // Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    // inImage_ transformed from BGR format to HSV
    cv::Mat inImageHSV = cv::Mat::zeros(inImage_.size(), CV_8UC3);
    cv::cvtColor(inImage_, inImageHSV, cv::COLOR_BGR2HSV);

    // Histogram-related parameters
    int h_bins = 180;
    int v_bins = 256;
    int histSize[] = { h_bins, v_bins };

    // hue varies from 0 to 179, saturation or value from 0 to 255
    float h_ranges[] = { 0, 180 };
    float v_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, v_ranges };

    // Use the 0-th and 2-nd channels - H and V
    int channels[] = { 0, 2 };

    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      // Calculate the blob's histogram
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, holesMasksImageVector[i],
        blobHistogram, 2, histSize, ranges, true, false);


      // Break the h_bins X v_bins image into boxes of box_x X box_y
      // (vertically by horizontally).
      // Measure how many non-zero points there are in each
      // box. If there are more than a threshold value, count that box as
      // an overall non zero box
      int box_x = 20;
      int box_y = 16;

      int overallNonZeroBoxes = 0;
      for (unsigned int rows = 0; rows < h_bins / box_x; rows++)
      {
        for (unsigned int cols = 0; cols < v_bins / box_y; cols++)
        {
          int nonZeroInBox = 0;
          for (unsigned int b_x = 0; b_x < box_x; b_x++)
          {
            for (unsigned int b_y = 0; b_y < box_y; b_y++)
            {
              if (blobHistogram.at<unsigned char>(
                  rows * box_x + b_x, cols * box_y + b_y) != 0)
              {
                nonZeroInBox++;
              }
            }
          }

          if (nonZeroInBox >
            Parameters::HoleFusion::non_zero_points_in_box_blob_histogram)
          {
            overallNonZeroBoxes++;
          }
        }
      }

      probabilitiesVector->at(i) =
        static_cast<float> (overallNonZeroBoxes)
        / ((static_cast<float>(h_bins) / box_x)
        * (static_cast<float>(v_bins) / box_y));

      msgs->push_back(TOSTR(probabilitiesVector->at(i)));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesColorHomogeneity");
    #endif
  }



  /**
    @brief Checks for difference in mean value of luminosity between
    (1) the pixels in between a hole's bounding box edges and the points
    outside the hole's outline and
    (2) the points inside the hole's outline.
    @param[in] conveyor [const blobHolesConveyor&] The candidate holes
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points's indices;
    each point is internal to its respective hole
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points' indices; these points are the points between
    the hole's outline and its bounding rectangle
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
  void RgbFilters::checkHolesLuminosityDiff(
    const HolesConveyor& conveyor,
    const cv::Mat& inImage,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesLuminosityDiff", "applyFilter");
    #endif

    // Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    // Instead of applying the formula
    // Y = 0.299 * R + 0.587 * G + 0.114 * B to find the luminosity of each
    // pixel, turn inImage into grayscale
    cv::Mat luminosityImage(inImage.size(), CV_8UC1);
    cv::cvtColor(inImage, luminosityImage, CV_BGR2GRAY);

    unsigned char* ptr = luminosityImage.ptr();

    // For each inflated rectangle, calculate the luminosity of
    // (1) the points between the blob's outline and the edges of the
    // inflated rectangle and
    // (2) the points inside the blob's outline
    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      // The current hole's inside points luminosity sum
      int blobLuminosity = 0;
      for (std::set<unsigned int>::iterator h_it = holesMasksSetVector[i].begin();
        h_it != holesMasksSetVector[i].end(); h_it++)
      {
        blobLuminosity += ptr[*h_it];
      }

      // The current hole's intermediate points luminosity sum
      int boundingBoxLuminosity = 0;
      for (std::set<unsigned int>::iterator i_it =
        intermediatePointsSetVector[i].begin();
        i_it != intermediatePointsSetVector[i].end(); i_it++)
      {
        boundingBoxLuminosity += ptr[*i_it];
      }

      // Mean luminosity of the inside points of the current hole
      float meanBlobLuminosity = static_cast<float> (blobLuminosity)
        / holesMasksSetVector[i].size();

      // Mean luminosity of the intermediate points
      float meanBoundingBoxLuminosity =
        static_cast<float> (boundingBoxLuminosity)
        / intermediatePointsSetVector[i].size();



      // If the luminosity of the inside of the candidate hole is greater
      // than the luminosity of the points beyond it and restricted by the
      // edges of its bounding box, it surely is not a hole
      if (meanBlobLuminosity > meanBoundingBoxLuminosity)
      {
        probabilitiesVector->at(rectanglesIndices[i]) = 0.0;
      }
      else
      {
        probabilitiesVector->at(rectanglesIndices[i]) =
          1 - meanBlobLuminosity / meanBoundingBoxLuminosity;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesLuminosityDiff");
    #endif
  }



  /**
    @brief Given a set of keypoints, their respective outline and
    bounding box points, and a model histogram, this filter looks for near
    equation between the histograms of the points between the blob's
    outline and the bounding box's edges and the model histogram,
    and for major difference between the
    histograms of the bounding box and the points inside the outline of the
    blob.
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inImage [const cv::Mat&] The input RGB image in unscaled
    format
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
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
    @param[in,out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void RgbFilters::checkHolesTextureDiff(
    const HolesConveyor& conveyor,
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const std::vector<cv::Mat>& holesMasksImageVector,
    const std::vector<cv::Mat>& intermediatePointsImageVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesTextureDiff", "applyFilter");
    #endif

    // Since not all rectangles may make it, store the indices
    // of the original keypoints that correspond to valid inflated rectangles
    // in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;


    // Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    // inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage_, inImageHSV, cv::COLOR_BGR2HSV);

    // Histogram-related parameters
    int h_bins = Parameters::Histogram::number_of_hue_bins;
    int s_bins = Parameters::Histogram::number_of_saturation_bins;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the 0-th and 2-nd channels
    int channels[] = { 0, 2 };


    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      // Produce the histogram for the points in between the blob's outline
      // and the inflated rectangle's edges
      cv::MatND blobToRectangleHistogram;
      cv::calcHist(&inImageHSV, 1, channels, intermediatePointsImageVector[i],
        blobToRectangleHistogram, 2, histSize, ranges, true, false);


      // Produce the histogram for the points inside the outline of the blob
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, holesMasksImageVector[i],
        blobHistogram, 2, histSize, ranges, true, false);


      // Find the correlation between the model histogram and the histogram
      // of the inflated rectangle
      double rectangleToModelCorrelation = cv::compareHist(
        blobToRectangleHistogram, inHistogram, CV_COMP_CORREL);

      // Find the correlation between the model histogram and the histogram
      // of the points inside the blob
      double blobToModelCorrelation = cv::compareHist(
        blobHistogram, inHistogram, CV_COMP_CORREL);

      //ROS_ERROR("R2M: %f", static_cast<float> (rectangleToModelCorrelation));
      //ROS_ERROR("B2M: %f", static_cast<float> (blobToModelCorrelation));

      // This blob is considered valid if there is a correlation between
      // blobToRectangleHistogram and the model histogram
      // (inHistogram)
      // greater than a threshold and, simultaneously, the blob's histogram
      // is more loosely correlated to the model histogram than the
      // blobToRectangleHistogram is
      if (rectangleToModelCorrelation >=
        Parameters::HoleFusion::match_texture_threshold &&
        rectangleToModelCorrelation > blobToModelCorrelation)
      {
        probabilitiesVector->at(rectanglesIndices[i]) =
          rectangleToModelCorrelation - blobToModelCorrelation;
      }
      else
      {
        probabilitiesVector->at(rectanglesIndices[i]) = 0.0;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesTextureDiff");
    #endif
  }



  /**
    @brief Given a set of keypoints, their respective outline and
    bounding box points, and a model histogram, this filter creates the
    back project of the @param inImage based on @param inHistogram and
    exports a vector of probabilities, that is a vector of how probable it
    is for a candidate hole's points between the blob's outline points
    and the bounding box's edges to have a high probability
    in the back project image, and for the points inside the candidate
    hole's outline to have a low probability in the back project image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inImage [const cv::Mat&] The input RGB image in unscaled format
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points; each point is internal to its
    respective hole
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points' indices; these points are the points between
    the hole's outline and its bounding rectangle
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @param[in,out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void RgbFilters::checkHolesTextureBackProject(
    const HolesConveyor& conveyor,
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesTextureBackProject", "applyFilter");
    #endif

    // Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::Image::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    // Obtain the backprojection of the inImage_, according to the inHistogram
    cv::MatND backProject;
    Histogram::getBackprojection(inImage_, inHistogram,
      &backProject, Parameters::Histogram::secondary_channel);

    // Because the backproject is often sparcely populated, dilate each
    // zero pixel with non-zero neighours to obtain a more accurate depiction
    // of the backproject image
    Morphology::dilationRelative(&backProject, 3);

    #ifdef DEBUG_SHOW
    //Visualization::show("backProject", backProject, 1);
    #endif

    unsigned char* ptr = backProject.ptr();

    // For each inflated rectangle, calculate the probabilities of
    // (1) the points between the blob's outline and the edges of the
    // inflated rectangle and
    // (2) the points inside the blob's outline
    // based on the backProjection cv::MatND
    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      float blobSum = 0;
      for (std::set<unsigned int>::iterator h_it = holesMasksSetVector[i].begin();
        h_it != holesMasksSetVector[i].end(); h_it++)
      {
        blobSum += ptr[*h_it];
      }

      float blobToRectangleSum = 0;
      for (std::set<unsigned int>::iterator i_it =
        intermediatePointsSetVector[i].begin();
        i_it != intermediatePointsSetVector[i].end(); i_it++)
      {
        blobToRectangleSum += ptr[*i_it];
      }

      // The average probability of the points consisting the inflated
      // rectangle matching the inHistogram
      float rectangleMatchProbability =
        blobToRectangleSum / holesMasksSetVector[i].size() / 255;

      // The average probability of the points inside the blob's outline
      // matching the inHistogram
      float blobMatchProbability =
        blobSum / intermediatePointsSetVector[i].size() / 255;

      // This blob is considered valid, with a non zero validity probability,
      // if the points consisting the inflated rectangle have a greater
      // resemblance (through the probability-expressing values of the
      // back project cv::MatND) to the inHistogram than the one of the points
      // inside the blob's outline
      if (rectangleMatchProbability > blobMatchProbability)
      {
        probabilitiesVector->at(rectanglesIndices[i]) =
          rectangleMatchProbability - blobMatchProbability;
      }
      else
      {
        probabilitiesVector->at(rectanglesIndices[i]) = 0.0;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesTextureBackProject");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is
    attached to an order which relates to the sequence of the overall
    filter execution.
    @param[in] conveyor [const HolesConveyor&] A struct that
    contains the final valid holes
    @param[in] rgbImage [const cv::Mat&] The input rgb image
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] holesMasksImageVector [const std::vector<cv::Mat>&]
    A vector containing masks of the points inside each hole's outline
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points's indices;
    each point is internal to its respective hole
    @param[in] intermediatePointsImageVector [const std::vector<cv::Mat>&]
    A vector containing masks of the points outside each hole's outline,
    but inside its bounding rectangle
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points' indices; these points are the points between
    the hole's outline and its bounding rectangle
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
  void RgbFilters::checkHoles(
    const HolesConveyor& conveyor,
    const cv::Mat& rgbImage,
    const cv::MatND& inHistogram,
    const std::vector<int>& rectanglesIndices,
    const std::vector<cv::Mat>& holesMasksImageVector,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<cv::Mat>& intermediatePointsImageVector,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    std::vector<std::vector<float> >* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles", "sift");
    #endif

    std::set<unsigned int> indexes;
    std::vector<std::string> finalMsgs;

    std::map<int, int> filtersOrder;

    if (Parameters::HoleFusion::run_checker_color_homogeneity > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_color_homogeneity] = 1;
    }
    if (Parameters::HoleFusion::run_checker_luminosity_diff > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_luminosity_diff] = 2;
    }
    if (Parameters::HoleFusion::run_checker_texture_diff > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_texture_diff] = 3;
    }
    if (Parameters::HoleFusion::run_checker_texture_backproject > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_texture_backproject] = 4;
    }

    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;

    int counter = 0;
    for (std::map<int, int>::iterator o_it = filtersOrder.begin();
      o_it != filtersOrder.end(); ++o_it)
    {
      applyFilter(
        o_it->second,
        rgbImage,
        conveyor,
        inHistogram,
        rectanglesIndices,
        holesMasksImageVector,
        holesMasksSetVector,
        intermediatePointsImageVector,
        intermediatePointsSetVector,
        &probabilitiesVector->at(counter),
        &imgs,
        &msgs);

      counter++;
    } // o_it iterator ends

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_check_holes) // Debug
    {
      Visualization::multipleShow("rgb checkHoles functions",
        imgs, msgs, 1200, 1);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("checkHoles");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is attached
    to an order which relates to the sequence of the overall filter execution.
    @param[in] method [const int&] The filter identifier to execute
    @param[in] img [const cv::Mat&] The input rgb image
    @param[in] conveyor [const HolesConveyor&] The structure that
    holds the final holes' data
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] holesMasksImageVector [const std::vector<cv::Mat>&]
    A vector containing masks of the points inside each hole's outline
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points's indices;
    each point is internal to its respective hole
    @param[in] intermediatePointsImageVector [const std::vector<cv::Mat>&]
    A vector containing masks of the points outside each hole's outline,
    but inside its bounding rectangle
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points' indices; these points are the points between
    the hole's outline and its bounding rectangle
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree with which each
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node.
    @param[in,out] imgs [std::vector<cv::Mat>*] A vector of images which shows
    the holes that are considered valid by each filter
    @param[in,out] msgs [std::vector<std::string>*] Debug messages
    @return void
   **/
  void RgbFilters::applyFilter(
    const int& method,
    const cv::Mat& img,
    const HolesConveyor& conveyor,
    const cv::MatND& inHistogram,
    const std::vector<int>& rectanglesIndices,
    const std::vector<cv::Mat>& holesMasksImageVector,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<cv::Mat>& intermediatePointsImageVector,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    std::vector<float>* probabilitiesVector,
    std::vector<cv::Mat>* imgs,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyFilter", "checkHoles");
    #endif

    std::string windowMsg;
    std::vector<std::string> finalMsgs;
    std::vector<std::string> msgs_;

    // Initialize structures
    finalMsgs.clear();
    msgs_.clear();

    switch(method)
    {
      // Filter #1 (Color homogenity inside blob)-----------------------------
      case 1 :
        {
          checkHolesColorHomogeneity(
            conveyor,
            img,
            holesMasksImageVector,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Color homogeneity";
          break;
        }
        // Filter #2 (Luminosity difference)----------------------------------
        // Check for luminosity difference between the points that constitute
        // the blob's bounding box and the points inside the blob's outline
      case 2 :
        {
          checkHolesLuminosityDiff(
            conveyor,
            img,
            holesMasksSetVector,
            intermediatePointsSetVector,
            rectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Luminosity difference";
          break;
        }
        // Filter #3 (Texture difference)-------------------------------------
      case 3 :
        {
          checkHolesTextureDiff(
            conveyor,
            img,
            inHistogram,
            holesMasksImageVector,
            intermediatePointsImageVector,
            rectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Texture difference";
          break;
        }
        // Filter #4 (Back project model histogram)---------------------------
      case 4 :
        {
          checkHolesTextureBackProject(
            conveyor,
            img,
            inHistogram,
            holesMasksSetVector,
            intermediatePointsSetVector,
            rectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Texture back project";
          break;
        }
    }

    for(int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      if(msgs_.size() == conveyor.keyPoints.size())
      {
        finalMsgs.push_back(msgs_[i]);
      }
    }


    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_check_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" ") + windowMsg;
      msgs->push_back(msg);

      cv::Mat tmp;
      tmp = Visualization::showHoles(
        windowMsg.c_str(),
        img,
        conveyor,
        -1,
        finalMsgs);

      imgs->push_back(tmp);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("applyFilter");
    #endif
  }

} // namespace pandora_vision
