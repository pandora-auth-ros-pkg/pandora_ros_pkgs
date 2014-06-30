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

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
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
  void RgbFilters::checkHolesColorHomogeneity(
    const cv::Mat& inImage,
    const std::vector<cv::Mat>& holesMasksImageVector,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesColorHomogeneity", "applyFilter");
    #endif

    // Copy the input image to inImage_ so as to get a pointer on it
    cv::Mat inImage_;

    inImage.copyTo(inImage_);

    // Reduce the colours of inImage_.
    // For a value of 16 for div, the maximum possible number of colours
    // is 2^12. For every duplication of div, the number of possible colours
    // is divided by a factor of 2^3
    int div = 16;

    for (int i = 0; i < inImage_.rows; i++)
    {
      // Get the address of row i
      unsigned char* data = inImage_.ptr<unsigned char>(i);

      for (int j = 0; j < inImage_.cols * inImage_.channels(); j++)
      {
        // Process each pixel
        data[j] = data[j] / div * div + div / 2;
      }
    }


    for (unsigned int i = 0; i < holesMasksImageVector.size(); i++)
    {
      // Sets featuring all the different colours found inside each image mask
      std::set<unsigned char> blueColourSet;
      std::set<unsigned char> greenColourSet;
      std::set<unsigned char> redColourSet;

      for (int rows = 0; rows < inImage_.rows; rows++)
      {
        for (int cols = 0; cols < inImage_.cols; cols++)
        {
          // Collect the different values of colour components for the
          // points of the current mask
          if (holesMasksImageVector[i].at<unsigned char>(rows, cols) != 0)
          {
            blueColourSet.insert(inImage_.at<cv::Vec3b>( rows, cols ).val[0]);
            greenColourSet.insert(inImage_.at<cv::Vec3b>( rows, cols ).val[1]);
            redColourSet.insert(inImage_.at<cv::Vec3b>( rows, cols ).val[2]);
          }
        }
      }

      // The number of distinct colours inside the mask
      int numberOfColours =
        blueColourSet.size() * greenColourSet.size() * redColourSet.size();

      // Threshold the number of colours
      if (numberOfColours < 1024)
      {
        numberOfColours = 0;
      }

      probabilitiesVector->at(i) = static_cast<float>(numberOfColours) / 4096;

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
    @param[in] inImage [const cv::Mat&] The RGB image in CV_8UC3 format
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
      float meanBlobLuminosity = 0.0;
      if (holesMasksSetVector[i].size() > 0)
      {
        meanBlobLuminosity = static_cast<float> (blobLuminosity)
          / holesMasksSetVector[i].size();
      }


      // Mean luminosity of the intermediate points
      float meanBoundingBoxLuminosity = 0.0;
      if (intermediatePointsSetVector[i].size() > 0)
      {
        meanBoundingBoxLuminosity = static_cast<float> (boundingBoxLuminosity)
          / intermediatePointsSetVector[i].size();
      }


      // If the luminosity of the inside of the candidate hole is greater
      // than the luminosity of the points beyond it and restricted by the
      // edges of its bounding box, it surely is not a hole
      if (meanBlobLuminosity < meanBoundingBoxLuminosity
        && meanBoundingBoxLuminosity > 0)
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
    bounding box points, and a model histogram, this filter creates the
    back project of the @param inImage based on @param inHistogram and
    exports a vector of probabilities, that is a vector of how probable it
    is for a candidate hole's points between the blob's outline points
    and the bounding box's edges to have a high probability
    in the back project image, and for the points inside the candidate
    hole's outline to have a low probability in the back project image
    @param[in] inImage [const cv::Mat&] The input RGB image in CV_8UC3 format
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

    // Obtain the backprojection of the inImage, according to the inHistogram
    cv::MatND backProject;
    Histogram::getBackprojection(inImage, inHistogram,
      &backProject, Parameters::Histogram::secondary_channel);

    // Obtain a homogenous backprojection image
    cv::Mat watersheded;
    EdgeDetection::watershedViaBackprojection(inImage, backProject, false,
      &watersheded);

    // Obtain a pointer on watersheded
    unsigned char* ptr = watersheded.ptr();

    // For each inflated rectangle, calculate the probabilities of
    // (1) the points between the blob's outline and the edges of the
    // inflated rectangle and
    // (2) the points inside the blob's outline
    // based on the backProjection cv::MatND
    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      float blobSum = 0.0;
      for (std::set<unsigned int>::iterator h_it = holesMasksSetVector[i].begin();
        h_it != holesMasksSetVector[i].end(); h_it++)
      {
        blobSum += static_cast<float>(ptr[*h_it]) / 255;
      }

      float blobToRectangleSum = 0.0;
      for (std::set<unsigned int>::iterator i_it =
        intermediatePointsSetVector[i].begin();
        i_it != intermediatePointsSetVector[i].end(); i_it++)
      {
        blobToRectangleSum += static_cast<float>(ptr[*i_it]) / 255;
      }

      // The average probability of the points consisting the inflated
      // rectangle matching the inHistogram
      float rectangleMatchProbability =
        blobToRectangleSum / intermediatePointsSetVector[i].size();

      // The average probability of the points inside the blob's outline
      // matching the inHistogram
      float blobMatchProbability = blobSum / holesMasksSetVector[i].size();

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

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesTextureBackProject");
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
    @param[in] inImage [const cv::Mat&] The input RGB image in CV_8UC3 format
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

    // inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage, inImageHSV, cv::COLOR_BGR2HSV);

    int* histSize = new int[2];

    // The first value will always be with regard to Hue
    histSize[0] = Parameters::Histogram::number_of_hue_bins;

    // Histogram-related parameters
    int v_bins = Parameters::Histogram::number_of_value_bins;

    if (Parameters::Histogram::secondary_channel == 1)
    {
      histSize[1] = Parameters::Histogram::number_of_saturation_bins;
    }

    if (Parameters::Histogram::secondary_channel == 2)
    {
      histSize[1] = Parameters::Histogram::number_of_value_bins;
    }

    // hue varies from 0 to 179, saturation or value from 0 to 255
    float h_ranges[] = { 0, 180 };
    float sec_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, sec_ranges };

    // Use the 0-th and secondaryChannel-st channels
    int channels[] = { 0, Parameters::Histogram::secondary_channel };


    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      // Produce the histogram for the points in between the blob's outline
      // and the inflated rectangle's edges
      cv::MatND blobToRectangleHistogram;
      cv::calcHist(&inImageHSV, 1, channels, intermediatePointsImageVector[i],
        blobToRectangleHistogram, 2, histSize, ranges, true, false);


      // Produce the histogram for the points inside the outline of the blob
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels,
        holesMasksImageVector[rectanglesIndices[i]],
        blobHistogram, 2, histSize, ranges, true, false);


      // Find the correlation between the model histogram and the histogram
      // of the inflated rectangle
      double rectangleToModelCorrelation = cv::compareHist(
        blobToRectangleHistogram, inHistogram, CV_COMP_HELLINGER);

      // Find the correlation between the model histogram and the histogram
      // of the points inside the blob
      double blobToModelCorrelation = cv::compareHist(
        blobHistogram, inHistogram, CV_COMP_HELLINGER);

      // This blob is considered valid if there is a correlation between
      // the histogram of the external to the hole's outline points
      // and the model histogram (inHistogram) greater than a threshold and,
      // simultaneously, the correlation between the histogram of the points
      // inside the hole's outline points and the model histogram is lower than
      // a threshold.
      // CAUTION: The use of the CV_COMP_HELLINGER for histogram comparison
      // inverts the inequality checks
      if (rectangleToModelCorrelation <=
        Parameters::HoleFusion::match_texture_threshold &&
        blobToModelCorrelation >=
        Parameters::HoleFusion::mismatch_texture_threshold)
      {
        probabilitiesVector->at(rectanglesIndices[i]) =
          blobToModelCorrelation - rectangleToModelCorrelation;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesTextureDiff");
    #endif
  }

} // namespace pandora_vision
