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

#include "hole_fusion_node/rgb_filters.h"

namespace pandora_vision
{

  /**
    @brief Checks for color homogenity in a region where points are
    constrained inside each @param inOutlines's elements. A candidate hole
    is considered valid if its H-V histogram has above a certain
    number of bins occupied.
    @param[in] inImage [const cv::Mat&] The RGB image in unscaled format
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void RgbFilters::checkHolesColorHomogenity(
    const cv::Mat& inImage,
    const HolesConveyor& conveyor,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesColorHomogenity", "applyFilter");
    #endif

    //!< Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    //!< inImage_ transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage_, inImageHSV, cv::COLOR_BGR2HSV);

    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      //!< Create the mask needed for the histogram of the
      //!< points inside this blobs'outline
      cv::Mat blobMask = cv::Mat::zeros(inImage.size(), CV_8UC1);

      //!< Draw the points inside the blob
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (cv::pointPolygonTest(
              conveyor.outlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobMask.at<unsigned char>(rows, cols) = 255;
          }
        }
      }
      //!< Blob mask is now ready


      //!< Histogram-related parameters
      int h_bins = 180;
      int v_bins = 256;
      int histSize[] = { h_bins, v_bins };

      //!< hue varies from 0 to 179, saturation or value from 0 to 255
      float h_ranges[] = { 0, 180 };
      float v_ranges[] = { 0, 256 };

      const float* ranges[] = { h_ranges, v_ranges };

      //!< Use the 0-th and 2-nd channels - H and V
      int channels[] = { 0, 2 };


      //!< Calculate the blob's histogram
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, blobMask, blobHistogram,
        2, histSize, ranges, true, false);

      /*
       *cv::imshow("blobHistogram", blobHistogram);
       *cv::waitKey(1);
       */

      //!< Break the h_bins X v_bins image into boxes of box_x X box_y
      //!< (vertically by horizontally).
      //!< Measure how many non-zero points there are in each
      //!< box. If there are more than a threshold value, count that box as
      //!< an overall non zero box
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
            Parameters::non_zero_points_in_box_blob_histogram)
          {
            overallNonZeroBoxes++;
          }
        }
      }

      probabilitiesVector->at(i) = static_cast<float> (overallNonZeroBoxes) /
        (h_bins / box_x * v_bins / box_y);

      msgs->push_back(TOSTR(probabilitiesVector->at(i)));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesColorHomogenity");
    #endif
  }



  /**
    @brief Checks for difference in mean value of luminosity between
    (1) the pixels in between the blob's bounding box edges and the points
    outside the blob's outline and
    (2) the points inside the blob's outline.
    @param[in] inImage [const cv::Mat&] The RGB image in unscaled format
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inflationSize [cosnt int&] grow the rectangle by
    inflationSize as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for
    debug reasons
    @return void
   **/
  void RgbFilters::checkHolesLuminosityDiff(
    const cv::Mat& inImage,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesLuminosityDiff", "applyFilter");
    #endif

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;

    //!< Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        probabilitiesVector->at(i) = 0.0;
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles


    //!< For each inflated rectangle, calculate the luminosity of
    //!< (1) the points between the blob's outline and the edges of the
    //!< inflated rectangle and
    //!< (2) the points inside the blob's outline
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< Instead of applying the formula
      //!< Y = 0.299 * R + 0.587 * G + 0.114 * B to find the luminosity of each
      //!< pixel, turn inImage_ into grayscale
      cv::Mat luminosityImage(inImage_.size(), CV_8UC1);
      cv::cvtColor(inImage_, luminosityImage, CV_BGR2GRAY);

      int boundingBoxLuminosity = 0;
      int boundingBoxDivisor = 0;
      int blobLuminosity = 0;
      int blobDivisor = 0;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          //!< Mean outline luminosity test
          if (cv::pointPolygonTest(
              conveyor.outlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobLuminosity +=
              (uint8_t)luminosityImage.at<unsigned char>(rows, cols);
            blobDivisor += 1;
          }
          else if (cv::pointPolygonTest(
              inflatedRectangles[i], cv::Point(cols, rows), false) > 0)
          {
            boundingBoxLuminosity +=
              (uint8_t)luminosityImage.at<unsigned char>(rows, cols);
            boundingBoxDivisor += 1;
          }
        }
      }

      //!< Mean luminosity of the points that the inflated rectangle is
      //!< consisted of, derived from the original bounding box of the blob
      float meanBoundingBoxLuminosity =
        static_cast<float> (boundingBoxLuminosity) / boundingBoxDivisor / 255;

      //!< Mean luminosity of the entire blob
      float meanBlobLuminosity =
        static_cast<float> (blobLuminosity) / blobDivisor / 255;


      //!< If the luminosity of the inside of the candidate hole is greater
      //!< than the luminosity of the points beyond it and restricted by the
      //!< edges of its bounding box, it surely is not a hole
      if (meanBlobLuminosity > meanBoundingBoxLuminosity)
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) = 0.0;
      }
      else
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) =
          1 - meanBlobLuminosity / meanBoundingBoxLuminosity;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(validKeyPointsIndices[i])));
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
    @param[in] inImage [const cv::Mat&] The input RGB image in unscaled format
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inflationSize [cosnt int&] grow the rectangle by
    inflationSize as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void RgbFilters::checkHolesTextureDiff(
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesTextureDiff", "applyFilter");
    #endif

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;


    //!< Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    //!< inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage_, inImageHSV, cv::COLOR_BGR2HSV);


    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        probabilitiesVector->at(i) = 0.0;
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles


    //!< For each inflated rectangle, calculate the non-zero masks of
    //!< (1) the points between the blob's outline and the edges of the
    //!< inflated rectangle and
    //!< (2) the points inside the blob's outline
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< Create the masks needed for the histograms of the outline points
      //!< and the points inside the blobs'outline
      cv::Mat blobToRectangleMask = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::Mat blobMask = cv::Mat::zeros(inImage.size(), CV_8UC1);


      //!< Draw the points inside the blob (blobMask)
      //!< and the points between the blob the
      //!< inflated rectangle that corresponds to it (blobToRectangleMask)
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (cv::pointPolygonTest(
              conveyor.outlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobMask.at<unsigned char>(rows, cols) = 255;
          }
          else if(cv::pointPolygonTest(
              inflatedRectangles[i], cv::Point(cols, rows), false) > 0)
          {
            blobToRectangleMask.at<unsigned char>(rows, cols) = 255;
          }
        }
      }
      //!< Masks are now ready


      //!< Histogram-related parameters
      int h_bins = Parameters::number_of_hue_bins;
      int s_bins = Parameters::number_of_saturation_bins;
      int histSize[] = { h_bins, s_bins };

      //!< hue varies from 0 to 179, saturation from 0 to 255
      float h_ranges[] = { 0, 180 };
      float s_ranges[] = { 0, 256 };

      const float* ranges[] = { h_ranges, s_ranges };

      //!< Use the 0-th and 2-nd channels
      int channels[] = { 0, 2 };


      //!< Produce the histogram for the points in between the blob's outline
      //!< and the inflated rectangle's edges
      cv::MatND blobToRectangleHistogram;
      cv::calcHist(&inImageHSV, 1, channels, blobToRectangleMask,
        blobToRectangleHistogram, 2, histSize, ranges, true, false);


      //!< Produce the histogram for the points inside the outline of the blob
      cv::MatND blobHistogram;
      cv::calcHist(&inImageHSV, 1, channels, blobMask,
        blobHistogram, 2, histSize, ranges, true, false);


      //!< Find the correlation between the model histogram and the histogram
      //!< of the inflated rectangle
      double rectangleToModelCorrelation= cv::compareHist(
        blobToRectangleHistogram, inHistogram, CV_COMP_CORREL);

      //!< Find the correlation between the model histogram and the histogram
      //!< of the points inside the blob
      double blobToModelCorrelation = cv::compareHist(
        blobHistogram, inHistogram, CV_COMP_CORREL);

      //ROS_ERROR("R2M: %f", static_cast<float> (rectangleToModelCorrelation));
      //ROS_ERROR("B2M: %f", static_cast<float> (blobToModelCorrelation));

      //!< This blob is considered valid if there is a correlation between
      //!< blobToRectangleHistogram and the model histogram
      //!< (inHistogram)
      //!< greater than a threshold and, simultaneously, the blob's histogram
      //!< is more loosely correlated to the model histogram than the
      //!< blobToRectangleHistogram is
      if (rectangleToModelCorrelation >=
        Parameters::match_texture_threshold &&
        rectangleToModelCorrelation > blobToModelCorrelation)
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) =
          rectangleToModelCorrelation - blobToModelCorrelation;
      }
      else
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) = 0.0;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(validKeyPointsIndices[i])));
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
    @param[in] inImage [const cv::Mat&] The input RGB image in unscaled format
    @param[in] inHistogram [const cv::MatND&]
    The model histogram's H and S component
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inflationSize [cosnt int&] grow the rectangle by
    inflationSize as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree which with the
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void RgbFilters::checkHolesTextureBackProject(
    const cv::Mat& inImage,
    const cv::MatND& inHistogram,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesTextureBackProject", "applyFilter");
    #endif

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;

    //!< Scale the inImage in [0, 255] into inImage_ if not already
    cv::Mat inImage_;
    if (inImage.type() != CV_8UC3 || inImage.type() != CV_8UC1)
    {
      inImage_ = Visualization::scaleImageForVisualization(inImage,
        Parameters::scale_method);
    }
    else
    {
      inImage.copyTo(inImage_);
    }

    //!< inImage transformed from BGR format to HSV
    cv::Mat inImageHSV;
    cv::cvtColor(inImage_, inImageHSV, cv::COLOR_BGR2HSV);

    //!< Histogram-related parameters
    //!< hue varies from 0 to 179, saturation from 0 to 255
    float h_ranges[] = { 0, 180 };
    float v_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, v_ranges };

    //!< Use the 0-th and 1-st channels
    int channels[] = { 0, 2 };

    //!< Calulate the inImageHSV's back project.
    //!< We will use it to find a mean probability for inHistogram's occurence
    //!< in the points consiting the inflated rectangle (below) and the points
    //!< inside the outline
    cv::MatND backProject;
    cv::calcBackProject(&inImageHSV, 1, channels, inHistogram, backProject,
      ranges, 1, true);

    #ifdef DEBUG_SHOW
      //Visualization::show("backProject", backProject, 1);
    #endif

    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        probabilitiesVector->at(i) = 0.0;
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles


    //!< For each inflated rectangle, calculate the probabilities of
    //!< (1) the points between the blob's outline and the edges of the
    //!< inflated rectangle and
    //!< (2) the points inside the blob's outline
    //!< based on the backProjection cv::MatND
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      float blobToRectangleSum  = 0;
      int blobToRectanglePoints  = 0;
      float blobSum = 0;
      int blobPoints = 0;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (cv::pointPolygonTest(
              conveyor.outlines[i], cv::Point(cols, rows), false) > 0)
          {
            blobSum +=
              (uint8_t)backProject.at<unsigned char>(rows, cols);
            blobPoints++;
          }
          else if (cv::pointPolygonTest(
              inflatedRectangles[i], cv::Point(cols, rows), false) > 0)
          {
            blobToRectangleSum +=
              (uint8_t)backProject.at<unsigned char>(rows, cols);
            blobToRectanglePoints++;
          }
        }
      }

      //!< The average probability of the points consisting the inflated
      //!< rectangle matching the inHistogram
      float rectangleMatchProbability =
        blobToRectangleSum / blobToRectanglePoints / 255;

      //!< The average probability of the points inside the blob's outline
      //!< matching the inHistogram
      float blobMatchProbability = blobSum / blobPoints / 255;

      //!< This blob is considered valid, with a non zero validity probability,
      //!< if the points consisting the inflated rectangle have a greater
      //!< resemblance (through the probability-expressing values of the
      //!< back project cv::MatND) to the inHistogram than the one of the points
      //!< inside the blob's outline
      if (rectangleMatchProbability > blobMatchProbability)
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) =
          rectangleMatchProbability - blobMatchProbability;
      }
      else
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) = 0.0;
      }

      msgs->push_back(TOSTR(probabilitiesVector->at(validKeyPointsIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesTextureBackProject");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is
    attached to an order which relates to the sequence of the overall
    filter execution.
    @param[in] rgbImage [const cv::Mat&] The input rgb image
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
  void RgbFilters::checkHoles(
    const cv::Mat& rgbImage,
    const cv::MatND& inHistogram,
    const HolesConveyor& conveyor,
    std::vector<std::vector<float> >* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles", "sift");
    #endif

    std::set<unsigned int> indexes;
    std::vector<std::string> finalMsgs;

    std::map<int, int> filtersOrder;

    if (Parameters::run_checker_color_homogenity > 0)
    {
      filtersOrder[Parameters::run_checker_color_homogenity ] = 1;
    }
    if (Parameters::run_checker_luminosity_diff> 0)
    {
      filtersOrder[Parameters::run_checker_luminosity_diff ] = 2;
    }
    if (Parameters::run_checker_texture_diff> 0)
    {
      filtersOrder[Parameters::run_checker_texture_diff] = 3;
    }
    if (Parameters::run_checker_texture_backproject> 0)
    {
      filtersOrder[Parameters::run_checker_texture_backproject] = 4;
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
        Parameters::rectangle_inflation_size,
        inHistogram,
        &probabilitiesVector->at(counter),
        &imgs,
        &msgs);

      counter++;
    } //!< o_it iterator ends

    #ifdef DEBUG_SHOW
    if(Parameters::debug_show_check_holes) // Debug
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
    @param[in] method [const unsigned int&] The filter identifier to execute
    @param[in] img [const cv::Mat&] The input rgb image
    @param[in] conveyor [const HolesConveyor&] The structure that
    holds the final holes' data
    @param[in] inflationSize [const int&] The amount of pixels by which each
    bounding box is inflated
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree with which each
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node.
    @param[in][out] imgs [std::vector<cv::Mat>*] A vector of images which shows
    the holes that are considered valid by each filter
    @param[in][out] msgs [std::vector<std::string>*] Debug messages
    @return void
   **/
  void RgbFilters::applyFilter(
    const unsigned int& method,
    const cv::Mat& img,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    const cv::MatND& inHistogram,
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

    //!< Initialize structures
    finalMsgs.clear();
    msgs_.clear();

    switch(method)
    {
      //!< Filter #1 (Color homogenity inside blob)-----------------------------
      case 1 :
        {
          checkHolesColorHomogenity(
            img,
            conveyor,
            probabilitiesVector,
            &msgs_);
          windowMsg = "Filter: Color homogenity";
          break;
        }
        //!< Filter #2 (Luminosity difference)----------------------------------
        //!< Check for luminosity difference between the points that constitute
        //!< the blob's bounding box and the points inside the blob's outline
      case 2 :
        {
          checkHolesLuminosityDiff(
            img,
            conveyor,
            inflationSize,
            probabilitiesVector,
            &msgs_);
          windowMsg = "Filter: Luminosity difference";
          break;
        }
        //!< Filter #3 (Texture difference)-------------------------------------
      case 3 :
        {
          checkHolesTextureDiff(
            img,
            inHistogram,
            conveyor,
            inflationSize,
            probabilitiesVector,
            &msgs_);
          windowMsg = "Filter: Texture difference";
          break;
        }
        //!< Filter #4 (Back project model histogram)---------------------------
      case 4 :
        {
          checkHolesTextureBackProject(
            img,
            inHistogram,
            conveyor,
            inflationSize,
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
    if(Parameters::debug_show_check_holes) // Debug
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
