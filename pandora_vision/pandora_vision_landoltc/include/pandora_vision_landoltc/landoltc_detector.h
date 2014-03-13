/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/
#ifndef LANDOLTCDETECTOR_H
#define LANDOLTCDETECTOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>
namespace pandora_vision
{
class LandoltCDetector
{
private:

  //!<Value for threshholding gradients
  int _minDiff;
  //!<Value for thresholding values in voting array
  int _threshold;
  //!<Vector containing centers of possible landolts
  std::vector <cv::Point> _centers;
  //!<Vector containing colors of landolts,used for seperating them later
  std::vector<cv::Scalar> _fillColors;
  //!<Vector containing bounding rectangles of each landolt
  std::vector<cv::Rect> _rectangles;
  //!<Vector containing centers of verified landolts
  std::vector<cv::Point> _newCenters;
  //!<Vector containing contour points of reference C
  std::vector<std::vector<cv::Point> > _refContours;
  //!<2D Matrix containing "votes" of each pixel, used for finding the centers
  cv::Mat _voting;
  //!<2D Matrix containing landoltsC's, each colored with a unique color
  cv::Mat _coloredContours;
  //!<2D Matric used for separating each LandoltC to each parts
  cv::Mat _mask;

public:

  //!< Constructor
  LandoltCDetector();

  //!< Destructor
  virtual ~LandoltCDetector();

  /**
  @brief Function for the initialization of the reference image
  @param void
  @return void
  **/
  void initializeReferenceImage(std::string path);

  /**
  @brief Rasterize line between two points
  @param A [cv::Point] The start point of a line
  @param B [cv::Point] The end point of a line
  @return void
  **/
  void rasterizeLine(cv::Point A, cv::Point B);

  /**
  @brief Finds Centers based on gradient
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param grX [float*] X gradient component
  @param grY [float*] Y gradient component
  @return void
  **/
  void findCenters(int rows, int cols, float* grX, float* grY);

  /**
  @brief Finds LandoltC Contours on RGB Frames
  @param inImage [cv::Mat&] Input Image
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param ref [std::vector<cv::Point>] Vector containing contour points of reference image
  @return void
  **/
  void findLandoltContours(cv::Mat& inImage, int rows, int cols, std::vector<cv::Point> ref);

  /**
  @brief Mask for separating a LandoltC Contour to its components
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @return void
  **/
  void applyMask(int rows, int cols);

  /**
  @brief Thinning algorith using the Zhang-Suen method
  @param in [cv::Mat&] Matrix containing the frame to thin
  @return void
  **/
  void thinning(cv::Mat& in);

  /**
  @brief Thinning iteration call from the thinning function
  @param in [cv::Mat&] Matrix containing the frame to thin
  @param iter [int] Number of iteration with values 1-2
  @return void
  **/
  void thinningIter(cv::Mat& in, int iter);

  /**
  @brief Function called for the initiation of LandoltC search in the frame
  @param input [cv::Mat&] Matrix containing the frame received from the camera
  @return void
  **/
  void begin(cv::Mat& input);

};
}
#endif
