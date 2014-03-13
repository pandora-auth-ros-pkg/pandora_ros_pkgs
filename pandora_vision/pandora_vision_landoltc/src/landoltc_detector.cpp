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

#include "pandora_vision_landoltc/landoltc_detector.h"

namespace pandora_vision
{
//!< Constructor
LandoltCDetector::LandoltCDetector()
{
  _minDiff = 60;

  _threshold = 90;
}

//!< Destructor
LandoltCDetector::~LandoltCDetector()
{
  ROS_INFO("landoltc_detector instance destroyed");
}

/**
@brief Function for the initialization of the reference image
@param void
@return void
**/
void LandoltCDetector::initializeReferenceImage(std::string path)
{
  //!<Loading reference image passed as argument to main
  cv::Mat ref;
  std::cout << path << std::endl;
  ref = cv::imread(path);
  if (!ref.data)
    std::cout << "Pattern image not loaded" << std::endl;

  //!< Turning to gray and binarizing ref image

  cv::cvtColor(ref, ref, CV_BGR2GRAY);
  cv::threshold(ref, ref, 128, 255, cv::THRESH_BINARY_INV);

  cv::findContours(ref, _refContours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
}

/**
@brief Rasterize line between two points
@param A [cv::Point] The start point of a line
@param B [cv::Point] The end point of a line
@return void
**/

void LandoltCDetector::rasterizeLine(cv::Point A, cv::Point B)
{
  unsigned short* votingData = (unsigned short*)(_voting.data);
  cv::Rect r(0, 0, _voting.cols, _voting.rows);

  //!< if line is out of frame return
  if(!cv::clipLine(r, A, B)) return;

  float rounding = 0.49999;

  int dx = B.x - A.x;
  int dy = B.y - A.y;

  //!<X major line

  if (abs(dx) >= abs(dy))
  {
    if (B.x < A.x) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.y - A.y) / (float)(B.x - A.x);
    float yIntercept = A.y - gradient * A.x + rounding;
    for (int i = A.x; i < B.x; i++)
    {
      float y = gradient * i + yIntercept;
      votingData[((int)y * _voting.cols) + i] += 1.0;
    }
  }

  //!<Y major line

  if (abs(dx) < abs(dy))
  {
    if (B.y < A.y) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.x - A.x) / (float)(B.y - A.y);
    float xIntercept = A.x - gradient * A.y + rounding;
    for (int i = A.y; i < B.y; i++)
    {
      float x = gradient * i + xIntercept;
      votingData[i * _voting.cols + (int)x] += 1.0;
    }
  }
}

/**
 @brief Finds Centers based on gradient
 @param rows [int] Number of rows of matrix
 @param cols [int] Number of columns of matrix
 @param grX [float*] X gradient component
 @param grY [float*] Y gradient component
 @return void
**/

void LandoltCDetector::findCenters(int rows, int cols, float* grX, float* grY)
{
  cv::Point center;

  //!< Rasterization of lines between thresholded points

  for (int x = 0; x < cols; x++)
  {
    for (int y = 0; y < rows; y++)
    {
      int i = y * cols + x;
      float dx = grX[i];
      float dy = grY[i];
      float mag = dx * dx + dy * dy;
      if (mag > (_minDiff * _minDiff))
      {
        mag = sqrt(mag);
        float s = 20 / mag;
        rasterizeLine(cv::Point(x + dx * s, y + dy * s), cv::Point(x - dx * s, y - dy * s));
      }
    }
  }

  const unsigned short* readvoting = (const unsigned short*)_voting.data;
  int de = 2;
  int bullcount = 0;

  //!<Searching for landoltC centers

  for (int y = de; y < rows - de ; y++)
  {
    for (int x = de; x < cols - de; x++)
    {
      int i = y * cols + x;
      int cur = readvoting[i];
      if (cur >= _threshold)
      {
        bool biggest = true;

        //!<Search if there's a bigger center in a smaller area
        for (int dy = -de; dy < de && biggest; dy++)
        {
          for(int dx = -de; dx < de; dx++)
          {
            float her = readvoting[(y + dy) * cols + (x + dx)];
            if (cur < her)
            {
              biggest = false;
              break;
            }
          }
        }

        if (biggest)
        {
          center.x = x;
          center.y = y;
          _centers.push_back(center);
          std::cout << "Bullseye " << bullcount++ << " xy " << center.x << "," << center.y << std::endl;
        }
      }
    }
  }

  return;
}

/**
  @brief Mask for separating a LandoltC Contour to its components
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @return void
**/

void LandoltCDetector::applyMask(int rows, int cols)
{

  for (int i = 0; i < _fillColors.size(); i++)
  {
    _mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::inRange(_coloredContours, _fillColors[i], _fillColors[i], _mask);
    cv::circle(_mask, _newCenters.at(i), 1, (255), -1);
    cv::waitKey(30);
    cv::imshow("Mask", _mask);
  }
}

/**
@brief Finds LandoltC Contours on RGB Frames
@param inImage [cv::Mat&] Input Image
@param rows [int] Number of rows of matrix
@param cols [int] Number of columns of matrix
@param ref [std::vector<cv::Point>] Vector containing contour points of reference image
@return void
**/

void LandoltCDetector::findLandoltContours(cv::Mat& inImage, int rows, int cols, std::vector<cv::Point> ref)
{
  cv::RNG rng(12345);

  //!<find contours and moments in frame used for shape matching later

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(inImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Moments> mu(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mu[i] = moments(contours[i], false);
  }

  std::vector<cv::Point2f> mc(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
  }

  std::vector<cv::Point> approx;

  //!<Shape matching using Hu Moments, and contour center proximity

  for(int i = 0; i < contours.size(); i++)
  {
    approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true) * 0.02, true);
    std::vector<cv::Point> cnt = contours[i];
    double prec = cv::matchShapes(cv::Mat(ref), cv::Mat(cnt), CV_CONTOURS_MATCH_I3, 0);

    for(std::vector<cv::Point>::iterator it = _centers.begin(); it != _centers.end(); ++it)
    {
      if (!isContourConvex(cv::Mat(cnt)) && fabs(mc[i].x - (*it).x) < 7 && fabs(mc[i].y - (*it).y) < 7 && prec < 0.43)
      {
        std::cout << "Prec is : " << prec << std::endl;
        cv::Rect bounding_rect = boundingRect((contours[i]));
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(_coloredContours, contours, i, color, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        _fillColors.push_back(color);
        _newCenters.push_back(mc[i]);
        cv::imshow("Contours", _coloredContours);
        _rectangles.push_back(bounding_rect);

      }
    }
  }
}

/**
@brief Function called from ImageCallBack that Initiates LandoltC search in the frame
@param input [cv::Mat&] Matrix containing the frame received from the camera
@return void
**/

void LandoltCDetector::begin(cv::Mat& input)
{
  cv::Mat gray, gradX, gradY, binary;
  cv::Mat erodeKernel(cv::Size(1, 1), CV_8UC1, cv::Scalar(1));

  cv::cvtColor(input, gray, CV_BGR2GRAY);

  _voting = cv::Mat::zeros(input.rows, input.cols, CV_16U);
  _coloredContours = cv::Mat::zeros(input.rows, input.cols, input.type());
  _mask = cv::Mat::zeros(input.rows, input.cols, CV_8UC1);

  cv::Sobel(gray, gradX, CV_32F, 1, 0, 3);
  cv::Sobel(gray, gradY, CV_32F, 0, 1, 3);

  float* gradXF = (float*)gradX.data;
  float* gradYF = (float*)gradY.data;

  findCenters(input.rows, input.cols, gradXF, gradYF);

  for(std::size_t i = 0; i < _centers.size(); i++)
  {
    cv::circle(input, _centers.at(i), 2, (0, 0, 255), -1);
  }

  blur(gray, gray, cv::Size(3, 3));

  cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 7, 2);

  cv::erode(binary, binary, erodeKernel);

  findLandoltContours(binary, input.rows, input.cols, _refContours[0]);

  for(std::size_t i = 0; i < _rectangles.size(); i++)
  {
    cv::rectangle(input, _rectangles.at(i), cv::Scalar(0, 0, 255), 1, 8, 0);
  }

  applyMask(input.rows, input.cols);

  cv::imshow("Raw", input);
  cv::imshow("Adaptive Threshold", binary);
  cv::waitKey(30);

  _centers.clear();
  _newCenters.clear();
  _rectangles.clear();
  _fillColors.clear();

}

/**
@brief Thinning algorith using the Zhang-Suen method
@param in [cv::Mat&] Matrix containing the frame to thin
@return void
**/

void LandoltCDetector::thinning(cv::Mat& in)
{
  in = in / 255;
  cv::Mat thinned;
  cv::Mat dif;
  in.copyTo(thinned);

  do
  {
    thinningIter(in, 1);
    thinningIter(in, 2);
    cv::absdiff(in, thinned, dif);
    in.copyTo(thinned);;
  }
  while(cv::countNonZero(dif) > 0);

  in = in * 255;
}

/**
@brief Thinning iteration call from the thinning function
@param in [cv::Mat&] Matrix containing the frame to thin
@param iter [int] Number of iteration with values 1-2
@return void
**/
void LandoltCDetector::thinningIter(cv::Mat& in, int iter)
{
  cv::Mat temp = cv::Mat::ones(in.size(), CV_8UC1);

  for(int y = 0; y < (in.rows - 1); y++)
  {
    for(int x = 0; x < (in.cols - 1); x++)
    {
      unsigned char p2 = in.at<uchar>(y - 1, x);
      unsigned char p3 = in.at<uchar>(y - 1, x + 1);
      unsigned char p4 = in.at<uchar>(y, x + 1);
      unsigned char p5 = in.at<uchar>(y + 1, x + 1);
      unsigned char p6 = in.at<uchar>(y + 1, x);
      unsigned char p7 = in.at<uchar>(y + 1, x - 1);
      unsigned char p8 = in.at<uchar>(y, x - 1);
      unsigned char p9 = in.at<uchar>(y - 1, x - 1);

      int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;

      int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1)
              + (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) + (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);

      if(iter == 1)
      {
        int c1 = p2 * p4 * p6;
        int c2 = p4 * p6 * p8;

        if(A == 1 && (B >= 2 && B <= 6) && c1 == 0 && c2 == 0) temp.at<uchar>(y, x) = 0;
      }

      if(iter == 2)
      {
        int c1 = p2 * p4 * p8;
        int c2 = p2 * p6 * p8;

        if(A == 1 && (B >= 2 && B <= 6) && c1 == 0 && c2 == 0) temp.at<uchar>(y, x) = 0;
      }
    }
  }

  cv::bitwise_and(in, temp, in);
}
}
