/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors:
 *   Bosdelekidis Vasilis <vasilis1bos@gmail.com>
 *********************************************************************/

#include <cmath>
#include <utility>
#include <algorithm>
#include <vector>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/barrel_detection/fast_symmetry_detector.h"
#include "pandora_vision_obstacle/barrel_detection/parameters.h"

namespace pandora_vision
{
  /**
    @class FastSymmetryDetectorTest
    @brief Tests the integrity of methods of class FastSymmetryDetector
   **/
  class FastSymmetryDetectorTest : public ::testing::Test
  {
    protected:
      FastSymmetryDetectorTest() {}

      /**
        @brief Constructs a rectangle of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] rgbIn [const unsigned char] The rgb value for all points i
        nside the rectangle
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted on
        return void
       **/
      void generateRgbRectangle(
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const unsigned char rgbIn,
          cv::Mat* image);

      /**
        @brief Constructs a rectangle of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] depthIn [const float&] The depth value for all points inside
        the rectangle
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted
        return void
       **/
      void generateDepthRectangle(
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const float& depthIn,
          cv::Mat* image);

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;
      }

      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image under processing
      cv::Mat depthImage;
  };



  /**
    @brief Constructs a rectangle of width @param x and height of @param y.
    All points inside the said rectangle are with a value of @param rgbIn.
    The outline of the rectangle is at a value of 1.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] rgbIn [const unsigned char] The rgb value for all points i
    nside the rectangle
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on
    return void
   **/
  void FastSymmetryDetectorTest::generateRgbRectangle(
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const unsigned char rgbIn,
      cv::Mat* image)
  {
    // Fill the inside of the desired rectangle with the @param rgbIn provided
    for (int rows = upperLeft.y; rows < upperLeft.y + y; rows++)
    {
      for (int cols = upperLeft.x; cols < upperLeft.x + x; cols++)
      {
        image->at< cv::Vec3b >(rows, cols).val[0] = rgbIn;
        image->at< cv::Vec3b >(rows, cols).val[1] = rgbIn;
        image->at< cv::Vec3b >(rows, cols).val[2] = rgbIn;
      }
    }
  }

  /**
    @brief Constructs a rectangle of width @param x and height of @param y.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] depthIn [const float&] The depth value for all points inside
    the rectangle
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on
    return void
   **/
  void FastSymmetryDetectorTest::generateDepthRectangle(
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const float& depthIn,
      cv::Mat* image)
  {
    // Fill the inside of the desired rectangle with the @param depthIn provided
    for (int rows = upperLeft.y; rows < upperLeft.y + y; rows++)
    {
      for (int cols = upperLeft.x; cols < upperLeft.x + x; cols++)
      {
        image->at< float >(rows, cols) = depthIn;
      }
    }
  }

  // ! Tests FastSymmetryDetector::rotateEdges
  TEST_F(FastSymmetryDetectorTest, rotateEdgesTest)
  {
  }

  // ! Tests FastSymmetryDetector::getMaxDistance
  TEST_F(FastSymmetryDetectorTest, getMaxDistanceTest)
  {
    // The image upon which the squares will be inprinted
    depthImage = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

    // Construct the squares_ image

    // Set the depth for each point of the depthImage to constant value
    for (int rows = 0; rows < depthImage.rows; rows++)
    {
      for (int cols = 0; cols < depthImage.cols; cols++)
      {
        depthImage.at<float>(rows, cols) = 0.2;
      }
    }

    /* Determine the shape of Hough accumulationmatrix */
    float rhoDivs = hypotf(depthImage.rows, depthImage.cols) + 1;
    float thetaDivs = 180.0;

    FastSymmetryDetector detector(depthImage.size(), cv::Size(rhoDivs, thetaDivs), 1);

    cv::Rect region(0, depthImage.rows, thetaDivs * 2.0, rhoDivs * 0.5);

    int canny_thresh_1 = BarrelDetection::fsd_canny_thresh_1;
    int canny_thresh_2 = BarrelDetection::fsd_canny_thresh_2;
    int min_pair_dist  = BarrelDetection::fsd_min_pair_dist;
    int max_pair_dist  = BarrelDetection::fsd_max_pair_dist;
    int no_of_peaks    = BarrelDetection::fsd_no_of_peaks;

    cv::Mat temp, edge;

    temp = depthImage.clone();

    temp.convertTo(edge, CV_8UC1, 255, 0);
    // edge.copyTo(temp1);

    /* Find the edges */
    if (edge.channels() == 3)
      cvtColor(edge, edge, CV_BGR2GRAY);
    cv::Canny(edge, edge, canny_thresh_1, canny_thresh_2);

    /* Vote for the accumulation matrix */
    detector.vote(edge, min_pair_dist, max_pair_dist);

    /* Get the symmetrical line */
    std::vector<std::pair<cv::Point, cv::Point> > result = detector.getResult(no_of_peaks);
    float maxDist = 0.0;
    float maxY;
    float minY;
    detector.getMaxDistance(&maxDist);

    // It is expected that the maximum distance will be zero, because no
    // object has been found
    EXPECT_NEAR(0.0, maxDist, 0.1);
    // And the symmetric lines are also zero
    EXPECT_EQ(0, result.size());

    // Construct a square
    cv::Mat square = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

    FastSymmetryDetectorTest::generateDepthRectangle
      (cv::Point2f(WIDTH - 350, HEIGHT - 350),
        310,
        310,
        0.9,
        &square);

    depthImage += square;


    temp = depthImage.clone();

    temp.convertTo(edge, CV_8UC1, 255, 0);
    // edge.copyTo(temp1);

    /* Find the edges */
    if (edge.channels() == 3)
      cvtColor(edge, edge, CV_BGR2GRAY);
    cv::Canny(edge, edge, canny_thresh_1, canny_thresh_2);

    /* Vote for the accumulation matrix */
    detector.vote(edge, min_pair_dist, max_pair_dist);

    /* Get the symmetrical line */
    result = detector.getResult(no_of_peaks);
    maxDist = 0.0;
    detector.getMaxDistance(&maxDist);

    // It is expected that the maximum distance between symmetric lines will
    // be equal to the square's width or height
    EXPECT_NEAR(310, maxDist, 100);

    for (int i = 0; i < result.size(); i ++)
    {
      // It is expected that points of the symmetry line are at the center of
      // the square (either vertically either horizontally)
      if (std::abs(result[i].first.x - result[i].second.x) < 100)
      {
        EXPECT_NEAR(445, result[i].first.x, 50);
        EXPECT_NEAR(445, result[i].second.x, 50);
      }
      else
      {
        EXPECT_NEAR(290, std::min(result[i].first.x, result[i].second.x), 50);
        EXPECT_NEAR(600, std::max(result[i].first.x, result[i].second.x), 50);
      }
    }
  }

}  // namespace pandora_vision
