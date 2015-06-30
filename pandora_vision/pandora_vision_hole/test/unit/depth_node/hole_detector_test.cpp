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

#include "depth_node/hole_detector.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class HoleDetectorTest
    @brief Tests the integrity of methods of class HoleDetector
   **/
  class HoleDetectorTest : public ::testing::Test
  {
    protected:
      HoleDetectorTest() {}

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
      void generateDepthRectangle(
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        const float& depthIn,
        cv::Mat* image );

      // Sets up one image: squares_,
      // which features three squares of size 100.
      // The first one (order matters here) has its upper left vertex at
      // (100, 100),
      // the second one has its upper right vertex at (WIDTH - 3, 3)
      // (so that the blob it represents can barely be identified)
      // and the the third one has its lower right vertex at
      // (WIDTH - 1, HEIGHT - 1)
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // The image upon which the squares will be inprinted
        squares_ = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

        // Construct the squares_ image

        // Set the depth for each point of the squares_ image to 1.0
        for (int rows = 0; rows < squares_.rows; rows++)
        {
          for (int cols = 0; cols < squares_.cols; cols++)
          {
            squares_.at<float>(rows, cols) = 1.0;
          }
        }

        // Construct the lower right square
        cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

        HoleDetectorTest::generateDepthRectangle
          (cv::Point2f(WIDTH - 100, HEIGHT - 100),
            100,
            100,
            1.2,
            &lowerRightSquare);

        // Construct the upper right image
        cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

        HoleDetectorTest::generateDepthRectangle
          (cv::Point2f(WIDTH - 103, 3),
            100,
            100,
            2.2,
            &upperRightSquare);

        // Construct the upper left square
        cv::Mat upperLeftSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1);

        // Construct the square_ image
        HoleDetectorTest::generateDepthRectangle
          (cv::Point2f(100, 100),
            100,
            100,
            1.8,
            &upperLeftSquare);

        // Synthesize the final squares_ image
        squares_ += lowerRightSquare + upperRightSquare + upperLeftSquare;

      }
      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;
  };



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
  void HoleDetectorTest::generateDepthRectangle(
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
        image->at<float>(rows, cols) = depthIn;
      }
    }
  }



  //! Tests HoleDetector::findHoles
  TEST_F(HoleDetectorTest, findHolesTest)
  {
    // Run HoleDetector:findHoles
    HolesConveyor conveyor = HoleDetector::findHoles(squares_);

    // The number of keypoints found
    int size = conveyor.size();

    // There should be two keypoints: the one of the upper left square
    // and the one of the upper right square. The lower right square is
    // adjacent to the edges of the image and will be clipped by the
    // edge contamination method
    ASSERT_EQ(2, size);

    // For every keypoint found, make assertions and expectations
    for (int k = 0; k < size; k++)
    {
      // The location of the keypoint should be near the center of the square
      // in which it lies
      EXPECT_NEAR(conveyor.holes[k].keypoint.pt.x,
        conveyor.holes[k].rectangle[0].x + 50, 1);

      // The hole should have exactly four vertices
      EXPECT_EQ (4, conveyor.holes[k].rectangle.size());

      // There should be 400 outline points
      EXPECT_EQ (400, conveyor.holes[k].outline.size());
    }
  }

}  // namespace pandora_vision
