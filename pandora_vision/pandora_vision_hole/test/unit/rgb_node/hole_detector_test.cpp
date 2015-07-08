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

#include "rgb_node/rgb_hole_detector.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace rgb
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

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct the lower right square
        cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

        HoleDetectorTest::generateRgbRectangle
          (cv::Point2f(WIDTH - 100, HEIGHT - 100),
            100,
            100,
            100,
            &lowerRightSquare);

        // Construct the upper right image
        cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

        HoleDetectorTest::generateRgbRectangle
          (cv::Point2f(WIDTH - 103, 3),
            100,
            100,
            100,
            &upperRightSquare);

        // Construct the upper left square
        cv::Mat upperLeftSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

        // Construct the upperLeft image
        HoleDetectorTest::generateRgbRectangle
          (cv::Point2f(100, 100),
            100,
            100,
            100,
            &upperLeftSquare);

        // Synthesize the final squares_ image
        squares_ = lowerRightSquare + upperRightSquare + upperLeftSquare;

        // Construct the squares_ image. The entire image is at a colour of
        // value approximate the the colour value of the images of walls
        for (int rows = 0; rows < HEIGHT; rows++)
        {
          for (int cols = 0; cols < WIDTH; cols++)
          {
            if (squares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
            {
              squares_.at<cv::Vec3b>( rows, cols ).val[0] = 116;
              squares_.at<cv::Vec3b>( rows, cols ).val[1] = 163;
              squares_.at<cv::Vec3b>( rows, cols ).val[2] = 171;
            }
          }
        }
      }

      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image under processing
      cv::Mat squares_;
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
  void HoleDetectorTest::generateRgbRectangle(
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    const unsigned char rgbIn,
    cv::Mat* image)
  {
    // Fill the inside of the desired rectangle with the @param rgbIn provided
    for(int rows = upperLeft.y; rows < upperLeft.y + y; rows++)
    {
      for (int cols = upperLeft.x; cols < upperLeft.x + x; cols++)
      {
        image->at<cv::Vec3b>(rows, cols).val[0] = rgbIn;
        image->at<cv::Vec3b>(rows, cols).val[1] = rgbIn;
        image->at<cv::Vec3b>(rows, cols).val[2] = rgbIn;
      }
    }
  }



  //! Tests HoleDetector::findHoles
  TEST_F (HoleDetectorTest, findHolesTest)
  {
    // Obtain the histogram of images of walls
    std::vector<cv::MatND> wallsHistogram;
    Histogram::getHistogram(&wallsHistogram,
      Parameters::Histogram::secondary_channel);

    // Extract the edges of the rgb image via segmentation
    Parameters::Rgb::edges_extraction_method = 0;

    // Run HoleDetector:findHoles
    HolesConveyor conveyor =
      RgbHoleDetector::findHoles (squares_, wallsHistogram);

    // The number of keypoints found
    int size = conveyor.size();

    // There should be one keypoint: the one of the upper left square.
    // The lower right square and the upper right square are
    // adjacent to the edges of the image and will be clipped by the
    // edge contamination method
    ASSERT_EQ (1, size);

    // For every keypoint found, make assertions and expectations
    for (int k = 0; k < size; k++)
    {
      // The location of the keypoint should near the center of the square
      // in which it lies
      EXPECT_NEAR(conveyor.holes[k].keypoint.pt.x,
        conveyor.holes[k].rectangle[0].x + 50, 1);

      // The hole should have exactly four vertices
      EXPECT_EQ(4, conveyor.holes[k].rectangle.size());

      // There should be 400 outline points
      EXPECT_EQ(400, conveyor.holes[k].outline.size());
    }


    // Extract the edges of the rgb image via backprojection
    Parameters::Rgb::edges_extraction_method = 1;

    // Clear the conveyor
    HolesConveyorUtils::clear(&conveyor);

    // Run HoleDetector:findHoles
    conveyor = RgbHoleDetector::findHoles(squares_, wallsHistogram);

    // The number of keypoints found
    size = conveyor.size();

    // There should be one keypoint: the one of the upper left square.
    // The lower right square and the upper right square are
    // adjacent to the edges of the image and will be clipped by the
    // edge contamination method
    ASSERT_EQ(2, size);

    // For every keypoint found, make assertions and expectations
    for (int k = 0; k < size; k++)
    {
      // The location of the keypoint should near the center of the square
      // in which it lies
      EXPECT_NEAR(conveyor.holes[k].keypoint.pt.x,
        conveyor.holes[k].rectangle[0].x + 50, 1);

      // The hole should have exactly four vertices
      EXPECT_EQ(4, conveyor.holes[k].rectangle.size());

      // There should be 400 minus 4 (vertices) * 3 (points) outline points
      EXPECT_EQ(400 - 4 * 3, conveyor.holes[k].outline.size());
    }
  }

}  // namespace rgb
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
