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
 * Author: Alexandros Philotheou, Vasilis Bosdelekidis
 *********************************************************************/

#include "rgb_node/rgb_processor.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class RgbProcessorTest
    @brief Tests the integrity of methods of class Rgb
   **/
  class RgbProcessorTest : public ::testing::Test
  {
    protected:

      RgbProcessorTest () {}

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
      void generateRgbRectangle (
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const unsigned char rgbIn,
          cv::Mat* image );

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

      }

      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image under processing
      cv::Mat squares_;
      RgbProcessor RgbProcessor_;

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
  void RgbProcessorTest::generateRgbRectangle (
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const unsigned char rgbIn,
      cv::Mat* image )
  {
    // Fill the inside of the desired rectangle with the @param rgbIn provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< cv::Vec3b >( rows, cols ).val[0] = rgbIn;
        image->at< cv::Vec3b >( rows, cols ).val[1] = rgbIn;
        image->at< cv::Vec3b >( rows, cols ).val[2] = rgbIn;
      }
    }
  }



  //! Tests RgbProcessor::findHoles
  TEST_F ( RgbProcessorTest, findHolesTest )
  {

    // Construct the lower right square
    cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    RgbProcessorTest::generateRgbRectangle
      ( cv::Point2f ( WIDTH - 200, HEIGHT - 200 ),
        50,
        50,
        100,
        &lowerRightSquare );

    // Construct the upper right image
    cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    RgbProcessorTest::generateRgbRectangle
      ( cv::Point2f ( WIDTH - 13, 3 ),
        10,
        10,
        100,
        &upperRightSquare );

    // Construct the upper left square
    cv::Mat upperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

    // Construct the upperLeft image
    RgbProcessorTest::generateRgbRectangle
      ( cv::Point2f ( 100, 100 ),
        50,
        50,
        100,
        &upperLeftSquare );

    //// Construct the mergable1 square
    //cv::Mat mergable1Square = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

    //// Construct the mergable1 image
    //RgbProcessorTest::generateRgbRectangle
    //  ( cv::Point2f ( 200, 200 ),
    //    50,
    //    50,
    //    100,
    //    &mergable1Square );

    //// Construct the mergable2 square
    //cv::Mat mergable2Square = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

    //// Construct the mergable2 image
    //RgbProcessorTest::generateRgbRectangle
    //  ( cv::Point2f ( 280, 280 ),
    //    40,
    //    40,
    //    100,
    //    &mergable2Square );

    //// Construct the non mergable square
    //cv::Mat nonMergableSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

    //// Construct the nonMergable image
    //RgbProcessorTest::generateRgbRectangle
    //  ( cv::Point2f ( 80, 80 ),
    //    40,
    //    40,
    //    200,
    //    &nonMergableSquare );

    squares_ = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    // Construct the squares_ image. The entire image is at a colour of
    // value approximate the the colour value of the images of walls
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( squares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
        {
          squares_.at< cv::Vec3b >( rows, cols ).val[0] = 116;
          squares_.at< cv::Vec3b >( rows, cols ).val[1] = 163;
          squares_.at< cv::Vec3b >( rows, cols ).val[2] = 171;
        }
      }
    }
    // Synthesize the final squares_ image
    squares_ += lowerRightSquare + upperRightSquare + upperLeftSquare;

    cv::waitKey();
    // Run RgbProcessor::findHoles
    HolesConveyor conveyor =
      RgbProcessor_.findHoles ( squares_ );

    // The number of keypoints found
    int size = conveyor.keypoint.size();

    // There should be four keypoints: the one of the upper left square, the one of the lower right square
    // The upper right square is
    // tiny and near the border and it will be clipped by the validation method
    ASSERT_EQ ( 2, size );

    // For every keypoint found, make assertions and expectations
    for (int k = 0; k < size; k++)
    {
      // The location of the keypoint should near the center of the square
      // in which it lies
      EXPECT_NEAR ( conveyor.keypoint[k].x,
          conveyor.rectangle[k].x + conveyor.rectangle[k].width / 2, 25 );
      EXPECT_NEAR ( conveyor.keypoint[k].y,
          conveyor.rectangle[k].y + conveyor.rectangle[k].height / 2, 25 );

      // The hole should have exactly four vertices
      //EXPECT_EQ ( 4, conveyor.holes[k].rectangle.size() );

      // Rectangle's perimeter must be near 10000
      EXPECT_NEAR ( 2500, conveyor.rectangle[k].width * conveyor.rectangle[k].height, 2000  );
    }

    //squares_ = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    //// Construct the squares_ image. The entire image is at a colour of
    //// value approximate the the colour value of the images of walls
    //for ( int rows = 0; rows < HEIGHT; rows++ )
    //{
    //  for ( int cols = 0; cols < WIDTH; cols++ )
    //  {
    //    if ( squares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
    //    {
    //      squares_.at< cv::Vec3b >( rows, cols ).val[0] = 116;
    //      squares_.at< cv::Vec3b >( rows, cols ).val[1] = 163;
    //      squares_.at< cv::Vec3b >( rows, cols ).val[2] = 171;
    //    }
    //  }
    //}
    //// Synthesize the final squares_ image with the mergable contours
    //squares_ += mergable1Square + mergable2Square + nonMergableSquare;

    //// Run RgbProcessor::findHoles
    //conveyor =
    //  RgbProcessor_.findHoles ( squares_ );

    //// The number of keypoints found
    //size = conveyor.keypoint.size();

    //// There should be two keypoints; the merged contour's and the nonmergable's
    //ASSERT_EQ ( 2, size );

    //// See if the merge was right
    //squares_ = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    //// Construct the squares_ image. The entire image is at a colour of
    //// value approximate the the colour value of the images of walls
    //for ( int rows = 0; rows < HEIGHT; rows++ )
    //{
    //  for ( int cols = 0; cols < WIDTH; cols++ )
    //  {
    //    if ( squares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
    //    {
    //      squares_.at< cv::Vec3b >( rows, cols ).val[0] = 116;
    //      squares_.at< cv::Vec3b >( rows, cols ).val[1] = 163;
    //      squares_.at< cv::Vec3b >( rows, cols ).val[2] = 171;
    //    }
    //  }
    //}
    //// Synthesize the final squares_ image with the mergable contours
    //squares_ += mergable1Square + mergable2Square;

    //// Run RgbProcessor::findHoles
    //conveyor =
    //  RgbProcessor_.findHoles ( squares_ );

    //// The number of keypoints found
    //size = conveyor.keypoint.size();

    //// There should be one keypoint
    //ASSERT_EQ ( 1, size );
    // For this keypoint, make assertions and expectations
      // The location of the keypoint should near the center of the square
      // in which it lies
      //EXPECT_NEAR ( conveyor.keypoint[0].x,
      //    conveyor.rectangle[0].x + conveyor.rectangle[0].width / 2, 25 );
      //EXPECT_NEAR ( conveyor.keypoint[0].y,
      //    conveyor.rectangle[0].y + conveyor.rectangle[0].height / 2, 25 );

      //// The hole should have exactly four vertices
      ////EXPECT_EQ ( 4, conveyor.holes[k].rectangle.size() );

      //// Rectangle's perimeter must be near 28900 due to the merge. 
      //// Due to dilations it could be an offset up to around 10 pixels 
      //// at each of the two directions(horizontal + vertical) (needs to fix)
      //EXPECT_NEAR ( 14400, conveyor.rectangle[0].width * conveyor.rectangle[0].height, 3000  );


  }

} // namespace pandora_vision
