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

#include "depth_node/depth_processor.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class DepthProcessorTest
    @brief Tests the integrity of methods of class DepthProcessor
   **/
  class DepthProcessorTest : public ::testing::Test
  {
    protected:

      DepthProcessorTest() {}

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
      void generateDepthRectangle (
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const float& depthIn,
          cv::Mat* image );

      //! Sets up one image: squares_,
      //! which features three squares of size 100.
      //! The first one (order matters here) has its upper left vertex at
      //! (100, 100),
      //! the second one has its upper right vertex at (WIDTH - 3, 3)
      //! (so that the blob it represents can barely be identified)
      //! and the the third one has its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1)
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;
      DepthProcessor DepthProcessor_;

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
  void DepthProcessorTest::generateDepthRectangle (
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const float& depthIn,
      cv::Mat* image )
  {
    // Fill the inside of the desired rectangle with the @param depthIn provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = depthIn;
      }
    }
  }



  //! Tests DepthProcessor::findHoles
  TEST_F ( DepthProcessorTest, findHolesTest )
  {
    // The image upon which the squares will be inprinted
    squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Construct the squares_ image

    // Set the depth for each point of the squares_ image to 1.0
    for ( int rows = 0; rows < squares_.rows; rows++ )
    {
      for ( int cols = 0; cols < squares_.cols; cols++ )
      {
        squares_.at< float >( rows, cols ) = 1.0;
      }
    }

    // Construct the lower right square
    cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( WIDTH - 200, HEIGHT - 300 ),
        80,
        80,
        1.2,
        &lowerRightSquare );

    // Construct the upper right image
    cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( WIDTH - 103, 3 ),
        5,
        5,
        2.2,
        &upperRightSquare );

    // Construct the upper left square
    cv::Mat upperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Construct the square_ image
    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( 20, 50 ),
        80,
        80,
        1.8,
        &upperLeftSquare );

    // Construct the huge square
    cv::Mat hugeSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Construct the square_ image
    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( 5, 250 ),
        400,
        50,
        1.8,
        &hugeSquare );

    // Construct the mergable1 square
    cv::Mat mergable1Square = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Construct the square_ image
    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( 195, 150 ),
        20,
        20,
        1.8,
        &mergable1Square );

    // Construct the mergable2 square
    cv::Mat mergable2Square = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Construct the mergable2 square
    DepthProcessorTest::generateDepthRectangle
      ( cv::Point2f ( 225, 180),
        40,
        40,
        1.7,
        &mergable2Square );

    cv::Mat cleanSquares_;
    
    //keep the original canvas without any squares
    squares_.copyTo(cleanSquares_);
    // Synthesize the final squares_ image
    squares_ += lowerRightSquare + upperRightSquare + upperLeftSquare + hugeSquare;
    // Run DepthProcessor::findHoles
    HolesConveyor conveyor;
    int size;
    conveyor = DepthProcessor_.findHoles ( squares_ );

    // The number of keypoints found
    size = conveyor.keypoint.size();

    // There should be two keypoints: the one of the upper left square
    // and the one of the lower right square. The upper right square is
    // is a tiny contour, the huge square is huge and will be clipped by the
    // validation strategy in depth local algorithm.
    ASSERT_EQ ( 2, size );

    // For every keypoint found, make assertions and expectations
    for ( int k = 0; k < size; k++ )
    {
      // The location of the keypoint should be near the center of the square
      // in which it lies
      EXPECT_NEAR ( conveyor.keypoint[k].x,
          conveyor.rectangle[k].x + conveyor.rectangle[k].width / 2, 25 );
      EXPECT_NEAR ( conveyor.keypoint[k].y,
          conveyor.rectangle[k].y + conveyor.rectangle[k].height / 2, 25 );

      // The hole should have exactly four vertices
      //EXPECT_EQ ( 4, conveyor.holes[k].rectangle.size() );

      // Rectangle's perimeter must be near 10000
      EXPECT_NEAR ( 6400, conveyor.rectangle[k].width * conveyor.rectangle[k].height, 5000  );
    }
    // Synthesize the final squares_ image with the two mergable squares
    cleanSquares_.copyTo(squares_);
    squares_ += mergable1Square + mergable2Square;
    // insert some blacks between the two mergable squares
    //for( int rows = 230; rows < 290; rows += 10 )
    //{
    //  for ( int cols = 160; cols < 260; cols += 2 )
    //  {
    //    squares_.at< float >( rows, cols ) = 0.0;
    //  }
    //}

    // Run DepthProcessor::findHoles
    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    conveyor = DepthProcessor_.findHoles ( squares_ );
    // The number of keypoints found
    size = conveyor.keypoint.size();
    // There should be one keypoint: the one from the merged contours
    ASSERT_EQ ( 1, size );

    // For every keypoint found, make assertions and expectations
    for ( int k = 0; k < size; k++ )
    {
      // The location of the keypoint should be near the center of the square
      // in which it lies
      EXPECT_NEAR ( conveyor.keypoint[k].x,
          conveyor.rectangle[k].x + conveyor.rectangle[k].width / 2, 25 );
      EXPECT_NEAR ( conveyor.keypoint[k].y,
          conveyor.rectangle[k].y + conveyor.rectangle[k].height / 2, 25 );

      // The hole should have exactly four vertices
      //EXPECT_EQ ( 4, conveyor.holes[k].rectangle.size() );

      // Rectangle's perimeter must be near 10000
      EXPECT_NEAR ( 6400, conveyor.rectangle[k].width * conveyor.rectangle[k].height, 5000  );
    }

    // Check that for small depth value there are no holes found
    
    // Set the depth for each point of the squares_ image to 0.4
    for ( int rows = 0; rows < squares_.rows; rows++ )
    {
      for ( int cols = 0; cols < squares_.cols; cols++ )
      {
        squares_.at< float >( rows, cols ) = 0.4;
      }
    }
    squares_ += mergable1Square + mergable2Square;
    

    // Run DepthProcessor::findHoles
    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    conveyor = DepthProcessor_.findHoles ( squares_ );
    // The number of keypoints found
    size = conveyor.keypoint.size();
    // There should be no keypoints: the hole depth node procedure is ignored
    ASSERT_EQ ( 0, size );

    // Check that contours that are detected from noisy pixels, are eliminated
    
    // Set the depth for each point of the squares_ image to 1.0
    for ( int rows = 0; rows < squares_.rows; rows++ )
    {
      for ( int cols = 0; cols < squares_.cols; cols++ )
      {
        squares_.at< float >( rows, cols ) = 1.0;
      }
    }

    // Set a small and random region to 0.0, simulating a noisy region
    for ( int rows = 150; rows < 200; rows += 2 )
    {
      for ( int cols = 80; cols < 140; cols += 3 )
      {
        squares_.at< float >( rows, cols ) = 0.0;
      }
    }
    //squares_ += mergable1Square + mergable2Square;
    

    // Run DepthProcessor::findHoles
    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    conveyor = DepthProcessor_.findHoles ( squares_ );
    // The number of keypoints found
    size = conveyor.keypoint.size();
    // There should be no keypoints: the noisy region must be ignored
    ASSERT_EQ ( 0, size );
  }

} // namespace pandora_vision
