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

#include "utils/blob_detection.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  class BlobDetectionTest : public ::testing::Test
  {
    protected:

      BlobDetectionTest() {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // A square with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
        square_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        // Two squares with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
        // A' (HEIGHT - 1 - 100, WIDTH - 1 - 100),
        // B' (HEIGHT - 1 - 100, WIDTH - 1)
        // C' (HEIGHT - 1, WIDTH - 1)
        // D' (HEIGHT - 1, WIDTH - 1 - 100)
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        ASSERT_EQ( HEIGHT, square_.rows );
        ASSERT_EQ( WIDTH, square_.cols );


        // Construct the square_ image
        for ( int cols = 100; cols < 200; cols++)
        {
          square_.at< unsigned char >( 100, cols ) = 255;
          square_.at< unsigned char >( 199, cols ) = 255;
        }
        for ( int rows = 100; rows < 200; rows++)
        {
          square_.at< unsigned char >( rows, 100 ) = 255;
          square_.at< unsigned char >( rows, 199 ) = 255;
        }

        // Locate the outline points of the square in the square_ image
        for ( int rows = 0; rows < square_.rows; rows++ )
        {
          for ( int cols = 0; cols < square_.cols; cols++ )
          {
            if ( square_.at< unsigned char >( rows, cols ) != 0 )
            {
              square_outline_points_vector_.push_back
                ( cv::Point2f ( cols, rows) );
            }
          }
        }

        ASSERT_EQ ( 396, square_outline_points_vector_.size() );




        // Construct the squares_ image
        for ( int cols = WIDTH - 100; cols < WIDTH; cols++)
        {
          squares_.at< unsigned char >( HEIGHT - 100, cols ) = 255;
          squares_.at< unsigned char >( HEIGHT - 1, cols ) = 255;
        }


        for ( int rows = HEIGHT - 100; rows < HEIGHT ; rows++)
        {
          squares_.at< unsigned char >( rows, WIDTH - 100 ) = 255;
          squares_.at< unsigned char >( rows, WIDTH - 1 ) = 255;
        }

        std::vector< cv::Point2f > lowerRightSquareOutline;
        for ( int rows = 0; rows < squares_.rows; rows++ )
        {
          for ( int cols = 0; cols < squares_.cols; cols++ )
          {
            if ( squares_.at< unsigned char>( rows, cols ) != 0)
            {
              lowerRightSquareOutline.push_back ( cv::Point2f ( cols, rows ) );
            }
          }
        }
        squares_outline_points_vector_.push_back( lowerRightSquareOutline );

        // Add the upper left square to the squares_ image
        square_.copyTo( squares_ );

        // Add the vector of outline points of the upper left square to the
        // vector holding the vectors of outline points of the squares_ image
        squares_outline_points_vector_.push_back
          ( square_outline_points_vector_ );

        ASSERT_EQ( 396 , squares_outline_points_vector_[0].size() );
        ASSERT_EQ( 396 , squares_outline_points_vector_[1].size() );

      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // A square with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
      cv::Mat square_;

      // Two squares with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
      // A' (HEIGHT - 1 - 100, WIDTH - 1 - 100),
      // B' (HEIGHT - 1 - 100, WIDTH - 1)
      // C' (HEIGHT - 1, WIDTH - 1)
      // D' (HEIGHT - 1, WIDTH - 1 - 100)
      cv::Mat squares_;

      // The vector holding the outline points
      // of the square in the square_ image
      std::vector< cv::Point2f > square_outline_points_vector_;

      // The vector holding the vector of outline points
      // of the squares in the squares_ image
      std::vector< std::vector< cv::Point2f > > squares_outline_points_vector_;

  };


  //! Test BlobDetection::brushfireKeypoint()
  TEST_F ( BlobDetectionTest, BrushfireKeypointTest )
  {
    /***************************************************************************
     * Test square_
     **************************************************************************/

    cv::KeyPoint k ( 150, 200, 1 );

    std::vector< cv::Point2f > blobOutlineVector;
    float blobArea = 0.0;

    // Run BlobDetection::brushfireKeypoint
    BlobDetection::brushfireKeypoint
      ( k, &square_, &blobOutlineVector, &blobArea );

    // As a preliminary test, check if the number of outline points found
    // is equal to the one it should be
    ASSERT_EQ ( square_outline_points_vector_.size(), blobOutlineVector.size() );

    // Check whether the outline points found are actually the outline points
    // of the square in square_
    int count_b_in_s = 0;
    for ( int b = 0; b < blobOutlineVector.size(); b++ )
    {
      for ( int s = 0; s < square_outline_points_vector_.size(); s++ )
      {
        if (blobOutlineVector[b].x == square_outline_points_vector_[s].x
        && blobOutlineVector[b].y == square_outline_points_vector_[s].y)
        {
          count_b_in_s++;
        }
      }
    }

    EXPECT_EQ ( square_outline_points_vector_.size(), count_b_in_s );

  }

}  // namespace pandora_vision
