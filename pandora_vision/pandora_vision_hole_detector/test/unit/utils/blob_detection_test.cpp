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

      //! Sets up two images: square_, which features a single non-zero value
      //! square of size 100 with its upper left vertex at (100, 100),
      //! and squares_, which features two non-zero value squares of size 100.
      //! The first one (order matters here) has its upper left vertex at
      //! (100, 100) and the second one its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1)
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // A square with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
        square_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        // Two squares with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
        // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
        // B' (WIDTH - 1 - 100, HEIGHT - 1)
        // C' (WIDTH - 1, HEIGHT - 1)
        // D' (WIDTH - 1, HEIGHT - 1 - 100)
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
              squareOutlinePointsVector_.push_back
                ( cv::Point2f ( cols, rows) );
            }
          }
        }

        // The number of actual outline points of the square should be
        // 4 x 100 - 4
        ASSERT_EQ ( 396, squareOutlinePointsVector_.size() );


        // Construct the squares_ image

        // Construct the lower right square
        cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);

        for ( int cols = WIDTH - 100; cols < WIDTH; cols++)
        {
          lowerRightSquare.at< unsigned char >( HEIGHT - 100, cols ) = 255;
          lowerRightSquare.at< unsigned char >( HEIGHT - 1, cols ) = 255;
        }


        for ( int rows = HEIGHT - 100; rows < HEIGHT ; rows++)
        {
          lowerRightSquare.at< unsigned char >( rows, WIDTH - 100 ) = 255;
          lowerRightSquare.at< unsigned char >( rows, WIDTH - 1 ) = 255;
        }

        std::vector< cv::Point2f > lowerRightSquareOutline;
        for ( int rows = 0; rows < lowerRightSquare.rows; rows++ )
        {
          for ( int cols = 0; cols < lowerRightSquare.cols; cols++ )
          {
            if ( lowerRightSquare.at< unsigned char>( rows, cols ) != 0)
            {
              lowerRightSquareOutline.push_back ( cv::Point2f ( cols, rows ) );
            }
          }
        }


        // Add the vector of outline points of the upper left square to the
        // vector holding the vectors of outline points of the squares_ image
        squaresOutlinePointsVector_.push_back ( squareOutlinePointsVector_ );

        // Add the vector of outline points of the lower right square to the
        // vector holding the vectors of outline points of the squares_ image
        squaresOutlinePointsVector_.push_back( lowerRightSquareOutline );

        // Add the upper left square and the lower right square
        squares_ = lowerRightSquare + square_;

        // The number of actual outline points of the squares should be
        // 4 x 100 - 4
        ASSERT_EQ( 396 , squaresOutlinePointsVector_[0].size() );
        ASSERT_EQ( 396 , squaresOutlinePointsVector_[1].size() );
      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // A square with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
      cv::Mat square_;

      // Two squares with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
      // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
      // B' (WIDTH - 1 - 100, HEIGHT - 1)
      // C' (WIDTH - 1, HEIGHT - 1)
      // D' (WIDTH - 1, HEIGHT - 1 - 100)
      cv::Mat squares_;

      // The vector holding the outline points
      // of the square in the square_ image
      std::vector< cv::Point2f > squareOutlinePointsVector_;

      // The vector holding the vector of outline points
      // of the squares in the squares_ image
      std::vector< std::vector< cv::Point2f > > squaresOutlinePointsVector_;

  };



  //! Test BlobDetection::brushfireKeypoint()
  TEST_F ( BlobDetectionTest, BrushfireKeypointTest )
  {
    /***************************************************************************
     * Test square_
     **************************************************************************/

    cv::KeyPoint k ( 101, 101, 1 );

    std::vector< cv::Point2f > blobOutlineVector;
    float blobArea = 0.0;

    // Run BlobDetection::brushfireKeypoint
    BlobDetection::brushfireKeypoint
      ( k, &square_, &blobOutlineVector, &blobArea );

    // As a preliminary test, check if the number of outline points found
    // is equal to the one it should be. The four vertices of the square are not
    // included in the square's outline due to the cross-expanding nature of the
    // brushfire algorithm
    ASSERT_EQ ( squareOutlinePointsVector_.size() - 4,
      blobOutlineVector.size() );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, blobArea );

    // Check whether the outline points found are actually the outline points
    // of the square in square_
    int count_b_in_s = 0;
    for ( int b = 0; b < blobOutlineVector.size(); b++ )
    {
      for ( int s = 0; s < squareOutlinePointsVector_.size(); s++ )
      {
        if (blobOutlineVector[b].x == squareOutlinePointsVector_[s].x
        && blobOutlineVector[b].y == squareOutlinePointsVector_[s].y)
        {
          count_b_in_s++;
        }
      }
    }

    EXPECT_EQ ( squareOutlinePointsVector_.size() - 4, count_b_in_s );
  }



  //! Test BlobDetection::brushfireKeypoints()
  TEST_F ( BlobDetectionTest, BrushfireKeypointsTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    cv::KeyPoint k_0 ( 150, 101, 1 );
    cv::KeyPoint k_1 ( WIDTH - 43, HEIGHT - 35, 1 );

    // Push_back the two keypoints
    std::vector<cv::KeyPoint> inKeyPoints;

    inKeyPoints.push_back( k_0 );
    inKeyPoints.push_back( k_1 );

    std::vector< std::vector< cv::Point2f > > blobsOutlineVector;
    std::vector< float > blobsArea;

    // Run BlobDetection::brushfireKeypoints
    BlobDetection::brushfireKeypoints
      ( inKeyPoints, &squares_, &blobsOutlineVector, &blobsArea );

    // As a preliminary test, check if the number of outline points found
    // is equal to the one it should be
    ASSERT_EQ ( squaresOutlinePointsVector_[0].size() - 4,
      blobsOutlineVector[0].size() );

    ASSERT_EQ ( squaresOutlinePointsVector_[1].size() - 4,
      blobsOutlineVector[1].size() );


    for ( int sq = 0; sq < 2; sq++ )
    {
      // Check whether the outline points found are actually the outline points
      // of the sq-th square in square_
      int count_b_in_s = 0;
      for ( int b = 0; b < blobsOutlineVector[sq].size(); b++ )
      {
        for ( int s = 0; s < squaresOutlinePointsVector_[sq].size(); s++ )
        {
          if (blobsOutlineVector[sq][b].x ==
            squaresOutlinePointsVector_[sq][s].x
            && blobsOutlineVector[sq][b].y ==
            squaresOutlinePointsVector_[sq][s].y)
          {
            count_b_in_s++;
          }
        }
      }

      EXPECT_EQ ( squaresOutlinePointsVector_[sq].size() - 4, count_b_in_s );

      // The square's area should be the number of visited points of the
      // brushfire algorithm, which,
      // excluding the square's four vertices, is 100 x 100 - 4
      EXPECT_EQ ( 9996, blobsArea[sq] );
    }
  }



  //! Test BlobDetection::brushfirePoint()
  TEST_F ( BlobDetectionTest, BrushfirePointTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    cv::Point2f p_0 ( 150, 101 );
    cv::Point2f p_1 ( WIDTH - 43, HEIGHT - 35 );

    // The sets of visted points for each square
    std::set< unsigned int > visited_0;
    std::set< unsigned int > visited_1;


    // Run BlobDetection::brushfirePoint for the upper left square
    BlobDetection::brushfirePoint ( p_0, &squares_, &visited_0 );

    // The number of visited points should be the number of visited points
    // of the brushfire algorithm, which,
    // excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, visited_0.size() );


    // Run BlobDetection::brushfirePoint for the upper left square
    BlobDetection::brushfirePoint ( p_1, &squares_, &visited_1 );

    // The number of visited points should be the number of visited points
    // of the brushfire algorithm, which,
    // excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, visited_1.size() );
  }



  //! Test BlobDetection::raycastKeypoint
  TEST_F ( BlobDetectionTest, RaycastKeypointTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    // The keypoint of the lower right square
    cv::KeyPoint k_1 ( WIDTH - 43, HEIGHT - 35, 1 );

    // The vector of outline points
    std::vector< cv::Point2f > blobOutlineVector_1;

    //Run BlobDetection::raycastKeypoint
    BlobDetection::raycastKeypoint
      ( k_1, &squares_, 360, &blobOutlineVector_1 );

    // Due to the approximate nature of the raycastKeypoint algorithm,
    // the number of outline points found should be smaller or equal to the
    // actual number of outline points of the square
    EXPECT_GE ( squaresOutlinePointsVector_[1].size(),
      blobOutlineVector_1.size() );


    // The keypoint of the upper left square
    cv::KeyPoint k_0 ( 101, 103, 1 );

    // The vector of outline points
    std::vector< cv::Point2f > blobOutlineVector_0;

    //Run BlobDetection::raycastKeypoint
    BlobDetection::raycastKeypoint
      ( k_0, &squares_, 360, &blobOutlineVector_0 );

    // Due to the approximate nature of the raycastKeypoint algorithm,
    // the number of outline points found should be smaller or equal to the
    // actual number of outline points of the square
    EXPECT_GE ( squaresOutlinePointsVector_[0].size(),
      blobOutlineVector_0.size() );
  }



  //! Test BlobDetection::raycastKeypoints
  TEST_F ( BlobDetectionTest, RaycastKeypointsTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    // The keypoint of the upper left square
    cv::KeyPoint k_0 ( 150, 103, 1 );

    // The keypoint of the lower right square
    cv::KeyPoint k_1 ( WIDTH - 43, HEIGHT - 35, 1 );

    // The vector of keypoints
    std::vector<cv::KeyPoint> inKeyPoints;

    // Push back the two keypoints
    inKeyPoints.push_back ( k_0 );
    inKeyPoints.push_back ( k_1 );

    // The vector of outline points
    std::vector< std::vector< cv::Point2f > > blobsOutlineVector;

    // The vector of blobs' areas
    std::vector<float> blobsArea;

    //Run BlobDetection::raycastKeypoints
    BlobDetection::raycastKeypoints
      ( &inKeyPoints, &squares_, 360, &blobsOutlineVector, &blobsArea );


    for ( int sq = 0; sq < 2; sq++ )
    {
      // Due to the approximate nature of the raycastKeypoint algorithm,
      // the number of outline points found should be smaller or equal to the
      // actual number of outline points of the square
      EXPECT_GE ( squaresOutlinePointsVector_[sq].size(),
        blobsOutlineVector[sq].size() );

      EXPECT_GE ( 10000, blobsArea[sq] );
    }
  }

}  // namespace pandora_vision
