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

#include "rgb_node/utils/outline_discovery.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace rgb
{
  /**
    @class OutlineDiscoveryTest
    @brief Tests the integrity of methods of class OutlineDetecion
   **/
  class OutlineDiscoveryTest : public ::testing::Test
  {
    protected:

      OutlineDiscoveryTest() {}

      /**
        @brief Constructs a rectangle of width @param x and height of @param y
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted on
        return void
       **/
      void generateRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        cv::Mat* image );

      //! Sets up three images: square_, which features a single non-zero value
      //! square of size 100 with its upper left vertex at (100, 100),
      //! squares_, which features two non-zero value squares of size 100.
      //! The first one (order matters here) has its upper left vertex at
      //! ( 100, 100 ) and the second one its lower right vertex at
      //! ( WIDTH - 1, HEIGHT - 1 )
      //! and corners_ which features two semi-closed squares: one with its
      //! lower right vertex at ( 99, 99 ) and one with its upper right vertex
      //! at ( HEIGHT - 99, WIDTH -99 )
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // A square with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
        square_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        // Construct the square_ image
        OutlineDiscoveryTest::generateRectangle
          ( cv::Point2f ( 100, 100 ), 100, 100, &square_ );

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


        // Two squares with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
        // A' (WIDTH - 100, HEIGHT - 100),
        // B' (WIDTH - 100, HEIGHT - 1)
        // C' (WIDTH - 1, HEIGHT - 1)
        // D' (WIDTH - 1, HEIGHT - 100)
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

        // Construct the squares_ image

        // Construct the lower right square
        cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);

        OutlineDiscoveryTest::generateRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100,
            &lowerRightSquare );


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
        ASSERT_EQ( 396, squaresOutlinePointsVector_[0].size() );
        ASSERT_EQ( 396, squaresOutlinePointsVector_[1].size() );

        ASSERT_EQ( HEIGHT, square_.rows );
        ASSERT_EQ( WIDTH, square_.cols );

        // Construct image corners_
        corners_ = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );

        for ( int rows = 0; rows < 99; rows++ )
        {
          corners_.at< unsigned char >( rows, 99 ) = 255;
        }

        for ( int cols = 0; cols < 99; cols++ )
        {
          corners_.at< unsigned char >( 99, cols ) = 255;
        }

        for ( int rows = HEIGHT - 100; rows < HEIGHT; rows++ )
        {
          corners_.at< unsigned char >( rows, WIDTH - 100 ) = 255;
        }

        for ( int cols = WIDTH - 100; cols < WIDTH; cols++ )
        {
          corners_.at< unsigned char >( HEIGHT - 100, cols ) = 255;
        }
      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // A square with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100)
      cv::Mat square_;

      // Two squares with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
      // A' (WIDTH - 100, HEIGHT - 100),
      // B' (WIDTH - 100, HEIGHT - 1)
      // C' (WIDTH - 1, HEIGHT - 1)
      // D' (WIDTH - 1, HEIGHT - 100)
      cv::Mat squares_;

      // Two semi-closed squares
      cv::Mat corners_;

      // The vector holding the outline points
      // of the square in the square_ image
      std::vector< cv::Point2f > squareOutlinePointsVector_;

      // The vector holding the vectors of outline points
      // of the squares in the squares_ image
      std::vector< std::vector< cv::Point2f > > squaresOutlinePointsVector_;

  };



  /**
    @brief Constructs a rectangle of width @param x and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on
    return void
   **/
  void OutlineDiscoveryTest::generateRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    cv::Mat* image )
  {
    // The four vertices of the rectangle
    cv::Point2f vertex_1(upperLeft.x, upperLeft.y);

    cv::Point2f vertex_2(upperLeft.x, upperLeft.y + y - 1);

    cv::Point2f vertex_3(upperLeft.x + x - 1, upperLeft.y + y - 1);

    cv::Point2f vertex_4(upperLeft.x + x - 1, upperLeft.y);

    cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

    for(unsigned int j = 0; j < 4; j++)
    {
      cv::line(*image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
    }
  }



  //! Tests OutlineDiscovery::brushfireKeypoint()
  TEST_F ( OutlineDiscoveryTest, brushfireKeypointTest )
  {
    /***************************************************************************
     * Test square_
     **************************************************************************/

    cv::KeyPoint ks ( 101, 101, 1 );

    std::vector< cv::Point2f > blobOutlineVectorSquare;
    float blobArea = 0.0;

    // Run OutlineDiscovery::brushfireKeypoint
    OutlineDiscovery::brushfireKeypoint
      ( ks, &square_, &blobOutlineVectorSquare, &blobArea );

    // As a preliminary test, check if the number of outline points found
    // is equal to the one it should be. The four vertices of the square are not
    // included in the square's outline due to the cross-expanding nature of the
    // brushfire algorithm
    ASSERT_EQ ( squareOutlinePointsVector_.size() - 4,
      blobOutlineVectorSquare.size() );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, blobArea );

    // Check whether the outline points found are actually the outline points
    // of the square in square_
    int count_b_in_s = 0;
    for ( int b = 0; b < blobOutlineVectorSquare.size(); b++ )
    {
      for ( int s = 0; s < squareOutlinePointsVector_.size(); s++ )
      {
        if (blobOutlineVectorSquare[b].x == squareOutlinePointsVector_[s].x
          && blobOutlineVectorSquare[b].y == squareOutlinePointsVector_[s].y)
        {
          count_b_in_s++;
        }
      }
    }

    EXPECT_EQ ( squareOutlinePointsVector_.size() - 4, count_b_in_s );


    /***************************************************************************
     * Test corners_
     **************************************************************************/

    cv::KeyPoint kc1 ( 5, 5, 1 );

    std::vector< cv::Point2f > blobOutlineVectorCorner1;
    blobArea = 0.0;

    // Run OutlineDiscovery::brushfireKeypoint
    OutlineDiscovery::brushfireKeypoint
      ( kc1, &corners_, &blobOutlineVectorCorner1, &blobArea );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 1
    // vertex (the lower right one)
    EXPECT_EQ ( 9999, blobArea );

    cv::KeyPoint kc2 ( WIDTH - 50, HEIGHT - 50, 1 );

    std::vector< cv::Point2f > blobOutlineVectorCorner2;
    blobArea = 0.0;

    // Run OutlineDiscovery::brushfireKeypoint
    OutlineDiscovery::brushfireKeypoint
      ( kc2, &corners_, &blobOutlineVectorCorner2, &blobArea );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 1
    // vertex (the lower right one)
    EXPECT_EQ ( 9999, blobArea );
  }



  //! Tests OutlineDiscovery::brushfireKeypoints()
  TEST_F ( OutlineDiscoveryTest, brushfireKeypointsTest )
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

    // Run OutlineDiscovery::brushfireKeypoints
    OutlineDiscovery::brushfireKeypoints
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



  //! Tests OutlineDiscovery::brushfirePoint()
  TEST_F ( OutlineDiscoveryTest, brushfirePointTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    cv::Point2f p_0 ( 150, 101 );
    cv::Point2f p_1 ( WIDTH - 43, HEIGHT - 35 );

    // The sets of visted points for each square
    std::set< unsigned int > visited_0;
    std::set< unsigned int > visited_1;


    // Run OutlineDiscovery::brushfirePoint for the upper left square
    OutlineDiscovery::brushfirePoint ( p_0, &squares_, &visited_0 );

    // The number of visited points should be the number of visited points
    // of the brushfire algorithm, which,
    // excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, visited_0.size() );


    // Run OutlineDiscovery::brushfirePoint for the upper left square
    OutlineDiscovery::brushfirePoint ( p_1, &squares_, &visited_1 );

    // The number of visited points should be the number of visited points
    // of the brushfire algorithm, which,
    // excluding the square's four vertices, is 100 x 100 - 4
    EXPECT_EQ ( 9996, visited_1.size() );


    /***************************************************************************
     * Test corners_
     **************************************************************************/

    cv::Point2f kc1 ( 5, 5 );

    std::set< unsigned int > visited;

    // Run OutlineDiscovery::brushfireKeypoint
    OutlineDiscovery::brushfirePoint
      ( kc1, &corners_, &visited );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 1
    // vertex (the lower right one)
    EXPECT_EQ ( 9999, visited.size() );


    cv::Point2f kc2 ( WIDTH - 50, HEIGHT - 50);

    visited.erase(visited.begin(), visited.end());

    // Run OutlineDiscovery::brushfireKeypoint
    OutlineDiscovery::brushfirePoint
      ( kc2, &corners_, &visited );

    // The square's area should be the number of visited points of the brushfire
    // algorithm, which, excluding the square's four vertices, is 100 x 100 - 1
    // vertex (the lower right one)
    EXPECT_EQ ( 9999, visited.size() );
  }



  //! Tests OutlineDiscovery::getShapesClearBorder
  TEST_F ( OutlineDiscoveryTest, getShapesClearBorderTest )
  {
    // Construct two squares, one within the other
    cv::Mat squares = cv::Mat::zeros ( squares_.size(), CV_8UC1 );

    OutlineDiscoveryTest::generateRectangle
      ( cv::Point( 10, 10 ), 200, 200, &squares );

    OutlineDiscoveryTest::generateRectangle
      ( cv::Point( 100, 100 ), 100, 100, &squares );

    // The number of non-zero pixels before getting the clear borders
    int nonZerosBefore = cv::countNonZero ( squares );

    // Run EdgeDetection::getShapesClearBorder
    OutlineDiscovery::getShapesClearBorder ( &squares );

    // The number of non-zero pixels after getting the clear borders
    int nonZerosAfter = cv::countNonZero ( squares );

    // The EdgeDetection::getShapesClearBorder method finds all borders,
    // not caring about shapes being inside other shapes
    EXPECT_EQ ( nonZerosBefore, nonZerosAfter );

  }



  //! Tests OutlineDiscovery::getShapesClearBorderSimple
  TEST_F ( OutlineDiscoveryTest, getShapesClearBorderSimpleTest )
  {
    // Construct two squares, one within the other
    cv::Mat squares = cv::Mat::zeros ( squares_.size(), CV_8UC1 );

    OutlineDiscoveryTest::generateRectangle
      ( cv::Point( 10, 10 ), 200, 200, &squares );

    // The number of non-zero pixels of the shape that encapsulates the one
    // below
    int nonZerosBefore = cv::countNonZero ( squares );

    OutlineDiscoveryTest::generateRectangle
      ( cv::Point( 100, 100 ), 100, 100, &squares );


    // Run EdgeDetection::getShapesClearBorderSimple
    OutlineDiscovery::getShapesClearBorderSimple ( &squares );

    // The number of non-zero pixels after getting the clear borders
    int nonZerosAfter = cv::countNonZero ( squares );

    // The EdgeDetection::getShapesClearBorderSimple method finds borders of
    // shapes, discarding everything within them: the square does not get
    // to be detected :(
    EXPECT_EQ ( nonZerosBefore, nonZerosAfter );

  }



  //! Tests OutlineDiscovery::raycastKeypoint
  TEST_F ( OutlineDiscoveryTest, raycastKeypointTest )
  {
    /***************************************************************************
     * Test squares_
     **************************************************************************/

    // The keypoint of the lower right square
    cv::KeyPoint k_1 ( WIDTH - 43, HEIGHT - 35, 1 );

    // The vector of outline points
    std::vector< cv::Point2f > blobOutlineVector_1;

    // The blob's area
    float area = 0.0;

    //Run OutlineDiscovery::raycastKeypoint
    OutlineDiscovery::raycastKeypoint
      ( k_1, &squares_, 360, false, &blobOutlineVector_1, &area );

    // Due to the approximate nature of the raycastKeypoint algorithm,
    // the number of outline points found should be smaller or equal to the
    // actual number of outline points of the square
    EXPECT_GE ( squaresOutlinePointsVector_[1].size(),
      blobOutlineVector_1.size() );


    // The keypoint of the upper left square
    cv::KeyPoint k_0 ( 101, 103, 1 );

    // The vector of outline points
    std::vector< cv::Point2f > blobOutlineVector_0;

    //Run OutlineDiscovery::raycastKeypoint
    OutlineDiscovery::raycastKeypoint
      ( k_0, &squares_, 360, false, &blobOutlineVector_0, &area );

    // Due to the approximate nature of the raycastKeypoint algorithm,
    // the number of outline points found should be smaller or equal to the
    // actual number of outline points of the square
    EXPECT_GE ( squaresOutlinePointsVector_[0].size(),
      blobOutlineVector_0.size() );

  }



  //! Tests OutlineDiscovery::raycastKeypoints
  TEST_F ( OutlineDiscoveryTest, raycastKeypointsTest )
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

    //Run OutlineDiscovery::raycastKeypoints
    OutlineDiscovery::raycastKeypoints
      ( inKeyPoints, &squares_, 360, &blobsOutlineVector, &blobsArea );


    for ( int sq = 0; sq < 2; sq++ )
    {
      // Due to the approximate nature of the raycastKeypoint algorithm,
      // the number of outline points found should be smaller or equal to the
      // actual number of outline points of the square
      EXPECT_GE ( squaresOutlinePointsVector_[sq].size(),
        blobsOutlineVector[sq].size() );

      EXPECT_GE ( 10000, blobsArea[sq] );
    }


    /***************************************************************************
     * Test corners_
     **************************************************************************/

    // Clear the keypoints vector
    inKeyPoints.clear();

    // Place a new keypoint in the keypoints vector
    cv::KeyPoint kc1 ( 5, 5, 1 );
    cv::KeyPoint kc2 ( WIDTH - 50, HEIGHT - 50, 1 );

    inKeyPoints.push_back( kc1 );
    inKeyPoints.push_back( kc2 );

    // Clear the outline vector
    blobsOutlineVector.clear();

    // Clear the areas vector
    blobsArea.clear();

    //Run OutlineDiscovery::raycastKeypoints
    OutlineDiscovery::raycastKeypoints
      ( inKeyPoints, &corners_, 360, &blobsOutlineVector, &blobsArea );

    // There will be exactly one keypoint, although the rays hit the edges
    // of the image
    EXPECT_EQ ( 2, blobsOutlineVector.size() );

    for ( int i = 0; i < 2; i++ )
    {
      // Approximately, the area of each square will be more than 9500 px2,
      // but less than 10000 px2
      EXPECT_LT ( 9500, blobsArea[i] );
      EXPECT_GT ( 10000, blobsArea[i] );
    }

  }

}  // rgb
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
