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

#include "utils/edge_detection.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class EdgeDetectionTest
    @brief Tests the integrity of methods of class EdgeDetection
   **/
  class EdgeDetectionTest : public ::testing::Test
  {
    protected:

      EdgeDetectionTest () {}

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
      void generateRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        cv::Mat* image );

      //! Sets up one image: squares_, which features two non-zero value squares
      //! of size 100. The first one (order matters here) has its upper left
      //! vertex at (100, 100) and the second one its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1). The rest of the image's pixels are with a
      //! value of 60
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Two squares with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
        // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
        // B' (WIDTH - 1 - 100, HEIGHT - 1)
        // C' (WIDTH - 1, HEIGHT - 1)
        // D' (WIDTH - 1, HEIGHT - 1 - 100)
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );


        // The two square outlines
        cv::Mat upperLeftSquare = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        cv::Mat lowerRightSquare = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );

        // Construct the upperLeftSquare image
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            upperLeftSquare.at < unsigned char >( rows, cols ) = 30;
          }
        }

        EdgeDetectionTest::generateRectangle
          ( cv::Point2f ( 100, 100 ), 100, 100, &upperLeftSquare );

        // Locate the outline points of the square in the upperLeftSquare image
        std::vector< cv::Point2f > upperLeftSquareOutlinePointsVector_;

        for ( int rows = 0; rows < upperLeftSquare.rows; rows++ )
        {
          for ( int cols = 0; cols < upperLeftSquare.cols; cols++ )
          {
            if ( upperLeftSquare.at< unsigned char >( rows, cols ) == 255 )
            {
              upperLeftSquareOutlinePointsVector_.push_back
                ( cv::Point2f ( cols, rows) );
            }
          }
        }


        // Construct the lower right square
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            lowerRightSquare.at < unsigned char >( rows, cols ) = 30;
          }
        }

        EdgeDetectionTest::generateRectangle
          ( cv::Point2f ( WIDTH - 1 - 100, HEIGHT - 1 - 100),
            100,
            100,
            &lowerRightSquare );


        // Locate the outline points of the square in the upperLeftSquare image
        std::vector< cv::Point2f > lowerRightSquareOutlinePointsVector_;

        for ( int rows = 0; rows < lowerRightSquare.rows; rows++ )
        {
          for ( int cols = 0; cols < lowerRightSquare.cols; cols++ )
          {
            if ( lowerRightSquare.at< unsigned char>( rows, cols ) == 255 )
            {
              lowerRightSquareOutlinePointsVector_.push_back
                ( cv::Point2f ( cols, rows ) );
            }
          }
        }


        // Add the vector of outline points of the upper left square to the
        // vector holding the vectors of outline points of the squares_ image
        squaresOutlinePointsVector_.push_back
          ( upperLeftSquareOutlinePointsVector_ );

        // Add the vector of outline points of the lower right square to the
        // vector holding the vectors of outline points of the squares_ image
        squaresOutlinePointsVector_.push_back
          ( lowerRightSquareOutlinePointsVector_ );

        // Add the upper left square and the lower right square
        squares_ = lowerRightSquare + upperLeftSquare;

        // The number of actual outline points of the squares should be
        // 4 x 100 - 4
        ASSERT_EQ( 396 , squaresOutlinePointsVector_[0].size() );
        ASSERT_EQ( 396 , squaresOutlinePointsVector_[1].size() );


        ASSERT_EQ( HEIGHT, squares_.rows );
        ASSERT_EQ( WIDTH, squares_.cols );
      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // Two squares with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100) and
      // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
      // B' (WIDTH - 1 - 100, HEIGHT - 1)
      // C' (WIDTH - 1, HEIGHT - 1)
      // D' (WIDTH - 1, HEIGHT - 1 - 100)
      cv::Mat squares_;

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
  void EdgeDetectionTest::generateRectangle (
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



  //! Test EdgeDetection::applyCanny
  TEST_F ( EdgeDetectionTest, ApplyCannyTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyCanny ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );
  }



  //! Test EdgeDetection::applyScharr
  TEST_F ( EdgeDetectionTest, ApplyScharrTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyScharr ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );
  }



  //! Test EdgeDetection::applySobel
  TEST_F ( EdgeDetectionTest, ApplySobelTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applySobel ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );
  }



  //! Test EdgeDetection::applyLaplacian
  TEST_F ( EdgeDetectionTest, ApplyLaplacianTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyLaplacian ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );
  }



  //! Test EdgeDetection::applyEdgeContamination
  TEST_F ( EdgeDetectionTest, ApplyEdgeContaminationTest)
  {
    // Obtain the edges image for the squares_ image.
    // Here, it does not matter with which operator the edges image is produced,
    // provided that the the value tested below changes accordingly
    cv::Mat squares_edges;
    EdgeDetection::applyLaplacian ( squares_, &squares_edges );

    // The number of non-zero pixels before the appliance of edge contamination
    EXPECT_EQ ( 2 * ( 3 * 396 - 4 ) , cv::countNonZero ( squares_edges ) );

    EdgeDetection::applyEdgeContamination ( &squares_edges );

    // The number of non-zero pixels after the appliance of edge contamination
    EXPECT_EQ ( 3 * 396 - 4, cv::countNonZero ( squares_edges ) );
  }



  //! Test EdgeDetection::computeDepthEdges
  TEST_F ( EdgeDetectionTest, ComputeDepthEdgesTest )
  {
    // Convert squares_ into a CV_32FC1 type image
    cv::Mat squares_32FC1 = cv::Mat::zeros ( squares_.size(), CV_32FC1 );

    for ( int rows = 0; rows < squares_.rows; rows++ )
    {
      for ( int cols = 0; cols < squares_.cols; cols++ )
      {
        squares_32FC1.at< float >( rows, cols ) =
          static_cast< float >(squares_.at< unsigned char >( rows, cols )) / 255;
      }
    }

    // Add an unfinished square to the squares_32FC1 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_32FC1.at< float >( rows, 300 ) = 2.0;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_32FC1.at< float >( 300, cols ) = 2.0;
    }

    cv::Mat denoisedEdges;
    EdgeDetection::computeDepthEdges ( squares_32FC1, &denoisedEdges );

    ASSERT_EQ ( CV_8UC1, denoisedEdges.type() );
  }



  //! Test EdgeDetection::connectPairs
  TEST_F ( EdgeDetectionTest, ConnectPairsTest )
  {
    // Obtain the edges image for the squares_ image.
    // Here, it does not matter with which operator the edges image is produced,
    // provided that the the value tested below changes accordingly
    cv::Mat squares_edges;
    EdgeDetection::applyLaplacian ( squares_, &squares_edges );

    // Add an unfinished square to the squares_edges image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_edges.at< unsigned char >( rows, 300 ) = 255;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_edges.at< unsigned char >( 300, cols ) = 255;
    }


    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p;

    p.first.x = 399;
    p.first.y = 300;

    p.second.x = 300;
    p.second.y = 399;

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs;
    pairs.push_back( p );


    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    int nonZerosBefore = cv::countNonZero( squares_edges );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &squares_edges, pairs, 1 );

    // The number of non-zero pixels after the connection of the two points
    int nonZerosAfter = cv::countNonZero( squares_edges );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );


    // Connect by line

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( squares_edges );

    // Connect the two points by line
    EdgeDetection::connectPairs ( &squares_edges, pairs, 0 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( squares_edges );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );
  }



  //! Test EdgeDetection::denoiseEdges
  TEST_F ( EdgeDetectionTest, DenoiseEdgesTest )
  {
    // Obtain the edges image for the squares_ image.
    // Here, it does not matter with which operator the edges image is produced,
    // provided that the the value tested below changes accordingly
    cv::Mat squares_edges;
    EdgeDetection::applyScharr( squares_, &squares_edges );

    // The number of non-zero pixels in the edges image, before denoising
    int nonZerosBefore = countNonZero ( squares_edges );

    // Add an unfinished square to the squares_edges image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_edges.at< unsigned char >( rows, 300 ) = 255;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_edges.at< unsigned char >( 300, cols ) = 255;
    }

    // Run EdgeDetection::denoiseEdges
    EdgeDetection::denoiseEdges( &squares_edges );

    // The number of non-zero pixels in the edges image, after denoising
    int nonZerosAfter = countNonZero ( squares_edges );

    // The type of the edges image should be CV_8UC1
    ASSERT_EQ ( squares_edges.type(), CV_8UC1 );

    // In this particular test, where the lower right square vanishes due to
    // appliance of the edge contamination method, and the unfinished square's
    // end points are connected, the number of non-zero pixels after the
    // denoising of the edges image should amount to lower than that of before
    EXPECT_GT ( nonZerosBefore, nonZerosAfter );
  }



  //! Test EdgeDetection::findNeighs
  TEST_F ( EdgeDetectionTest, FindNeighsTest )
  {
    // A gamma shape
    cv::Mat gamma = cv::Mat::zeros( squares_.size(), CV_8UC1 );

    for ( int rows = 300; rows < 400; rows++ )
    {
      gamma.at< unsigned char >( rows, 300 ) = 255;
    }

    for ( int cols = 300; cols < 500; cols++ )
    {
      gamma.at< unsigned char >( 300, cols ) = 255;
    }

    std::set< unsigned int > ret;

    // Find the end points of a curve where a point trully lies on
    std::pair< GraphNode, GraphNode > p_valid =
      EdgeDetection::findNeighs ( &gamma, 300, 451, &ret);

    // The point should lie on a curve.
    // The curve should indeed be a curve: it is constituted by points
    EXPECT_LT ( 0, ret.size() );

    // The first end point's coordinates
    EXPECT_EQ ( p_valid.first.x, 300 );
    EXPECT_EQ ( p_valid.first.y, 499 );

    // The second end point's coordinates
    EXPECT_EQ ( p_valid.second.x, 399 );
    EXPECT_EQ ( p_valid.second.y, 300 );


    ret.clear();

    // Find the end points of a curve where a point trully lies on
    std::pair< GraphNode, GraphNode > p_invalid =
      EdgeDetection::findNeighs ( &gamma, 100, 200, &ret);

    // The point should not lie on a curve.
    EXPECT_EQ ( 1, ret.size() );

    // The first end point's coordinates will be the origin
    EXPECT_EQ ( p_invalid.first.x, 0 );
    EXPECT_EQ ( p_invalid.first.y, 0 );

    // The second end point's coordinates will be also be the origin
    EXPECT_EQ ( p_invalid.second.x, 0 );
    EXPECT_EQ ( p_invalid.second.y, 0 );

  }



  //! Test EdgeDetection::getShapesClearBorder
  TEST_F ( EdgeDetectionTest, GetShapesClearBorderTest )
  {
    // Construct two squares, one within the other
    cv::Mat squares = cv::Mat::zeros ( squares_.size(), CV_8UC1 );

    EdgeDetectionTest::generateRectangle
      ( cv::Point( 10, 10 ), 200, 200, &squares );

    EdgeDetectionTest::generateRectangle
      ( cv::Point( 100, 100 ), 100, 100, &squares );

    // The number of non-zero pixels before getting the clear borders
    int nonZerosBefore = cv::countNonZero ( squares );

    // Run EdgeDetection::getShapesClearBorder
    EdgeDetection::getShapesClearBorder ( &squares );

    // The number of non-zero pixels after getting the clear borders
    int nonZerosAfter = cv::countNonZero ( squares );

    // The EdgeDetection::getShapesClearBorder method finds all borders,
    // not caring about shapes being inside other shapes
    EXPECT_EQ ( nonZerosBefore, nonZerosAfter );

  }



  //! Test EdgeDetection::getShapesClearBorderSimple
  TEST_F ( EdgeDetectionTest, GetShapesClearBorderSimpleTest )
  {
    // Construct two squares, one within the other
    cv::Mat squares = cv::Mat::zeros ( squares_.size(), CV_8UC1 );

    EdgeDetectionTest::generateRectangle
      ( cv::Point( 10, 10 ), 200, 200, &squares );

    // The number of non-zero pixels of the shape that encapsulates the one
    // below
    int nonZerosBefore = cv::countNonZero ( squares );

    EdgeDetectionTest::generateRectangle
      ( cv::Point( 100, 100 ), 100, 100, &squares );


    // Run EdgeDetection::getShapesClearBorderSimple
    EdgeDetection::getShapesClearBorderSimple ( &squares );

    // The number of non-zero pixels after getting the clear borders
    int nonZerosAfter = cv::countNonZero ( squares );

    // The EdgeDetection::getShapesClearBorderSimple method finds borders of
    // shapes, discarding everything within them: the square does not get
    // to be detected :(
    EXPECT_EQ ( nonZerosBefore, nonZerosAfter );

  }

} // namespace pandora_vision
