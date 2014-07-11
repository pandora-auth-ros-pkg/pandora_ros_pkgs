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

        // The edges of this square are not touching the image's borders.
        // Remove "- 1" for that.
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



  //! Tests EdgeDetection::applyCanny
  TEST_F ( EdgeDetectionTest, applyCannyTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyCanny ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );

    EXPECT_LT ( 0, cv::countNonZero( squares_edges ) );
  }



  //! Tests EdgeDetection::applyScharr
  TEST_F ( EdgeDetectionTest, applyScharrTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyScharr ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );

    EXPECT_LT ( 0, cv::countNonZero( squares_edges ) );
  }



  //! Tests EdgeDetection::applySobel
  TEST_F ( EdgeDetectionTest, applySobelTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applySobel ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );

    EXPECT_LT ( 0, cv::countNonZero( squares_edges ) );
  }



  //! Tests EdgeDetection::applyLaplacian
  TEST_F ( EdgeDetectionTest, applyLaplacianTest )
  {
    cv::Mat squares_edges;
    EdgeDetection::applyLaplacian ( squares_, &squares_edges );

    ASSERT_EQ ( CV_8UC1, squares_edges.type() );

    EXPECT_LT ( 0, cv::countNonZero( squares_edges ) );
  }



  //! Tests EdgeDetection::applyEdgeContamination
  TEST_F ( EdgeDetectionTest, applyEdgeContaminationTest)
  {
    // Modify the squares_ image. Add squares adjacent to the corners of it.

    // The edges of this square are not touching the image's borders.
    // Remove "- 1" for that.
    EdgeDetectionTest::generateRectangle
      ( cv::Point2f ( 1, 1 ),
        10,
        10,
        &squares_ );

    EdgeDetectionTest::generateRectangle
      ( cv::Point2f ( 1, HEIGHT - 1 - 10 ),
        10,
        10,
        &squares_ );

    EdgeDetectionTest::generateRectangle
      ( cv::Point2f ( WIDTH - 1 - 10, 1 ),
        10,
        10,
        &squares_ );

    // Uncomment for visual inspection
    //Visualization::show("Modified squares_ image", squares_, 0);

    // Obtain the edges image for the squares_ image.
    // Here, it does not matter with which operator the edges image is produced,
    // provided that the the value tested below changes accordingly
    cv::Mat squares_edges;
    EdgeDetection::applyLaplacian ( squares_, &squares_edges );

    // The number of non-zero pixels before the appliance of edge contamination
    EXPECT_EQ ( 2 * ( 3 * 396 - 4 ) + 3 * ( 3 * 36 - 4 ),
      cv::countNonZero ( squares_edges ) );

    // Uncomment for visual inspection
    //Visualization::show("Before calling applyEdgeContamination", squares_edges, 0);

    EdgeDetection::applyEdgeContamination ( &squares_edges );

    // Uncomment for visual inspection
    //Visualization::show("After calling applyEdgeContamination", squares_edges, 0);

    // The number of non-zero pixels after the appliance of edge contamination
    EXPECT_EQ ( 3 * 396 - 4, cv::countNonZero ( squares_edges ) );
  }



  //! Tests EdgeDetection::computeDepthEdges
  TEST_F ( EdgeDetectionTest, computeDepthEdgesTest )
  {
    // Traverse all available edge detectors
    for ( int p = 0; p < 5; p++ )
    {
      Parameters::Edge::edge_detection_method = p;

      // Test the toggle switch
      for ( int t = 0; t < 2; t++ )
      {
        Parameters::Edge::mixed_edges_toggle_switch = t;

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

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled("Before calling computeDepthEdges",
         * squares_32FC1, 0);
         */

        cv::Mat denoisedEdges;
        EdgeDetection::computeDepthEdges ( squares_32FC1, &denoisedEdges );

        // Uncomment for visual inspection
        //Visualization::show("After calling computeDepthEdges", denoisedEdges, 0);

        ASSERT_EQ ( CV_8UC1, denoisedEdges.type() );

        EXPECT_LT ( 0, cv::countNonZero( denoisedEdges ) );
      }
    }
  }



  //! Tests EdgeDetection::computeRgbEdges
  TEST_F ( EdgeDetectionTest, computeRgbEdgesTest)
  {
    // Convert squares_ into a CV_8UC3 image
    cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
    cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

    // Add an unfinished square to the squares_8UC3 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
    }

    // Uncomment for visual inspection
    //Visualization::show("Before calling computeRgbEdges", squares_8UC3, 0);

    // The final edges image
    // extractionMethod = 0
    cv::Mat denoisedEdges_0;

    // A dummy histogram
    cv::MatND histogram = cv::Mat::zeros ( squares_.size(), CV_8UC1 );

    // Run EdgeDetection::computeRgbEdges
    EdgeDetection::computeRgbEdges
      ( squares_8UC3, 0, histogram, &denoisedEdges_0 );

    EXPECT_LT ( 0, cv::countNonZero( denoisedEdges_0 ) );

    // Uncomment for visual inspection
    //Visualization::show("After calling computeRgbEdges 0", denoisedEdges_0, 0);

    // The final edges image
    // extractionMethod = 1
    cv::Mat denoisedEdges_1;

    // Run EdgeDetection::computeRgbEdges
    EdgeDetection::computeRgbEdges
      ( squares_8UC3, 1, histogram, &denoisedEdges_1 );

    // Because of the void histogram, the edges image is blank
    EXPECT_EQ ( 0, cv::countNonZero( denoisedEdges_1 ) );

    // Uncomment for visual inspection
    //Visualization::show("After calling computeRgbEdges 1", denoisedEdges_1, 0);

  }



  //! Tests EdgeDetection::connectPairs
  TEST_F ( EdgeDetectionTest, connectPairsTest )
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
    std::pair< GraphNode, GraphNode > p_1;

    p_1.first.x = 399;
    p_1.first.y = 300;

    p_1.second.x = 300;
    p_1.second.y = 399;

    std::pair< GraphNode, GraphNode > p_2;

    p_2.first.x = 300;
    p_2.first.y = 399;

    p_2.second.x = 399;
    p_2.second.y = 300;

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs;
    pairs.push_back( p_1 );
    pairs.push_back( p_2 );


    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    int nonZerosBefore = cv::countNonZero( squares_edges );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &squares_edges, pairs, 1 );

    // Uncomment for visual inspection
    //Visualization::show("squares_edges arc", squares_edges, 0);

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

    // Uncomment for visual inspection
    //Visualization::show("squares_edges line", squares_edges, 0);


    // Commence full blown connectPairs test.

    /////////////////////////// Features a U shape /////////////////////////////
    cv::Mat u = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    for ( int rows = 100; rows < 200; rows++ )
    {
      u.at< unsigned char >( rows, 100 ) = 255;
      u.at< unsigned char >( rows, 199 ) = 255;
    }

    for ( int cols = 100; cols < 200; cols++ )
    {
      u.at< unsigned char >( 199, cols ) = 255;
    }

    // Backup u
    cv::Mat u_backup;
    u.copyTo(u_backup);

    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p_u_1;

    p_u_1.first.x = 100;
    p_u_1.first.y = 100;

    p_u_1.second.x = 100;
    p_u_1.second.y = 199;


    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_u_1;
    pairs_u_1.push_back( p_u_1 );

    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( u );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &u, pairs_u_1, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( u );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("u arc forward", u , 0);

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_u_2;

    // Reverse the order of the graph nodes
    std::pair< GraphNode, GraphNode > p_u_2;

    p_u_2.first.x = 100;
    p_u_2.first.y = 199;

    p_u_2.second.x = 100;
    p_u_2.second.y = 100;

    pairs_u_2.push_back( p_u_2 );

    // Reset u
    u_backup.copyTo(u);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( u );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &u, pairs_u_2, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( u );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("u arc reverse", u , 0);

    // Connect by line

    // Reset u
    u_backup.copyTo(u);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( u );

    // Connect the two points by line
    EdgeDetection::connectPairs ( &u, pairs_u_1, 0 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( u );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("u line", u , 0);


    /////////////////////////// Features a C shape /////////////////////////////
    cv::Mat c = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    for ( int rows = 100; rows < 200; rows++ )
    {
      c.at< unsigned char >( rows, 100 ) = 255;
    }

    for ( int cols = 100; cols < 200; cols++ )
    {
      c.at< unsigned char >( 100, cols ) = 255;
      c.at< unsigned char >( 199, cols ) = 255;
    }

    // Backup c
    cv::Mat c_backup;
    c.copyTo(c_backup);

    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p_c_1;

    p_c_1.first.x = 100;
    p_c_1.first.y = 199;

    p_c_1.second.x = 199;
    p_c_1.second.y = 199;


    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_c_1;
    pairs_c_1.push_back( p_c_1 );

    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( c );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &c, pairs_c_1, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( c );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("c arc forward", c , 0);

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_c_2;

    // Reverse the order of the graph nodes
    std::pair< GraphNode, GraphNode > p_c_2;

    p_c_2.first.x = 199;
    p_c_2.first.y = 199;

    p_c_2.second.x = 100;
    p_c_2.second.y = 199;

    pairs_c_2.push_back( p_c_2 );

    // Reset c
    c_backup.copyTo(c);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( c );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &c, pairs_c_2, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( c );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("c arc reverse", c , 0);

    // Connect by line

    // Reset c
    c_backup.copyTo(c);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( c );

    // Connect the two points by line
    EdgeDetection::connectPairs ( &c, pairs_c_1, 0 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( c );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("c line", c , 0);


    /////////////////////////// Features a pi shape ////////////////////////////
    cv::Mat pi = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    for ( int rows = 100; rows < 200; rows++ )
    {
      pi.at< unsigned char >( rows, 100 ) = 255;
      pi.at< unsigned char >( rows, 199 ) = 255;
    }

    for ( int cols = 100; cols < 200; cols++ )
    {
      pi.at< unsigned char >( 100, cols ) = 255;
    }

    // Backup pi
    cv::Mat pi_backup;
    pi.copyTo(pi_backup);

    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p_pi_1;

    p_pi_1.first.x = 199;
    p_pi_1.first.y = 100;

    p_pi_1.second.x = 199;
    p_pi_1.second.y = 199;


    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_pi_1;
    pairs_pi_1.push_back( p_pi_1 );

    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( pi );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &pi, pairs_pi_1, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( pi );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("pi arc forward", pi , 0);

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_pi_2;

    // Reverse the order of the graph nodes
    std::pair< GraphNode, GraphNode > p_pi_2;

    p_pi_2.first.x = 199;
    p_pi_2.first.y = 199;

    p_pi_2.second.x = 199;
    p_pi_2.second.y = 100;

    pairs_pi_2.push_back( p_pi_2 );

    // Reset pi
    pi_backup.copyTo(pi);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( pi );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &pi, pairs_pi_2, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( pi );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("pi arc reverse", pi , 0);

    // Connect by line

    // Reset pi
    pi_backup.copyTo(pi);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( pi );

    // Connect the two points by line
    EdgeDetection::connectPairs ( &pi, pairs_pi_1, 0 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( pi );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("pi line", pi , 0);


    /////////////////////// Features a backwards C shape ///////////////////////
    cv::Mat bc = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    for ( int rows = 100; rows < 200; rows++ )
    {
      bc.at< unsigned char >( rows, 199 ) = 255;
    }

    for ( int cols = 100; cols < 200; cols++ )
    {
      bc.at< unsigned char >( 100, cols ) = 255;
      bc.at< unsigned char >( 199, cols ) = 255;
    }

    // Backup bc
    cv::Mat bc_backup;
    bc.copyTo(bc_backup);

    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p_bc_1;

    p_bc_1.first.x = 100;
    p_bc_1.first.y = 100;

    p_bc_1.second.x = 199;
    p_bc_1.second.y = 100;


    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_bc_1;
    pairs_bc_1.push_back( p_bc_1 );

    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( bc );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &bc, pairs_bc_1, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( bc );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("bc arc forward", bc , 0);

    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_bc_2;

    // Reverse the order of the graph nodes
    std::pair< GraphNode, GraphNode > p_bc_2;

    p_bc_2.first.x = 199;
    p_bc_2.first.y = 100;

    p_bc_2.second.x = 100;
    p_bc_2.second.y = 100;

    pairs_bc_2.push_back( p_bc_2 );

    // Reset bc
    bc_backup.copyTo(bc);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( bc );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &bc, pairs_bc_2, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( bc );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("bc arc reverse", bc , 0);

    // Connect by line

    // Reset bc
    bc_backup.copyTo(bc);

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( bc );

    // Connect the two points by line
    EdgeDetection::connectPairs ( &bc, pairs_bc_1, 0 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( bc );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("bc line", bc , 0);

    /////////////////////// Features a | | shape ///////////////////////
    // This is made to test that if no outline point is found,
    // the image is preserved and the points are not connected
    cv::Mat ii = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    for ( int rows = 100; rows < 200; rows++ )
    {
      ii.at< unsigned char >( rows, 100 ) = 255;
      ii.at< unsigned char >( rows, 199 ) = 255;
    }


    // Backup ii
    cv::Mat ii_backup;
    ii.copyTo(ii_backup);

    // Construct the pair to be connected: it is the two ends of the unfinished
    // square
    std::pair< GraphNode, GraphNode > p_ii_1;

    p_ii_1.first.x = 100;
    p_ii_1.first.y = 100;

    p_ii_1.second.x = 100;
    p_ii_1.second.y = 199;


    // Construct the vector of pairs
    std::vector< std::pair< GraphNode, GraphNode > > pairs_ii_1;
    pairs_ii_1.push_back( p_ii_1 );

    // Connect by arc

    // The number of non-zero pixels before the connection of the two points
    nonZerosBefore = cv::countNonZero( ii );

    // Connect the two points by arc
    EdgeDetection::connectPairs ( &ii, pairs_ii_1, 1 );

    // The number of non-zero pixels after the connection of the two points
    nonZerosAfter = cv::countNonZero( ii );

    // If the two points where connected, that means that there should be more
    // non-zero pixels after the connection
    EXPECT_EQ( nonZerosBefore, nonZerosAfter );

    // Uncomment for visual inspection
    //Visualization::show("ii failure", ii , 0);

  }



  //! Tests EdgeDetection::detectEdges
  TEST_F ( EdgeDetectionTest, detectEdgesTest )
  {
    // Traverse all available edge detectors
    for ( int p = 0; p < 5; p++ )
    {
      // Test the toggle switch
      for ( int t = 0; t < 2; t++ )
      {
        Parameters::Edge::mixed_edges_toggle_switch = t;

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

        // The squares_32FC1 image in 8UC1 format, scaled
        cv::Mat squares_8UC1 =
          Visualization::scaleImageForVisualization( squares_32FC1, 0);

        // The image of edges
        cv::Mat edges;
        EdgeDetection::detectEdges( squares_8UC1, &edges, p );

        ASSERT_EQ ( CV_8UC1, edges.type() );

        EXPECT_LT ( 0, cv::countNonZero( edges ) );
      }
    }
  }



  //! Tests EdgeDetection::denoiseEdges
  TEST_F ( EdgeDetectionTest, denoiseEdgesTest )
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



  // Tests EdgeDetection::floodFillPostprocess
  TEST_F ( EdgeDetectionTest, floodFillPortprocess )
  {
    // Convert squares_ into a CV_8UC3 image
    cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
    cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

    // Add an unfinished square to the squares_8UC3 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
    }

    // Run EdgeDetection::floodFillPostprocess
    EdgeDetection::floodFillPostprocess (&squares_8UC3);

    ASSERT_EQ ( CV_8UC3, squares_8UC3.type() );
  }



  //! Tests EdgeDetection::identifyCurveAndEndpoints
  TEST_F ( EdgeDetectionTest, identifyCurveAndEndpoints )
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
      EdgeDetection::identifyCurveAndEndpoints ( &gamma, 300, 451, &ret);

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
      EdgeDetection::identifyCurveAndEndpoints ( &gamma, 100, 200, &ret);

    // The point should not lie on a curve.
    EXPECT_EQ ( 1, ret.size() );

    // The first end point's coordinates will be the origin
    EXPECT_EQ ( p_invalid.first.x, 0 );
    EXPECT_EQ ( p_invalid.first.y, 0 );

    // The second end point's coordinates will be also be the origin
    EXPECT_EQ ( p_invalid.second.x, 0 );
    EXPECT_EQ ( p_invalid.second.y, 0 );
  }



  //! Tests EdgeDetection::identifyCurvesAndEndpoints
  TEST_F ( EdgeDetectionTest, identifyCurvesAndEndpoints )
  {
    // Three gamma shapes
    cv::Mat gammas = cv::Mat::zeros( squares_.size(), CV_8UC1 );

    // Valid
    for ( int rows = 10; rows < 40; rows++ )
    {
      gammas.at< unsigned char >( rows, 10 ) = 255;
    }

    for ( int cols = 10; cols < 50; cols++ )
    {
      gammas.at< unsigned char >( 10, cols ) = 255;
    }

    // Invalid
    for ( int rows = 60; rows < 70; rows++ )
    {
      gammas.at< unsigned char >( rows, 60 ) = 255;
    }

    for ( int cols = 60; cols < 70; cols++ )
    {
      gammas.at< unsigned char >( 60, cols ) = 255;
    }

    // Valid
    for ( int rows = 300; rows < 400; rows++ )
    {
      gammas.at< unsigned char >( rows, 300 ) = 255;
    }

    for ( int cols = 300; cols < 500; cols++ )
    {
      gammas.at< unsigned char >( 300, cols ) = 255;
    }


    std::vector< std::set<unsigned int > > lines;
    std::vector< std::pair< GraphNode, GraphNode > > endPoints;

    // Run EdgeDetection::identifyCurvesAndEndpoints
    EdgeDetection::identifyCurvesAndEndpoints( &gammas, &lines, &endPoints );

    // There should be as many lines as there are pairs of end-points
    ASSERT_EQ (lines.size(), endPoints.size());

    // The gammas image should be processed and turned void
    int nonZero = cv::countNonZero( gammas );
    ASSERT_EQ ( 0, nonZero );

    // There should be two curves exceeding the length threshold,
    // hence there are two different entries for curves and end-points
    EXPECT_EQ( 2, lines.size() );
    EXPECT_EQ( 2, endPoints.size() );

    // Every curve found should have a length greater than the threshold set.
    for ( int i = 0; i < lines.size(); i++ )
    {
      ASSERT_LT( Parameters::Outline::minimum_curve_points, lines[i].size() );
    }


    // The coordinates of the end-points found
    EXPECT_EQ ( endPoints[0].second.x, 10 );
    EXPECT_EQ ( endPoints[0].second.y, 49 );

    // The second end point's coordinates
    EXPECT_EQ ( endPoints[0].first.x, 39 );
    EXPECT_EQ ( endPoints[0].first.y, 10 );


    // The first end point's coordinates
    EXPECT_EQ ( endPoints[1].second.x, 300 );
    EXPECT_EQ ( endPoints[1].second.y, 499 );

    // The second end point's coordinates
    EXPECT_EQ ( endPoints[1].first.x, 399 );
    EXPECT_EQ ( endPoints[1].first.y, 300 );

  }



  //! Tests EdgeDetection::produceEdgesViaBackprojection
  TEST_F ( EdgeDetectionTest, produceEdgesViaBackprojectionTest)
  {
    // Convert squares_ into a CV_8UC3 image
    cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
    cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

    // Add an unfinished square to the squares_8UC3 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
    }

    // A dummy histogram
    cv::MatND histogram = cv::Mat::zeros( squares_8UC3.size(), CV_8UC1 );

    // Uncomment for visual inspection
    /*
     *Visualization::show("Before calling produceEdgesViaBackprojection",
     *  squares_8UC3, 0);
     */

    cv::Mat edges;

    // Run EdgeDetection::segmentation
    EdgeDetection::produceEdgesViaBackprojection
      ( squares_8UC3, histogram, &edges );

    // Uncomment for visual inspection
    /*
     *Visualization::show("After calling produceEdgesViaBackprojection 0",
     *  edges, 0);
     */

    // The edges image should not be blank
    ASSERT_LT ( 0, cv::countNonZero ( edges ) );

    // The edges image should be of type CV_8UC1
    ASSERT_EQ ( CV_8UC1, edges.type() );
  }



  //! Tests EdgeDetection::produceEdgesViaSegmentation
  TEST_F ( EdgeDetectionTest, produceEdgesViaSegmentationTest )
  {
    // Traverse all available edge detectors
    for ( int p = 0; p < 5; p++ )
    {
      Parameters::Edge::edge_detection_method = p;

      // Posterize?
      for ( int f = 0; f < 2; f++ )
      {
        Parameters::Rgb::posterize_after_segmentation = f;

        // Convert squares_ into a CV_8UC3 image
        cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
        cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

        // Add an unfinished square to the squares_8UC3 image
        for ( int rows = 300; rows < 400; rows++ )
        {
          squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
        }

        for ( int cols = 300; cols < 400; cols++ )
        {
          squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
        }


        // Uncomment for visual inspection
        //Visualization::show("Before calling produceEdgesViaSegmentation 0",
        //squares_8UC3, 0);

        // Segmentation using cv::pyrMeanShiftFiltering
        cv::Mat edges_0;

        // Run EdgeDetection::segmentation
        // segmentation method = 0
        EdgeDetection::produceEdgesViaSegmentation ( squares_8UC3, &edges_0 );

        // Uncomment for visual inspection
        /*
         *Visualization::show("After calling produceEdgesViaSegmentation 0",
         *  edges_0, 0);
         */

        // The image should not be blank
        ASSERT_LT ( 0, cv::countNonZero ( edges_0 ) );

        // The edges image should be of type CV_8UC1
        ASSERT_EQ ( CV_8UC1, edges_0.type() );


        // Uncomment for visual inspection
        /*
         *Visualization::show("Before calling produceEdgesViaSegmentation 1",
         *squares_8UC3, 0);
         */
      }
    }
  }



  //! Tests EdgeDetection::segmentation
  TEST_F ( EdgeDetectionTest, segmentationTest )
  {
    // Convert squares_ into a CV_8UC3 image
    cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
    cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

    // Add an unfinished square to the squares_8UC3 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
    }

    // Uncomment for visual inspection
    //Visualization::show("Before calling segmentation", squares_8UC3, 0);

    // Segmentation using cv::pyrMeanShiftFiltering
    cv::Mat segmented;

    // Run EdgeDetection::segmentation
    EdgeDetection::segmentation ( squares_8UC3, &segmented );

    // Uncomment for visual inspection
    //Visualization::show("After calling segmentation", segmented, 0);

    ASSERT_EQ ( CV_8UC3, segmented.type() );
  }



  //! Tests EdgeDetection::watershedViaBackprojection
  TEST_F ( EdgeDetectionTest, watershedViaBackprojectionTest )
  {
    // Convert squares_ into a CV_8UC3 image
    cv::Mat squares_8UC3 = cv::Mat::zeros ( squares_.size(), CV_8UC3 );
    cv::cvtColor( squares_, squares_8UC3, CV_GRAY2BGR );

    // Add an unfinished square to the squares_8UC3 image
    for ( int rows = 300; rows < 400; rows++ )
    {
      squares_8UC3.at< cv::Vec3b >( rows, 300 ) = 128;
    }

    for ( int cols = 300; cols < 400; cols++ )
    {
      squares_8UC3.at< cv::Vec3b >( 300, cols ) = 128;
    }

    // Uncomment for visual inspection
    //Visualization::show("Before", squares_8UC3, 0);

    // A dummy backprojection image
    cv::Mat backprojection = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );


    bool edges = false;
    for ( int i = 0; i < 2; i++ )
    {
      edges = edges ? false : true;

      // The watersheded image
      cv::Mat watersheded;

      // Run EdgeDetection::watershedViaBackprojection
      EdgeDetection::watershedViaBackprojection( squares_8UC3,
        backprojection, edges, &watersheded );

      ASSERT_EQ ( CV_8UC1, watersheded.type() );
    }
  }

} // namespace pandora_vision
