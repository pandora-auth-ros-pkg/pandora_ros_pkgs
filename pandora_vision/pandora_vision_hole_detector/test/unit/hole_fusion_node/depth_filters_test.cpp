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

#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/filters_resources.h"
#include "hole_fusion_node/planes_detection.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class DepthFiltersTest
    @brief Tests the integrity of methods of class DepthFilters
   **/
  class DepthFiltersTest : public ::testing::Test
  {
    protected:

      DepthFiltersTest() : cloud ( new PointCloud ) {}

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
      void generateDepthRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        const float& depthIn,
        cv::Mat* image );

      /**
        @brief Constructs the internals of a rectangular hole
        of width @param x and height of @param y
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        return [HolesConveyor] A struct containing the elements of one hole
       **/
      HolesConveyor getConveyor (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y );

      //! Sets up one image: squares_,
      //! which features three squares of size 100.
      //! The first one (order matters here) has its upper left vertex at
      //! (100, 100),
      //! the second one has its upper right vertex at (WIDTH - 3, 3)
      //! (so that the blob it represents can barely be identified)
      //! and the the third one has its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1).
      //! It creates the corresponding conveyor entries for these square holes
      //! and the corresponding point cloud to match the squares_ depth image
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

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

        DepthFiltersTest::generateDepthRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100,
            0.2,
            &lowerRightSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100 ),
          &conveyor);


        // Construct the upper right image
        cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

        DepthFiltersTest::generateDepthRectangle
          ( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100,
            0.2,
            &upperRightSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100 ),
          &conveyor);

        // Construct the upper left square
        cv::Mat upperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        DepthFiltersTest::generateDepthRectangle
          ( cv::Point2f ( 100, 100 ),
            100,
            100,
            0.3,
            &upperLeftSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor);

        // Compose the final squares_ image
        squares_ += lowerRightSquare + upperRightSquare + upperLeftSquare;


        // Construct the point cloud corresponding to the squares_ image

        // Fill in the cloud data
        cloud->width = WIDTH;
        cloud->height = HEIGHT;
        cloud->points.resize ( cloud->width * cloud->height );

        // Generate the data
        for ( int i = 0; i < cloud->points.size (); i++ )
        {
          // Row
          int x = i / WIDTH;

          // Column
          int y = i % WIDTH;

          cloud->points[i].x = 1 + x / 10;
          cloud->points[i].y = 1 + y / 10;
          cloud->points[i].z = squares_.at< float >( x, y );
        }

      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;

      // The conveyor of holes that will be used to test methods of class
      // DepthFilters
      HolesConveyor conveyor;

      // The point cloud corresponding to the squares_ image
      PointCloudPtr cloud;

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
  void DepthFiltersTest::generateDepthRectangle (
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



  /**
    @brief Constructs the internals of a rectangular hole
    of width @param x and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    return [HolesConveyor] A struct containing the elements of one hole
   **/
  HolesConveyor DepthFiltersTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // What will be returned: the internal elements of one hole
    HolesConveyor conveyor;

    // The hole's keypoint
    cv::KeyPoint k (  upperLeft.x + x / 2, upperLeft.y + y / 2 , 1 );

    conveyor.keyPoints.push_back(k);


    // The four vertices of the rectangle
    cv::Point2f vertex_1( upperLeft.x, upperLeft.y );

    cv::Point2f vertex_2( upperLeft.x, upperLeft.y + y - 1 );

    cv::Point2f vertex_3( upperLeft.x + x - 1, upperLeft.y + y - 1 );

    cv::Point2f vertex_4( upperLeft.x + x - 1, upperLeft.y );

    std::vector<cv::Point2f> rectangle;
    rectangle.push_back(vertex_1);
    rectangle.push_back(vertex_2);
    rectangle.push_back(vertex_3);
    rectangle.push_back(vertex_4);

    conveyor.rectangles.push_back(rectangle);


    // The outline points of the hole will be obtained through the depiction
    // of the points consisting the rectangle
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

    for(unsigned int j = 0; j < 4; j++)
    {
      cv::line(image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
    }


    std::vector<cv::Point2f> outline;
    for ( int rows = 0; rows < image.rows; rows++ )
    {
      for ( int cols = 0; cols < image.cols; cols++ )
      {
        if ( image.at<unsigned char>( rows, cols ) != 0 )
        {
          outline.push_back( cv::Point2f ( cols, rows ) );
        }
      }
    }

    conveyor.outlines.push_back(outline);

    return conveyor;

  }



  //! Test DepthFilters::applyFilter
  TEST_F ( DepthFiltersTest, ApplyFilterTest )
  {
    // Inflations size : 0

    // Create the needed by the DepthFilters::applyFilter method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_0);

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // Run DepthFilters::applyFilter for every filter
    for ( int f = 1; f < 6; f++ )
    {
      std::vector< float > probabilitiesVector_0 ( 3, 0.0 );

      std::vector< cv::Mat > imgs;
      std::vector< std::string > msgs;

      // Run DepthFilters::applyFilter once
      DepthFilters::applyFilter(
        f, squares_, cloud, conveyor,
        holesMasksSetVector_0,
        inflatedRectanglesVector_0,
        inflatedRectanglesIndices_0,
        intermediatePointsSetVector_0,
        &probabilitiesVector_0,
        &imgs,
        &msgs);

      if ( f == 1 )
      {
        // All three holes should have an inflated rectangle for inflation
        // size of value 0
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_LT ( 0.3, probabilitiesVector_0[i] );
        }
      }

      if ( f == 2 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          ASSERT_EQ ( 1.0, probabilitiesVector_0[i] );
        }
      }

      if ( f == 3 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector_0[i] );
        }
      }

      if ( f == 4 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
        }
      }

      if ( f == 5 )
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector_0[0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_0[1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_0[2], 0.1 );
      }

    }


    // Inflations size : 10

    // Create the needed by the DepthFilters::applyFilter method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_10);

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );

    std::vector< std::set< unsigned int > > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // Run DepthFilters::applyFilter for every filter
    for ( int f = 1; f < 6; f++ )
    {
      std::vector< float > probabilitiesVector_10 ( 3, 0.0 );

      std::vector< cv::Mat > imgs;
      std::vector< std::string > msgs;

      // Run DepthFilters::applyFilter once
      DepthFilters::applyFilter(
        f, squares_, cloud, conveyor,
        holesMasksSetVector_10,
        inflatedRectanglesVector_10,
        inflatedRectanglesIndices_10,
        intermediatePointsSetVector_10,
        &probabilitiesVector_10,
        &imgs,
        &msgs);

      if ( f == 1 )
      {
        // Only the last two holes should have an inflated rectangle
        // for inflation size of value 2
        ASSERT_EQ ( 0.0, probabilitiesVector_10[0] );
        ASSERT_EQ ( 0.0, probabilitiesVector_10[1] );

        EXPECT_EQ ( 1.0, probabilitiesVector_10[2] );
      }

      if ( f == 2 )
      {
        // The lower right and upper right squares' inflated rectangles
        // exceed the image's boundaries, hence, their probability of lying on
        // a plane is diminished to zero
        ASSERT_EQ ( 0.0, probabilitiesVector_10[0] );
        ASSERT_EQ ( 0.0, probabilitiesVector_10[1] );

        // Only the upper left square's inflated rectangle is within the image's
        // bounds.
        ASSERT_EQ ( 1.0, probabilitiesVector_10[2] );
      }

      if ( f == 3 )
      {
        for ( int i = 0; i < probabilitiesVector_10.size(); i++ )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector_10[i] );
        }
      }

      if ( f == 4 )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_LT ( 0.9, probabilitiesVector_10[2] );
      }

      if ( f == 5 )
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector_10[0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_10[1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_10[2], 0.1 );
      }
    }
  }



  //! Test DepthFilters::checkHoles
  TEST_F ( DepthFiltersTest, CheckHolesTest )
  {
    // Inflations size : 0

    // Create the needed by the DepthFilters::checkHoles method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_0);

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_0(
      5, // Five depth filters in total
      std::vector< float >( conveyor.keyPoints.size(), 0.0 ) );


    // Set the execution order for ease of testing
    Parameters::HoleFusion::run_checker_depth_diff = 1;
    Parameters::HoleFusion::run_checker_outline_of_rectangle = 2;
    Parameters::HoleFusion::run_checker_depth_area = 3;
    Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle = 4;
    Parameters::HoleFusion::run_checker_depth_homogeneity = 5;

    // Run DepthFilters::checkHoles
    DepthFilters::checkHoles(
      conveyor,
      squares_,
      cloud,
      holesMasksSetVector_0,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      intermediatePointsSetVector_0,
      &probabilitiesVector2D_0 );

    for ( int f = 0; f < probabilitiesVector2D_0.size(); f++ )
    {
      for ( int h = 0; h < probabilitiesVector2D_0[f].size(); h++ )
      {
        if ( f == 0 )
        {
          EXPECT_LT ( 0.3, probabilitiesVector2D_0[f][h] );
        }

        if ( f == 1 )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector2D_0[f][h] );
        }

        if ( f == 2 )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector2D_0[f][h] );
        }

        if ( f == 3 )
        {
          EXPECT_EQ ( 0.0, probabilitiesVector2D_0[f][h] );
        }

        if ( f == 4 )
        {
          // The east and south edges of the lower right square are clipped
          EXPECT_NEAR ( 196 / 9996, probabilitiesVector2D_0[f][0], 0.1 );
          EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_0[f][1], 0.1 );
          EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_0[f][2], 0.1 );
        }
      }
    }

    // Inflations size : 10

    // Create the needed by the DepthFilters::checkHoles method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_10);

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );

    std::vector< std::set< unsigned int > > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_10(
      5, // Five depth filters in total
      std::vector< float >( conveyor.keyPoints.size(), 0.0 ) );

    // Run DepthFilters::checkHoles
    DepthFilters::checkHoles(
      conveyor,
      squares_,
      cloud,
      holesMasksSetVector_10,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      intermediatePointsSetVector_10,
      &probabilitiesVector2D_10 );

    for ( int f = 0; f < probabilitiesVector2D_10.size(); f++ )
    {
      if ( f == 0 )
      {
        // Only the last two holes should have an inflated rectangle
        // for inflation size of value 2
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][1] );

        EXPECT_EQ ( 1.0, probabilitiesVector2D_10[f][2] );
      }

      if ( f == 1 )
      {
        // The lower right and upper right squares' inflated rectangles
        // exceed the image's boundaries, hence, their probability of lying on
        // a plane is diminished to zero
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][1] );

        // Only the upper left square's inflated rectangle is within the image's
        // bounds.
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10[f][2] );
      }

      if ( f == 2 )
      {
        for ( int h = 0; h < probabilitiesVector2D_10[f].size(); h++ )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector2D_10[f][h] );
        }
      }

      if ( f == 3 )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10[f][1] );
        EXPECT_LT ( 0.9, probabilitiesVector2D_10[f][2] );
      }

      if ( f == 4 )
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector2D_10[f][0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_10[f][1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_10[f][2], 0.1 );
      }
    }
  }



  //! Test DepthFilters::checkHolesDepthArea
  TEST_F ( DepthFiltersTest, CheckHolesDepthAreaTest )
  {
    // Generate the vector of holes' mask (set)
    std::vector< std::set< unsigned int > > holesMasksSetVector;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    // Needed vectors by the DepthFilters::checkHolesDepthDiff method
    std::vector<std::string> msgs;
    std::vector<float> probabilitiesVector( 3, 0.0 );

    // Run DepthFilters::checkHolesDepthDiff
    DepthFilters::checkHolesDepthArea(
      conveyor,
      squares_,
      holesMasksSetVector,
      &msgs,
      &probabilitiesVector );

    for ( int i = 0; i < probabilitiesVector.size(); i++ )
    {
      EXPECT_EQ ( 1.0, probabilitiesVector[i] );
    }
  }



  //! Test DepthFilters::checkHolesDepthDiff
  TEST_F ( DepthFiltersTest, CheckHolesDepthDiffTest )
  {
    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 0
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // Needed vectors by the DepthFilters::checkHolesDepthDiff method
    std::vector<std::string> msgs;
    std::vector<float> probabilitiesVector_0( 3, 0.0 );

    // Run DepthFilters::checkHolesDepthDiff
    DepthFilters::checkHolesDepthDiff(
      squares_,
      conveyor,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &msgs,
      &probabilitiesVector_0 );



    for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
    {
      // All three holes should have an inflated rectangle for inflation
      // size of value 0
      EXPECT_LT ( 0.3, probabilitiesVector_0[i] );
    }



    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 2
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_2;
    std::vector< int > inflatedRectanglesIndices_2;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      2,
      &inflatedRectanglesVector_2,
      &inflatedRectanglesIndices_2 );

    // Needed vectors by the DepthFilters::checkHolesDepthDiff method
    std::vector<float> probabilitiesVector_2( 3, 0.0 );
    msgs.clear();

    // Run DepthFilters::checkHolesDepthDiff
    DepthFilters::checkHolesDepthDiff(
      squares_,
      conveyor,
      inflatedRectanglesVector_2,
      inflatedRectanglesIndices_2,
      &msgs,
      &probabilitiesVector_2 );


    // Only the last two holes should have an inflated rectangle
    // for inflation size of value 2
    ASSERT_EQ ( 0.0, probabilitiesVector_2[0] );
    ASSERT_LT ( 0.0, probabilitiesVector_2[1] );
    ASSERT_LT ( 0.0, probabilitiesVector_2[2] );


    EXPECT_LT ( 0.8, probabilitiesVector_2[1] );
    EXPECT_EQ ( 1.0, probabilitiesVector_2[2] );


    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 8
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_8;
    std::vector< int > inflatedRectanglesIndices_8;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      8,
      &inflatedRectanglesVector_8,
      &inflatedRectanglesIndices_8 );

    // Needed vectors by the DepthFilters::checkHolesDepthDiff method
    std::vector< float > probabilitiesVector_8( 3, 0.0 );
    msgs.clear();

    // Run DepthFilters::checkHolesDepthDiff
    DepthFilters::checkHolesDepthDiff(
      squares_,
      conveyor,
      inflatedRectanglesVector_8,
      inflatedRectanglesIndices_8,
      &msgs,
      &probabilitiesVector_8);


    // Only the last two holes should have an inflated rectangle
    // for inflation size of value 2
    ASSERT_EQ ( 0.0, probabilitiesVector_8[0] );
    ASSERT_EQ ( 0.0, probabilitiesVector_8[1] );
    ASSERT_LT ( 0.0, probabilitiesVector_8[2] );


    EXPECT_EQ ( 1.0, probabilitiesVector_8[2] );


    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 180
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_180;
    std::vector< int > inflatedRectanglesIndices_180;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      180,
      &inflatedRectanglesVector_180,
      &inflatedRectanglesIndices_180 );

    // Needed vectors by the DepthFilters::checkHolesDepthDiff method
    std::vector<float> probabilitiesVector_180( 3, 0.0 );
    msgs.clear();

    // Run DepthFilters::checkHolesDepthDiff
    DepthFilters::checkHolesDepthDiff(
      squares_,
      conveyor,
      inflatedRectanglesVector_180,
      inflatedRectanglesIndices_180,
      &msgs,
      &probabilitiesVector_180 );


    // Only the last two holes should have an inflated rectangle
    // for inflation size of value 2
    ASSERT_EQ ( 0.0, probabilitiesVector_180[0] );
    ASSERT_EQ ( 0.0, probabilitiesVector_180[1] );
    ASSERT_EQ ( 0.0, probabilitiesVector_180[2] );

  }



  //! Test DepthFilters::checkHolesDepthHomogeneity
  TEST_F ( DepthFiltersTest, CheckHolesDepthHomogeneityTest)
  {
    // Generate the vector of holes' mask (set)
    std::vector< std::set< unsigned int > > holesMasksSetVector;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );

    // Needed vectors by the DepthFilters::checkHolesDepthHomogeneity method
    std::vector<std::string> msgs;
    std::vector<float> probabilitiesVector( 3, 0.0 );

    // Run DepthFilters::checkHolesDepthHomogeneity
    DepthFilters::checkHolesDepthHomogeneity(
      conveyor,
      squares_,
      holesMasksSetVector,
      &msgs,
      &probabilitiesVector );

    // The east and south edges of the lower right square are clipped
    EXPECT_NEAR ( 196 / 9996, probabilitiesVector[0], 0.1 );
    EXPECT_NEAR ( 392 / 9996, probabilitiesVector[1], 0.1 );
    EXPECT_NEAR ( 392 / 9996, probabilitiesVector[2], 0.1 );
  }



  //! Test DepthFilters::checkHolesOutlineToRectanglePlaneConstitution
  TEST_F ( DepthFiltersTest, CheckHolesOutlineToRectanglePlaneConstitutionTest )
  {
    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 0
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // Generate the intermediate points set vector
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // Needed vectors by the
    // DepthFilters::checkHolesOutlineToRectanglePlaneConstitution method
    std::vector<std::string> msgs;
    std::vector<float> probabilitiesVector_0( 3, 0.0 );

    // Run DepthFilters::checkHolesOutlineToRectanglePlaneConstitution
    DepthFilters::checkHolesOutlineToRectanglePlaneConstitution(
      squares_,
      cloud,
      intermediatePointsSetVector_0,
      inflatedRectanglesIndices_0,
      &probabilitiesVector_0,
      &msgs );

    for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
    {
      EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
    }


    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 10
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );

    // Generate the intermediate points set vector
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // Needed vectors by the
    // DepthFilters::checkHolesOutlineToRectanglePlaneConstitution method
    msgs.clear();
    std::vector<float> probabilitiesVector_10( 3, 0.0 );

    // Run DepthFilters::checkHolesOutlineToRectanglePlaneConstitution
    DepthFilters::checkHolesOutlineToRectanglePlaneConstitution(
      squares_,
      cloud,
      intermediatePointsSetVector_10,
      inflatedRectanglesIndices_10,
      &probabilitiesVector_10,
      &msgs );

    EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
    EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
    EXPECT_LT ( 0.9, probabilitiesVector_10[2] );

  }



  //! Test DepthFilters::checkHolesRectangleEdgesPlaneConstitution
  TEST_F ( DepthFiltersTest, CheckHolesRectangleEdgesPlaneConstitutionTest )
  {


    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 0
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );


    // Needed vectors by the
    // DepthFilters::checkHolesRectangleEdgesPlaneConstitution method
    std::vector<std::string> msgs;
    std::vector<float> probabilitiesVector_0( 3, 0.0 );

    // Run DepthFilters::checkHolesRectangleEdgesPlaneConstitution
    DepthFilters::checkHolesRectangleEdgesPlaneConstitution(
      squares_,
      cloud,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &probabilitiesVector_0,
      &msgs );

    // All of the rectangles lie on planes
    for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
    {
      ASSERT_EQ ( 1.0, probabilitiesVector_0[i] );
    }



    // Generate the inflated rectangles and corresponding indices vectors
    // for an inflation size of value 10
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );


    // Needed vectors by the
    // DepthFilters::checkHolesRectangleEdgesPlaneConstitution method
    msgs.clear();
    std::vector<float> probabilitiesVector_10( 3, 0.0 );

    // Run DepthFilters::checkHolesRectangleEdgesPlaneConstitution
    DepthFilters::checkHolesRectangleEdgesPlaneConstitution(
      squares_,
      cloud,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &probabilitiesVector_10,
      &msgs );

    // The lower right and upper right squares' inflated rectangles
    // exceed the image's boundaries, hence, their probability of lying on
    // a plane is diminished to zero
    ASSERT_EQ ( 0.0, probabilitiesVector_10[0] );
    ASSERT_EQ ( 0.0, probabilitiesVector_10[1] );

    // Only the upper left square's inflated rectangle is within the image's
    // bounds.
    ASSERT_EQ ( 1.0, probabilitiesVector_10[2] );
  }


} // namespace pandora_vision
