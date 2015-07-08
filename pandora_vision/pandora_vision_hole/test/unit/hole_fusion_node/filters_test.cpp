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

#include "hole_fusion_node/filters.h"
#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/rgb_filters.h"
#include "hole_fusion_node/filters_resources.h"
#include "hole_fusion_node/planes_detection.h"
#include "rgb_node/utils/histogram.h"
#include "rgb_node/utils/parameters.h"
#include "gtest/gtest.h"
#include <stdlib.h>
#include <sys/time.h>

namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace hole_fusion
{
  /**
    @class FiltersTest
    @brief Tests the integrity of methods of class Filters
   **/
  class FiltersTest : public ::testing::Test
  {
    protected:

      FiltersTest() : cloud ( new PointCloud ) {}

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
        @brief Constructs a rectangle of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] rgbIn [const unsigned char] The colour value for all points
        inside the rectangle
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted
        return void
       **/
      void generateRgbRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        const unsigned char rgbIn,
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

      //! Sets up two images: depthSquares_,
      //! which features three squares of size 100.
      //! The first one (order matters here) has its upper left vertex at
      //! (100, 100),
      //! the second one has its upper right vertex at (WIDTH - 3, 3)
      //! (so that the blob it represents can barely be identified)
      //! and the the third one has its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1).
      //! It creates the corresponding conveyor entries for these square holes
      //! and the corresponding point cloud to match the depthSquares_ depth image.
      //! The second image is rgbSquares_,
      //! which features four squares of size 100, and one of size 140
      //! The first one (order matters here) has its upper left vertex at
      //! (100, 100),
      //! the second one has its upper right vertex at (WIDTH - 3, 3)
      //! (so that the blob it represents can barely be identified)
      //! and the the third one has its lower right vertex at
      //! (WIDTH - 1, HEIGHT - 1).
      //! It creates a square whose upper left vertex is located
      //! at ( 250, 250 ), filled with random colours.
      //! Finally, there is the square with edges of length 140px, which
      //! surrounds the upper left square, and is of black colour
      //! (constructed to test the Filters::checkHolesLuminosityDiff method)
      //! It creates the corresponding conveyor entries for these square holes.
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        ///////////// Construct the depth image and the point cloud ////////////

        // The image upon which the squares will be inprinted
        depthSquares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Construct the depthSquares_ image

        // Set the depth for each point of the depthSquares_ image to 1.0
        for ( int rows = 0; rows < depthSquares_.rows; rows++ )
        {
          for ( int cols = 0; cols < depthSquares_.cols; cols++ )
          {
            depthSquares_.at< float >( rows, cols ) = 1.0;
          }
        }

        // Construct the lower right square
        cv::Mat depthLowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

        FiltersTest::generateDepthRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100,
            0.2,
            &depthLowerRightSquare );


        // Construct the upper right image
        cv::Mat depthUpperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

        FiltersTest::generateDepthRectangle
          ( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100,
            0.2,
            &depthUpperRightSquare );


        // Construct the upper left square
        cv::Mat depthUpperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        FiltersTest::generateDepthRectangle
          ( cv::Point2f ( 100, 100 ),
            100,
            100,
            0.3,
            &depthUpperLeftSquare );


        // Compose the final depthSquares_ image
        depthSquares_ +=
          depthLowerRightSquare +
          depthUpperRightSquare +
          depthUpperLeftSquare;


        // Construct the point cloud corresponding to the depthSquares_ image

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
          cloud->points[i].z = depthSquares_.at< float >( x, y );
        }



        /////////////////////// Construct the rgb image ////////////////////////

        // Construct the lower right square
        cv::Mat rgbLowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

        FiltersTest::generateRgbRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100,
            140,
            &rgbLowerRightSquare );


        // Construct the upper right image
        cv::Mat rgbUpperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

        FiltersTest::generateRgbRectangle
          ( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100,
            160,
            &rgbUpperRightSquare );


        // Construct the upper left square
        cv::Mat rgbUpperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        FiltersTest::generateRgbRectangle
          ( cv::Point2f ( 100, 100 ),
            100,
            100,
            180,
            &rgbUpperLeftSquare );


        // Construct the square surrounding the upper left square
        cv::Mat rgbSurroundingSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        FiltersTest::generateRgbRectangle
          ( cv::Point2f ( 80, 80 ),
            140,
            140,
            1,
            &rgbSurroundingSquare );


        // Construct the middle square. In contrast to the other three
        // rectangles, this is scattered with random colours inside it
        cv::Mat rgbMiddleSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        cv::Point upperLeft ( 250, 250 );
        int x = 100;
        int y = 100;

        // The seed for the rand_r method
        unsigned int seed = 0;
        for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
        {
          for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
          {
            unsigned char randomBlue = rand_r(&seed) % 256;

            unsigned char randomGreen = rand_r(&seed) % 256;

            unsigned char randomRed = rand_r(&seed) % 256;

            rgbMiddleSquare.at< cv::Vec3b >( rows, cols ).val[0] = randomBlue;
            rgbMiddleSquare.at< cv::Vec3b >( rows, cols ).val[1] = randomGreen;
            rgbMiddleSquare.at< cv::Vec3b >( rows, cols ).val[2] = randomRed;
          }
        }


        // Create the conveyor of candidate holes for both images

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 80, 80 ),
            140,
            140 ),
          &conveyor);

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 250, 250 ),
            100,
            100 ),
          &conveyor);


        // The image upon which the squares will be inprinted
        rgbSquares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        // Compose the final rgbSquares_ image
        rgbSquares_ =
          rgbLowerRightSquare +
          rgbSurroundingSquare +
          rgbUpperRightSquare +
          rgbUpperLeftSquare +
          rgbMiddleSquare;

        // Construct the rgbSquares_ image. The entire image is at a colour of
        // value approximate the the colour value of the images of walls
        for ( int rows = 0; rows < rgbSquares_.rows; rows++ )
        {
          for ( int cols = 0; cols < rgbSquares_.cols; cols++ )
          {
            if (rgbSquares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
            {
              rgbSquares_.at< cv::Vec3b >( rows, cols ).val[0] = 116;
              rgbSquares_.at< cv::Vec3b >( rows, cols ).val[1] = 163;
              rgbSquares_.at< cv::Vec3b >( rows, cols ).val[2] = 171;
            }
          }
        }
      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The depth image
      cv::Mat depthSquares_;

      // The RGB image
      cv::Mat rgbSquares_;

      // The conveyor of holes that will be used to test methods of class
      // Filters
      HolesConveyor conveyor;

      // The point cloud corresponding to the depthSquares_ image
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
  void FiltersTest::generateDepthRectangle (
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
    @brief Constructs a rectangle of width @param x and height of @param y.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] rgbIn [const unsigned char] The colour value for all points
    inside the rectangle
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on
    return void
   **/
  void FiltersTest::generateRgbRectangle (
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
        image->at< cv::Vec3b >( rows, cols ) = rgbIn;
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
  HolesConveyor FiltersTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // What will be returned: the internal elements of one hole
    HoleConveyor hole;

    // The hole's keypoint
    cv::KeyPoint k (  upperLeft.x + x / 2, upperLeft.y + y / 2 , 1 );

    hole.keypoint = k;


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

    hole.rectangle = rectangle;


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

    hole.outline = outline;

    // Push hole into a HolesConveyor
    HolesConveyor conveyor;
    conveyor.holes.push_back(hole);

    return conveyor;
  }



  //! Tests Filters::applyFilter
  TEST_F ( FiltersTest, applyFilterTest )
  {
    // Inflations size : 0

    // Create the needed by the Filters::applyFilter method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      depthSquares_,
      &holesMasksSetVector_0);

    // The vector of mask images
    std::vector< cv::Mat > holesMasksImageVector_0;

    FiltersResources::createHolesMasksImageVector(
      conveyor,
      depthSquares_,
      &holesMasksImageVector_0 );

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      depthSquares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // The intermediate points vector of sets
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // The intermediate points vector of images
    std::vector< cv::Mat > intermediatePointsImageVector_0;

    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsImageVector_0 );


    // Run Filters::applyFilter for every filter,
    // except for the ones that utilize textures
    for ( int f = 1; f < 10; f++ )
    {
      if (f == 3 || f == 4)
      {
        continue;
      }

      std::vector< float > probabilitiesVector_0 ( conveyor.size(), 0.0 );

      std::vector< cv::Mat > imgs;
      std::vector< std::string > msgs;

      // A dummy histogram
      std::vector<cv::MatND> histogram;

      // Run Filters::applyFilter
      Filters::applyFilter(
        conveyor,
        f,
        depthSquares_,
        rgbSquares_,
        histogram,
        cloud,
        holesMasksSetVector_0,
        holesMasksImageVector_0,
        inflatedRectanglesVector_0,
        inflatedRectanglesIndices_0,
        intermediatePointsSetVector_0,
        intermediatePointsImageVector_0,
        &probabilitiesVector_0,
        &imgs,
        &msgs);

      // Color homogeneity
      if ( f == 1 )
      {
        // All squares except the one whose internal colours are randomly
        // generated are dull.
        EXPECT_EQ ( 0.0, probabilitiesVector_0[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_0[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_0[2] );
        EXPECT_EQ ( 0.0, probabilitiesVector_0[3] );
        EXPECT_LT ( 0.8, probabilitiesVector_0[4] );
      }

      // Luminosity difference
      if ( f == 2 )
      {
        // All probabilities amount to zero: the size of each set inside the
        // intermediatePointsSetVector_0 vector is zero
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
        }
      }

      // Texture-based filters are not tested yet.
      // TODO: import an image from the walls directory in order to test them

      // Depth difference
      if ( f == 5 )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector_0[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_0[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_0[2] );

        // The surrounding square has its vertices closer than its keypoint
        EXPECT_LT ( 0.0, probabilitiesVector_0[3] );

        EXPECT_EQ ( 0.0, probabilitiesVector_0[4] );
      }

      // Rectangle points plane constitution
      if ( f == 6 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          ASSERT_EQ ( 1.0, probabilitiesVector_0[i] );
        }
      }

      // Depth / area
      if ( f == 7 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector_0[i] );
        }
      }

      // Intermediate points plane constitution
      if ( f == 8 )
      {
        for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
        {
          EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
        }
      }

      // Depth homogeneity
      if ( f == 9 )
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector_0[0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_0[1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_0[2], 0.1 );

        // The surrounding square encloses a hole which actually has edges
        EXPECT_LT ( 0, probabilitiesVector_0[3]);

        EXPECT_EQ ( 0, probabilitiesVector_0[4]);
      }

    }


    // Inflations size : 10

    // Create the needed by the Filters::applyFilter method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      depthSquares_,
      &holesMasksSetVector_10);

    // The vector of mask images
    std::vector< cv::Mat > holesMasksImageVector_10;

    FiltersResources::createHolesMasksImageVector(
      conveyor,
      depthSquares_,
      &holesMasksImageVector_10 );

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      depthSquares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );

    std::vector< std::set< unsigned int > > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // The intermediate points vector of images
    std::vector< cv::Mat > intermediatePointsImageVector_10;

    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsImageVector_10 );

    // Run Filters::applyFilter for every filter
    // except for the ones that utilize textures
    for ( int f = 1; f < 10; f++ )
    {
      if (f == 3 || f == 4)
      {
        continue;
      }

      std::vector< float > probabilitiesVector_10 ( conveyor.size(), 0.0 );

      std::vector< cv::Mat > imgs;
      std::vector< std::string > msgs;
      std::vector<cv::MatND> histogram;

      // Run Filters::applyFilter
      Filters::applyFilter(
        conveyor,
        f,
        depthSquares_,
        rgbSquares_,
        histogram,
        cloud,
        holesMasksSetVector_10,
        holesMasksImageVector_10,
        inflatedRectanglesVector_10,
        inflatedRectanglesIndices_10,
        intermediatePointsSetVector_10,
        intermediatePointsImageVector_10,
        &probabilitiesVector_10,
        &imgs,
        &msgs);

      // Color homogeneity
      if ( f == 1 )
      {
        // All squares except the one whose internal colours are randomly
        // generated are dull.
        EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[3] );
        EXPECT_LT ( 0.8, probabilitiesVector_10[4] );
      }

      // Luminosity difference
      if ( f == 2 )
      {
        // The inflated rectangles of the lower right and upper right rectangles
        // are out of the image's bounds.
        // The upper left rectangle is surrounded by a black rectangle,
        // so there goes
        // The surrounding rectangle and the middle one are just fine
        EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.8, probabilitiesVector_10[3] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[4] );
      }

      // Texture-based filters are not tested yet.
      // TODO: import an image from the walls directory in order to test them

      // Depth difference
      if ( f == 5 )
      {
        // Only the last two holes should have an inflated rectangle
        // for inflation size of value 10
        EXPECT_LT ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[3] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[4] );
      }

      // Rectangle points plane constitution
      if ( f == 6 )
      {
        EXPECT_LT ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[3] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[4] );
      }

      // Depth / area
      if ( f == 7 )
      {
        for ( int i = 0; i < probabilitiesVector_10.size(); i++ )
        {
          EXPECT_EQ ( 1.0, probabilitiesVector_10[i] );
        }
      }

      // Intermediate points plane constitution
      if ( f == 8 )
      {
        EXPECT_LT ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.9, probabilitiesVector_10[3] );
        EXPECT_LT ( 0.9, probabilitiesVector_10[4] );
      }

      // Depth homogeneity
      if ( f == 9 )
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector_10[0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_10[1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector_10[2], 0.1 );
        EXPECT_LT ( 0, probabilitiesVector_10[3] );
        EXPECT_EQ ( 0, probabilitiesVector_10[4] );
      }
    }
  }



  //! Tests Filters::applyFiltersTest:
  TEST_F ( FiltersTest, applyFiltersTest )
  {
    /////////////////////////// Inflations size : 0 ////////////////////////////

    // Create the needed by the Filters::applyFilters method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      depthSquares_,
      &holesMasksSetVector_0);

    // The vector of mask images
    std::vector< cv::Mat > holesMasksImageVector_0;

    FiltersResources::createHolesMasksImageVector(
      conveyor,
      depthSquares_,
      &holesMasksImageVector_0 );

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      depthSquares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // The intermediate points vector of sets
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // The intermediate points vector of images
    std::vector< cv::Mat > intermediatePointsImageVector_0;

    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsImageVector_0 );

    // ------------------------------ RGBD_MODE --------------------------------

    // Set the order of the filters' execution in random
    Parameters::Filters::DepthDiff::priority = 1;
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 2;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 4;
    Parameters::Filters::DepthArea::priority = 5;
    Parameters::Filters::DepthHomogeneity::priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 0;

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_0_RGBD_MODE(
      7, // Seven RGB + D filters in total
         // (Nine excluding the ones using textures)
      std::vector< float >( conveyor.size(), 0.0 ) );

    // A dummy histogram
    std::vector<cv::MatND> histogram;

    // Test all RGB and Depth filters
    int filteringMode = RGBD_MODE;

    // Run Filters::applyFilters
    Filters::applyFilters(
      conveyor,
      filteringMode,
      depthSquares_,
      rgbSquares_,
      histogram,
      cloud,
      holesMasksSetVector_0,
      holesMasksImageVector_0,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      intermediatePointsSetVector_0,
      intermediatePointsImageVector_0,
      &probabilitiesVector2D_0_RGBD_MODE );

    // Filter-wise
    for ( int f = 0; f < probabilitiesVector2D_0_RGBD_MODE.size(); f++ )
    {
      if ( f == Parameters::Filters::DepthDiff::priority - 1 )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::RectanglePlaneConstitution::priority - 1 )
      {
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::DepthArea::priority - 1 )
      {
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1 )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::DepthHomogeneity::priority - 1)
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector2D_0_RGBD_MODE[f][0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_0_RGBD_MODE[f][1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_0_RGBD_MODE[f][2], 0.1 );
        EXPECT_LT ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::ColourHomogeneity::rgbd_priority - 1)
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_LT ( 0.8, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }

      if (f == Parameters::Filters::LuminosityDiff::rgbd_priority - 1)
      {
        // All probabilities amount to zero: the size of each set inside the
        // intermediatePointsSetVector_0 vector is zero
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGBD_MODE[f][4] );
      }
    }


    // ---------------------------- RGB_ONLY_MODE ------------------------------

    // Set the order of the filters' execution in random
    Parameters::Filters::ColourHomogeneity::rgb_priority = 2;
    Parameters::Filters::LuminosityDiff::rgb_priority = 1;
    Parameters::Filters::TextureDiff::rgb_priority = 0;
    Parameters::Filters::TextureBackprojection::rgb_priority = 0;

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_0_RGB_ONLY_MODE(
      2, // (Four excluding the ones using textures)
      std::vector< float >( conveyor.size(), 0.0 ) );

    // Test only the RGB filters
    filteringMode = RGB_ONLY_MODE;

    // Run Filters::applyFilters
    Filters::applyFilters(
      conveyor,
      filteringMode,
      depthSquares_,
      rgbSquares_,
      histogram,
      cloud,
      holesMasksSetVector_0,
      holesMasksImageVector_0,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      intermediatePointsSetVector_0,
      intermediatePointsImageVector_0,
      &probabilitiesVector2D_0_RGB_ONLY_MODE );

    // Filter-wise
    for ( int f = 0; f < probabilitiesVector2D_0_RGB_ONLY_MODE.size(); f++ )
    {
      if ( f == Parameters::Filters::ColourHomogeneity::rgb_priority - 1)
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][3] );
        EXPECT_LT ( 0.8, probabilitiesVector2D_0_RGB_ONLY_MODE[f][4] );
      }

      if (f == Parameters::Filters::LuminosityDiff::rgb_priority - 1)
      {
        // All probabilities amount to zero: the size of each set inside the
        // intermediatePointsSetVector_0 vector is zero
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_0_RGB_ONLY_MODE[f][4] );
      }
    }

    ////////////////////////// Inflations size : 10 ////////////////////////////

    // Create the needed by the Filters::applyFilters method vectors
    std::vector< std::set<unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      depthSquares_,
      &holesMasksSetVector_10);

    // The vector of mask images
    std::vector< cv::Mat > holesMasksImageVector_10;

    FiltersResources::createHolesMasksImageVector(
      conveyor,
      depthSquares_,
      &holesMasksImageVector_10 );

    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_10;
    std::vector< int > inflatedRectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      depthSquares_,
      10,
      &inflatedRectanglesVector_10,
      &inflatedRectanglesIndices_10 );

    // The intermediate points vector of sets
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // The intermediate points vector of images
    std::vector< cv::Mat > intermediatePointsImageVector_10;

    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      depthSquares_,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      &intermediatePointsImageVector_10 );

    // ------------------------------ RGBD_MODE --------------------------------

    // Set the order of the filters' execution in random
    Parameters::Filters::DepthDiff::priority = 1;
    Parameters::Filters::ColourHomogeneity::rgbd_priority = 2;
    Parameters::Filters::RectanglePlaneConstitution::priority = 3;
    Parameters::Filters::LuminosityDiff::rgbd_priority = 4;
    Parameters::Filters::DepthArea::priority = 5;
    Parameters::Filters::DepthHomogeneity::priority = 6;
    Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 7;
    Parameters::Filters::TextureDiff::rgbd_priority = 0;
    Parameters::Filters::TextureBackprojection::rgbd_priority = 0;

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_10_RGBD_MODE(
      7, // Seven RGB + D filters in total
         // (Nine excluding the ones using textures)
      std::vector< float >( conveyor.size(), 0.0 ) );


    // Test all RGB and Depth filters
    filteringMode = RGBD_MODE;

    // Run Filters::applyFilters
    Filters::applyFilters(
      conveyor,
      filteringMode,
      depthSquares_,
      rgbSquares_,
      histogram,
      cloud,
      holesMasksSetVector_10,
      holesMasksImageVector_10,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      intermediatePointsSetVector_10,
      intermediatePointsImageVector_10,
      &probabilitiesVector2D_10_RGBD_MODE );

    // Filter-wise
    for ( int f = 0; f < probabilitiesVector2D_10_RGBD_MODE.size(); f++ )
    {
      if ( f == Parameters::Filters::DepthDiff::priority - 1 )
      {
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::RectanglePlaneConstitution::priority - 1 )
      {
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::DepthArea::priority - 1 )
      {
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_EQ ( 1.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1 )
      {
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::DepthHomogeneity::priority - 1)
      {
        // The east and south edges of the lower right square are clipped
        EXPECT_NEAR ( 196 / 9996, probabilitiesVector2D_10_RGBD_MODE[f][0], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_10_RGBD_MODE[f][1], 0.1 );
        EXPECT_NEAR ( 392 / 9996, probabilitiesVector2D_10_RGBD_MODE[f][2], 0.1 );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if ( f == Parameters::Filters::ColourHomogeneity::rgbd_priority - 1)
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_LT ( 0.8, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }

      if (f == Parameters::Filters::LuminosityDiff::rgbd_priority - 1)
      {
        // All probabilities amount to zero: the size of each set inside the
        // intermediatePointsSetVector_0 vector is zero
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][2] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][3] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGBD_MODE[f][4] );
      }
    }


    // ---------------------------- RGB_ONLY_MODE ------------------------------

    // Set the order of the filters' execution in random
    Parameters::Filters::ColourHomogeneity::rgb_priority = 2;
    Parameters::Filters::LuminosityDiff::rgb_priority = 1;
    Parameters::Filters::TextureDiff::rgb_priority = 0;
    Parameters::Filters::TextureBackprojection::rgb_priority = 0;

    // Apply all active filters and obtain a 2D vector containing the
    // probabilities of validity of each candidate hole, produced by all
    // active filters
    std::vector<std::vector<float> > probabilitiesVector2D_10_RGB_ONLY_MODE(
      2, // (Four excluding the ones using textures)
      std::vector< float >( conveyor.size(), 0.0 ) );

    // Test only the RGB filters
    filteringMode = RGB_ONLY_MODE;

    // Run Filters::applyFilters
    Filters::applyFilters(
      conveyor,
      filteringMode,
      depthSquares_,
      rgbSquares_,
      histogram,
      cloud,
      holesMasksSetVector_10,
      holesMasksImageVector_10,
      inflatedRectanglesVector_10,
      inflatedRectanglesIndices_10,
      intermediatePointsSetVector_10,
      intermediatePointsImageVector_10,
      &probabilitiesVector2D_10_RGB_ONLY_MODE );

    // Filter-wise
    for ( int f = 0; f < probabilitiesVector2D_10_RGB_ONLY_MODE.size(); f++ )
    {
      if ( f == Parameters::Filters::ColourHomogeneity::rgb_priority - 1)
      {
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][2] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][3] );
        EXPECT_LT ( 0.8, probabilitiesVector2D_10_RGB_ONLY_MODE[f][4] );
      }

      if (f == Parameters::Filters::LuminosityDiff::rgb_priority - 1)
      {
        // All probabilities amount to zero: the size of each set inside the
        // intermediatePointsSetVector_0 vector is zero
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][0] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][1] );
        EXPECT_EQ ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][2] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][3] );
        EXPECT_LT ( 0.0, probabilitiesVector2D_10_RGB_ONLY_MODE[f][4] );
      }
    }
  }
}  // namespace hole_fusion
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
