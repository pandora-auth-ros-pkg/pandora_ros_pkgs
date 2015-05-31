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

#include "hole_fusion_node/hole_fusion.h"
#include "utils/parameters.h"
#include "gtest/gtest.h"
#include <stdlib.h>
#include <sys/time.h>

namespace pandora_vision
{
  /**
    @class FiltersTest
    @brief Tests the integrity of methods of class Filters
   **/
  class HoleFusionTest : public ::testing::Test
  {
    protected:

      //HoleFusionTest() : cloud ( new PointCloud ) {}

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
        @brief Constructs a rectangle with highly variant color of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted
        return void
       **/
      void generateNonHomogeneousDepthRectangle (
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const int& range,
          cv::Mat* image );

      /**
        @brief Constructs a rectangle with highly variant color between border and internal of width @param x and height of @param y.
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The rectangle's width
        @param[in] y [const int&] The rectangle's height
        @param[in] diff [const float&] The difference between border and internal
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted
        return void
       **/
      void generateBorderInternalDifferenceDepthRectangle (
          const cv::Point2f& upperLeft,
          const int& x,
          const int& y,
          const float& diff,
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
      PointCloudPtr pointCloud_; 

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
  void HoleFusionTest::generateDepthRectangle (
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
    @brief Constructs a rectangle with highly variant color of width @param x and height of @param y.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted
    return void
   **/
  void HoleFusionTest::generateNonHomogeneousDepthRectangle (
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const int& range,
      cv::Mat* image )
  {
    // Fill the inside of the desired rectangle with the @param depthIn provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      // The seed for the random depth value
      unsigned int seed = 0;
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = rand_r(&seed) % range;
      }
    }
  }


  /**
    @brief Constructs a rectangle with highly variant color between border and internal of width @param x and height of @param y.
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[in] diff [const float&] The difference between border and internal
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted
    return void
   **/
  void HoleFusionTest::generateBorderInternalDifferenceDepthRectangle(
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y,
      const float& diff,
      cv::Mat* image )
  {
    float borderColor = 2.0;
    // Fill the border of the desired rectangle with a color.
    for( int rows = upperLeft.y; rows < upperLeft.y + 10; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = borderColor;
      }
    }

    for( int rows = upperLeft.y + y - 10; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = borderColor;
      }
    }

    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + 10; cols++ )
      {
        image->at< float >( rows, cols ) = borderColor;
      }
    }

    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x + x - 10; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = borderColor;
      }
    }

    // Fill the internal of the desired rectangle with color bigger from the border.
    // The difference is @param diff
    for( int rows = upperLeft.y + 10; rows < upperLeft.y + y - 10; rows++ )
    {
      for ( int cols = upperLeft.x + 10; cols < upperLeft.x + x - 10; cols++ )
      {
        image->at< float >( rows, cols ) = borderColor + diff;
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
  void HoleFusionTest::generateRgbRectangle (
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
  HolesConveyor HoleFusionTest::getConveyor (
      const cv::Point2f& upperLeft,
      const int& x,
      const int& y )
  {
    // What will be returned: the internal elements of one hole
    HolesConveyor hole;

    // The hole's keypoint
    cv::Point2f k (  upperLeft.x + x / 2, upperLeft.y + y / 2 );

    hole.keypoint.push_back( k );

    cv::Rect rect(upperLeft.x, upperLeft.y, x, y);
    hole.rectangle.push_back(rect);

    return hole;
  }



  //! Tests HoleFusion::validateDistance
  TEST_F ( HoleFusionTest, validateDistanceTest )
  {

    ///////////// Construct the depth image ////////////

    // The image upon which the small depth val area and the big depth val area will be inprinted
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

    // Construct the small depth value square area
    cv::Mat depthBigValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        150,
        150,
        1.0,
        &depthBigValSquare );


    // Construct the big depth value square area
    cv::Mat depthSmallValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 50, 50 ),
        100,
        100,
        2.55,
        &depthSmallValSquare );

    // Construct the big depth value square area for the very big rgb square 
    cv::Mat depthBigBigValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 200, 200),
        100,
        100,
        3.1,
        &depthBigBigValSquare );

    // Compose the final depthSquares_ image
    depthSquares_ +=
      depthBigValSquare +
      depthSmallValSquare +
      depthBigBigValSquare;

    /////////////////////// Construct the rgb image ////////////////////////

    // Construct the square inside the small depth val area
    cv::Mat rgbInsideBigDepthValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( WIDTH - 125, HEIGHT - 125 ),
        10,
        10,
        140,
        &rgbInsideBigDepthValSquare );


    // Construct the square inside the big depth val area
    cv::Mat rgbInsideSmallDepthValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( 100, 100 ),
        25,
        25,
        140,
        &rgbInsideSmallDepthValSquare );

    // Construct the big square inside the big depth val area
    cv::Mat rgbInsideBigBigDepthValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( 210, 210 ),
        85,
        85,
        140,
        &rgbInsideBigBigDepthValSquare );

    std::vector<bool> realContours(2, true);
    HolesConveyor conveyorTemp;
    conveyorTemp = getConveyor(
        cv::Point2f ( WIDTH - 125, HEIGHT - 125 ),
        10,
        10 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 100, 100 ),
        25,
        25 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 210, 210 ),
        85,
        85 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    HoleFusion::validateDistance(depthSquares_, &conveyor, &realContours, pointCloud_);
    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    int counter = 0;
    for(int i = 0; i < 2; i ++)
    {
      if(realContours[i])
        counter++;
    }

    // Expected that the small hole on the small depth val area 
    // and the big hole on the big depth val area  will be eliminated
    EXPECT_EQ ( 1, counter);
  }


  //! Tests HoleFusion::mergeHoles
  TEST_F ( HoleFusionTest, mergeHolesTest )
  {

    ///////////// Construct the depth image ////////////

    // The image upon which the big depth val area, 
    // the small depth val area, 
    // the big depth variance area and the small depth variance 
    // will be inprinted
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

    // Construct the big depth value square area
    cv::Mat depthBigValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( WIDTH - 50, HEIGHT - 50 ),
        50,
        50,
        3.1,
        &depthBigValSquare );


    // Construct the small depth value square area
    cv::Mat depthSmallValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 20, 20),
        50,
        50,
        1.2,
        &depthSmallValSquare );

    // Compose the final depthSquares_ image
    depthSquares_ +=
      depthBigValSquare +
      depthSmallValSquare;

    /////////////////////// Construct the rgb image ////////////////////////

    // Construct the square inside the big depth val area
    cv::Mat rgbInsideBigDepthValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( WIDTH - 40, HEIGHT - 40 ),
        10,
        10,
        140,
        &rgbInsideBigDepthValSquare );


    // Construct the square inside the small depth val area
    cv::Mat rgbInsideSmallDepthValSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( 30, 30 ),
        5,
        5,
        140,
        &rgbInsideSmallDepthValSquare );

    std::vector<bool> realContours(2, true);
    HolesConveyor conveyorTemp;
    conveyorTemp = getConveyor(
        cv::Point2f ( WIDTH - 40, HEIGHT - 40 ),
        10,
        10 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 30, 30 ),
        5,
        5 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    HoleFusion::validateDistance(depthSquares_, &conveyor, &realContours, pointCloud_);
    int counter = 0;
    for(int i = 0; i < 2; i ++)
    {
      if(realContours[i])
        counter++;
    }

    // Expected that the small hole on the small depth val area will be eliminated
    EXPECT_EQ ( 1, counter);

    // Construct the small depth variance square area
    cv::Mat depthSmallVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateNonHomogeneousDepthRectangle
      ( cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        70,
        70,
        2,
        &depthSmallVarSquare );


    // Construct the medium depth variance square area
    cv::Mat depthMedVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateNonHomogeneousDepthRectangle( 
        cv::Point2f ( 80, 80 ),
        70,
        70,
        3,
        &depthMedVarSquare );

    cv::Mat depthBigVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );
    // unstuffed removal method 0 means remove holes on depth area with big overall depth variance, 
    // while 1 means remove holes on depth area with big difference avg from points at the border 
    // and all other points until the rectangle's center point
    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      // Construct the big depth variance square area

      HoleFusionTest::generateNonHomogeneousDepthRectangle( 
          cv::Point2f ( 200, 200 ),
          70,
          70,
          10,
          &depthBigVarSquare );
    }
    else
    {
      // Construct the square area with big depth variance between border and internal

      HoleFusionTest::generateBorderInternalDifferenceDepthRectangle( 
          cv::Point2f ( 200, 200 ),
          50,
          50,
          5.0,
          &depthBigVarSquare );
    }

    // Construct the very small depth variance square area
    cv::Mat depthSmallSmallVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateNonHomogeneousDepthRectangle( 
        cv::Point2f ( 200, 200 ),
        70,
        70,
        1,
        &depthSmallSmallVarSquare );

    // Compose the final depthSquares_ image
    depthSquares_ +=
      depthBigVarSquare +
      depthSmallVarSquare +
      depthMedVarSquare;


    /////////////////////// Construct the rgb image ////////////////////////

    // Construct the square inside the medium depth variance area
    cv::Mat rgbInsideMedDepthVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        40,
        40,
        140,
        &rgbInsideMedDepthVarSquare );


    // Construct the square inside the small depth variance area
    cv::Mat rgbInsideSmallDepthVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

    HoleFusionTest::generateRgbRectangle
      ( cv::Point2f ( 90, 90 ),
        40,
        40,
        140,
        &rgbInsideSmallDepthVarSquare );

    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      // Construct the square inside the big depth variance area
      cv::Mat rgbInsideBigDepthVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

      HoleFusionTest::generateRgbRectangle
        ( cv::Point2f ( 220, 220),
          40,
          40,
          140,
          &rgbInsideBigDepthVarSquare );
    }
    else
    {
      // Construct the square inside the big depth variance area
      cv::Mat rgbInsideBigDepthVarSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

      HoleFusionTest::generateRgbRectangle
        ( cv::Point2f ( 200, 200),
          50,
          50,
          140,
          &rgbInsideBigDepthVarSquare );
    }

    //std::vector<bool> realContours(4, true);
    conveyorTemp = getConveyor(
        cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 90, 90 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 220, 220 ),
          40,
          40 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    else
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 200, 200 ),
          50,
          50 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    HolesConveyor depthConveyor;
    HolesConveyor thermalConveyor;
    HolesConveyor preValidatedHoles;
    std::map<int, float> validHolesMap;
    HoleFusion::mergeHoles(
        &conveyor, 
        &depthConveyor, 
        &thermalConveyor, 
        depthSquares_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);

    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      // Expected that the small hole on the small depth val area 
      // and the big hole on the big depth val area will be eliminated, 
      // and also the two holes inside the small depth variance area 
      // and the hole on the big depth variance area
      EXPECT_EQ ( 1, conveyor.rectangle.size());
    }
    else
    {
      // Expected that the small hole on the small depth val area 
      // and the big hole on the big depth val area will be eliminated, 
      // and also the two holes inside the small depth variance area 
      // and the hole on the big depth variance between border and internal area
      EXPECT_EQ ( 1, conveyor.rectangle.size());
    }

    // Construct the overlapping depth hole with the last rgb hole left
    cv::Mat depthOverlappingSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 70, 70 ),
        60,
        60,
        1.9,
        &depthOverlappingSquare );

    conveyorTemp = getConveyor(
        cv::Point2f ( 70, 70 ),
        60,
        60 );
    depthConveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    depthConveyor.rectangle.push_back(conveyorTemp.rectangle[0]);

    preValidatedHoles.keypoint.clear();
    preValidatedHoles.rectangle.clear();
    validHolesMap.clear();

    HoleFusion::mergeHoles(
        &conveyor, 
        &depthConveyor, 
        &thermalConveyor, 
        depthSquares_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);

    // It is expected that there is only one hole with probability 0.6
    EXPECT_EQ ( 1, preValidatedHoles.rectangle.size());
    EXPECT_NEAR ( 0.6, validHolesMap[0], 0.05);

    // Generate thermal small squares to merge with the aboveset
    // overlapping rgb and depth squares


    cv::Mat thermalOverlappingSquare1 = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 65, 65 ),
        10,
        10,
        1.9,
        &thermalOverlappingSquare1 );

    conveyorTemp = getConveyor(
        cv::Point2f ( 65, 65 ),
        10,
        10 );
    thermalConveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    thermalConveyor.rectangle.push_back(conveyorTemp.rectangle[0]);

    cv::Mat thermalOverlappingSquare2 = cv::Mat::zeros(HEIGHT, WIDTH, CV_32FC1 );

    HoleFusionTest::generateDepthRectangle
      ( cv::Point2f ( 95, 95 ),
        25,
        25,
        1.9,
        &thermalOverlappingSquare2 );

    conveyorTemp = getConveyor(
        cv::Point2f ( 95, 95 ),
        25,
        25 );
    thermalConveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    thermalConveyor.rectangle.push_back(conveyorTemp.rectangle[0]);

    preValidatedHoles.keypoint.clear();
    preValidatedHoles.rectangle.clear();
    validHolesMap.clear();

    HoleFusion::mergeHoles(
        &conveyor, 
        &depthConveyor, 
        &thermalConveyor, 
        depthSquares_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);

    // It is expected that there is only one hole with probability 1.0
    // and keypoint at (96.875, 96.875)
    EXPECT_EQ ( 1, preValidatedHoles.rectangle.size());
    EXPECT_EQ ( 1.0, validHolesMap[0]);
    EXPECT_NEAR(96.875, preValidatedHoles.keypoint[0].x, 5);
    EXPECT_NEAR(96.875, preValidatedHoles.keypoint[0].y, 5);

    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    // Test that for very small depth, 
    // the small variance threshold is used to eliminate RGB contours 
    // (thus just a small variance inside the hole's bounding box 
    // does not mean invalid RGB hole)

    // Set the depth for each point of the depthSquares_ image to 0.3
    for ( int rows = 0; rows < depthSquares_.rows; rows++ )
    {
      for ( int cols = 0; cols < depthSquares_.cols; cols++ )
      {
        depthSquares_.at< float >( rows, cols ) = 0.3;
      }
    }
    // Compose the final depthSquares_ image
    depthSquares_ +=
      depthBigVarSquare +
      depthSmallVarSquare +
      depthMedVarSquare;
    conveyorTemp = getConveyor(
        cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 90, 90 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 220, 220 ),
          40,
          40 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    else
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 200, 200 ),
          50,
          50 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    preValidatedHoles.keypoint.clear();
    preValidatedHoles.rectangle.clear();
    validHolesMap.clear();
    depthConveyor.keypoint.clear();
    depthConveyor.rectangle.clear();
    thermalConveyor.keypoint.clear();
    thermalConveyor.rectangle.clear();

    HoleFusion::mergeHoles(
        &conveyor, 
        &depthConveyor, 
        &thermalConveyor, 
        depthSquares_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);
    // It is expected that there are two valid holes those inside small and medium depth variance
    EXPECT_EQ ( 2, preValidatedHoles.rectangle.size());

    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
    // Set the depth for each point of the depthSquares_ image to 0.3
    for ( int rows = 0; rows < depthSquares_.rows; rows++ )
    {
      for ( int cols = 0; cols < depthSquares_.cols; cols++ )
      {
        depthSquares_.at< float >( rows, cols ) = 0.3;
      }
    }
    // Compose the final depthSquares_ image
    depthSquares_ +=
      depthBigVarSquare +
      depthSmallSmallVarSquare +
      depthMedVarSquare;
    conveyorTemp = getConveyor(
        cv::Point2f ( WIDTH - 150, HEIGHT - 150 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    conveyorTemp = getConveyor(
        cv::Point2f ( 90, 90 ),
        40,
        40 );
    conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
    conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    if(Parameters::HoleFusion::unstuffed_removal_method == 0)
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 220, 220 ),
          40,
          40 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    else
    {
      conveyorTemp = getConveyor(
          cv::Point2f ( 200, 200 ),
          50,
          50 );
      conveyor.keypoint.push_back(conveyorTemp.keypoint[0]);
      conveyor.rectangle.push_back(conveyorTemp.rectangle[0]);
    }
    preValidatedHoles.keypoint.clear();
    preValidatedHoles.rectangle.clear();
    validHolesMap.clear();
    depthConveyor.keypoint.clear();
    depthConveyor.rectangle.clear();

    HoleFusion::mergeHoles(
        &conveyor, 
        &depthConveyor, 
        &thermalConveyor, 
        depthSquares_, 
        pointCloud_, 
        &preValidatedHoles, 
        &validHolesMap);
    // It is expected that there is one valid hole the onee inside the medium variance square
    EXPECT_EQ ( 1, preValidatedHoles.rectangle.size());

    conveyor.keypoint.clear();
    conveyor.rectangle.clear();
  }


} // namespace pandora_vision
