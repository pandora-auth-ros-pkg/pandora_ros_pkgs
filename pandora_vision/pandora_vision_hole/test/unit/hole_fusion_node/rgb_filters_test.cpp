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

#include "hole_fusion_node/rgb_filters.h"
#include "hole_fusion_node/filters_resources.h"
#include "hole_fusion_node/utils/histogram.h"
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
    @class RgbFiltersTest
    @brief Tests the integrity of methods of class RgbFilters
   **/
  class RgbFiltersTest : public ::testing::Test
  {
    protected:

      RgbFiltersTest() {}

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

      //! Sets up one image: squares_,
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
      //! (constructed to test the RgbFilters::checkHolesLuminosityDiff method)
      //! It creates the corresponding conveyor entries for these square holes.
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct the lower right square
        cv::Mat lowerRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

        RgbFiltersTest::generateRgbRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100,
            140,
            &lowerRightSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100 ),
          &conveyor);


        // Construct the upper right image
        cv::Mat upperRightSquare = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3 );

        RgbFiltersTest::generateRgbRectangle
          ( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100,
            160,
            &upperRightSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100 ),
          &conveyor);

        // Construct the upper left square
        cv::Mat upperLeftSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        RgbFiltersTest::generateRgbRectangle
          ( cv::Point2f ( 100, 100 ),
            100,
            100,
            180,
            &upperLeftSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor);

        // Construct the square surrounding the upper left square
        cv::Mat surroundingSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        RgbFiltersTest::generateRgbRectangle
          ( cv::Point2f ( 80, 80 ),
            140,
            140,
            1,
            &surroundingSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 80, 80 ),
            140,
            140 ),
          &conveyor);

        // Construct the middle square. In contrast to the other three
        // rectangles, this is scattered with random colours inside it
        cv::Mat middleSquare = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

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

            middleSquare.at< cv::Vec3b >( rows, cols ).val[0] = randomBlue;
            middleSquare.at< cv::Vec3b >( rows, cols ).val[1] = randomGreen;
            middleSquare.at< cv::Vec3b >( rows, cols ).val[2] = randomRed;
          }
        }

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 250, 250 ),
            100,
            100 ),
          &conveyor);


        // The image upon which the squares will be inprinted
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3 );

        // Compose the final squares_ image
        squares_ =
          lowerRightSquare +
          surroundingSquare +
          upperRightSquare +
          upperLeftSquare +
          middleSquare;

        // Construct the squares_ image. The entire image is at a colour of
        // value approximate the the colour value of the images of walls
        for ( int rows = 0; rows < squares_.rows; rows++ )
        {
          for ( int cols = 0; cols < squares_.cols; cols++ )
          {
            if (squares_.at< cv::Vec3b >( rows, cols ).val[0] == 0)
            {
              squares_.at< cv::Vec3b >( rows, cols ).val[0] = 116;
              squares_.at< cv::Vec3b >( rows, cols ).val[1] = 163;
              squares_.at< cv::Vec3b >( rows, cols ).val[2] = 171;
            }
          }
        }

      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;

      // The conveyor of holes that will be used to test methods of class
      // RgbFilters
      HolesConveyor conveyor;

  };



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
  void RgbFiltersTest::generateRgbRectangle (
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
  HolesConveyor RgbFiltersTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // A single hole
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

    // Push hole back into a HolesConveyor
    HolesConveyor conveyor;
    conveyor.holes.push_back( hole );

    return conveyor;

  }



  // Tests RgbFilters::checkHolesColorHomogeneity
  TEST_F ( RgbFiltersTest, checkHolesColorHomogeneityTest )
  {
    // Generate the needed resources
    std::vector<cv::Mat> holesMasksImageVector;
    FiltersResources::createHolesMasksImageVector(
      conveyor,
      squares_,
      &holesMasksImageVector );

    // The vector of probabilities returned
    std::vector<float> probabilitiesVector( conveyor.size(), 0.0 );
    std::vector<std::string> msgs;

    // Run RgbFilters::checkHolesColorHomogeneity
    RgbFilters::checkHolesColorHomogeneity(
      squares_,
      holesMasksImageVector,
      &probabilitiesVector,
      &msgs);

    // All squares except the one whose internal colours are randomly
    // generated are dull.
    EXPECT_EQ ( 0.0, probabilitiesVector[0] );
    EXPECT_EQ ( 0.0, probabilitiesVector[1] );
    EXPECT_EQ ( 0.0, probabilitiesVector[2] );
    EXPECT_EQ ( 0.0, probabilitiesVector[3] );
    EXPECT_LT ( 0.8, probabilitiesVector[4] );

  }



  //! Tests RgbFilters::checkHolesLuminosityDiff
  TEST_F ( RgbFiltersTest, checkHolesLuminosityDiffTest )
  {
    // Generate the needed resources for an inflation size of value 0

    // The vector of set masks
    std::vector< std::set< unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_0 );

    // The vectors of inflated rectangles and in-bounds inflated rectangles'
    // indices
    std::vector< std::vector< cv::Point2f > > rectanglesVector_0;
    std::vector< int > rectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &rectanglesVector_0,
      &rectanglesIndices_0 );

    // The intermediate points vector of sets
    std::vector<std::set<unsigned int> > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      rectanglesVector_0,
      rectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // The vector of probabilities returned
    std::vector< float > probabilitiesVector_0( conveyor.size(), 0.0 );
    std::vector< std::string > msgs;

    // Run RgbFilters::checkHolesLuminosityDiff
    RgbFilters::checkHolesLuminosityDiff(
      squares_,
      holesMasksSetVector_0,
      intermediatePointsSetVector_0,
      rectanglesIndices_0,
      &probabilitiesVector_0,
      &msgs);

    // All probabilities amount to zero: the size of each set inside the
    // intermediatePointsSetVector_0 vector is zero
    for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
    {
      EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
    }


    // Generate the needed resources for an inflation size of value 10

    // The vector of set masks
    std::vector< std::set< unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_10 );

    // The vectors of inflated rectangles and in-bounds inflated rectangles'
    // indices
    std::vector< std::vector< cv::Point2f > > rectanglesVector_10;
    std::vector< int > rectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &rectanglesVector_10,
      &rectanglesIndices_10 );

    // The intermediate points vector of sets
    std::vector< std::set< unsigned int> > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      rectanglesVector_10,
      rectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // The vector of probabilities returned
    std::vector< float > probabilitiesVector_10( conveyor.size(), 0.0 );
    msgs.clear();

    // Run RgbFilters::checkHolesLuminosityDiff
    RgbFilters::checkHolesLuminosityDiff(
      squares_,
      holesMasksSetVector_10,
      intermediatePointsSetVector_10,
      rectanglesIndices_10,
      &probabilitiesVector_10,
      &msgs);

    // The inflated rectangles of the lower right and upper right rectangles
    // are out of the image's bounds.
    // The upper left rectangle is surrounded by a black rectangle, so there goes
    // The surrounding rectangle and the middle one are just fine
    EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
    EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
    EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
    EXPECT_LT ( 0.8, probabilitiesVector_10[3] );
    EXPECT_LT ( 0.0, probabilitiesVector_10[4] );
  }



  //! Tests RgbFilters::checkHolesTextureBackProject
  TEST_F ( RgbFiltersTest, checkHolesTextureBackProjectTest )
  {
    // Generate the needed resources for an inflation size of value 0

    // The vector of set masks
    std::vector< std::set< unsigned int > > holesMasksSetVector_0;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_0 );

    // The vectors of inflated rectangles and in-bounds inflated rectangles'
    // indices
    std::vector< std::vector< cv::Point2f > > rectanglesVector_0;
    std::vector< int > rectanglesIndices_0;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &rectanglesVector_0,
      &rectanglesIndices_0 );

    // The intermediate points vector of sets
    std::vector<std::set<unsigned int> > intermediatePointsSetVector_0;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      rectanglesVector_0,
      rectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // The vector of probabilities returned
    std::vector< float > probabilitiesVector_0( conveyor.size(), 0.0 );
    std::vector< std::string > msgs;

    // Generate the histogram of walls
    std::vector<cv::MatND> histogram;
    Histogram::getHistogram
      ( &histogram, Parameters::Histogram::secondary_channel );

    // Run RgbFilters::checkHolesTextureBackProject
    RgbFilters::checkHolesTextureBackProject(
      squares_,
      histogram,
      holesMasksSetVector_0,
      intermediatePointsSetVector_0,
      rectanglesIndices_0,
      &probabilitiesVector_0,
      &msgs);

    // All probabilities amount to zero: the size of each set inside the
    // intermediatePointsSetVector_0 vector is zero
    for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
    {
      EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
    }


    // Generate the needed resources for an inflation size of value 10

    // The vector of set masks
    std::vector< std::set< unsigned int > > holesMasksSetVector_10;

    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector_10 );

    // The vectors of inflated rectangles and in-bounds inflated rectangles'
    // indices
    std::vector< std::vector< cv::Point2f > > rectanglesVector_10;
    std::vector< int > rectanglesIndices_10;

    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      10,
      &rectanglesVector_10,
      &rectanglesIndices_10 );

    // The intermediate points vector of sets
    std::vector< std::set< unsigned int> > intermediatePointsSetVector_10;

    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      rectanglesVector_10,
      rectanglesIndices_10,
      &intermediatePointsSetVector_10 );

    // The vector of probabilities returned
    std::vector< float > probabilitiesVector_10( conveyor.size(), 0.0 );
    msgs.clear();

    // Run RgbFilters::checkHolesTextureBackProject
    RgbFilters::checkHolesTextureBackProject(
      squares_,
      histogram,
      holesMasksSetVector_10,
      intermediatePointsSetVector_10,
      rectanglesIndices_10,
      &probabilitiesVector_10,
      &msgs);

    // The inflated rectangles of the lower right and upper right rectangles
    // are out of the image's bounds.
    // The upper left rectangle is surrounded by a black rectangle, so there goes
    // The surrounding rectangle and the middle one are just fine
    EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
    EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
    EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
    EXPECT_LT ( 0.0, probabilitiesVector_10[3] );
    EXPECT_LT ( 0.0, probabilitiesVector_10[4] );
  }



  //! Tests RgbFilters::checkHolesTextureDiff
  TEST_F ( RgbFiltersTest, checkHolesTextureDiffTest )
  {
    // Histogram generation : secondary channel toggle
    for ( int sec = 1; sec < 3; sec++ )
    {
      Parameters::Histogram::secondary_channel = sec;

      // Generate the needed resources for an inflation size of value 0

      // The vector of image masks
      std::vector< cv::Mat > holesMasksImageVector_0;

      FiltersResources::createHolesMasksImageVector(
        conveyor,
        squares_,
        &holesMasksImageVector_0 );

      // The vectors of inflated rectangles and in-bounds inflated rectangles'
      // indices
      std::vector< std::vector< cv::Point2f > > rectanglesVector_0;
      std::vector< int > rectanglesIndices_0;

      FiltersResources::createInflatedRectanglesVector(
        conveyor,
        squares_,
        0,
        &rectanglesVector_0,
        &rectanglesIndices_0 );

      // The intermediate points vector of images
      std::vector< cv::Mat > intermediatePointsImageVector_0;

      FiltersResources::createIntermediateHolesPointsImageVector(
        conveyor,
        squares_,
        rectanglesVector_0,
        rectanglesIndices_0,
        &intermediatePointsImageVector_0 );

      // There shouldn't be any non-zero pixels inside each mask
      for ( int i = 0; i < intermediatePointsImageVector_0.size(); i++ )
      {
        int nonZero = 0;
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for (int cols = 0; cols < WIDTH; cols++)
          {
            if (intermediatePointsImageVector_0[i].at
              < unsigned char >( rows, cols ) != 0)
            {
              nonZero++;
            }
          }
        }
        EXPECT_EQ ( 0, nonZero );
      }

      // The vector of probabilities returned
      std::vector< float > probabilitiesVector_0( conveyor.size(), 0.0 );
      std::vector< std::string > msgs;

      // Generate the histogram of walls
      std::vector<cv::MatND> histogram;
      Histogram::getHistogram
        ( &histogram, Parameters::Histogram::secondary_channel );

      // Run RgbFilters:checkHolesTextureDiff:
      RgbFilters::checkHolesTextureDiff(
        squares_,
        histogram,
        holesMasksImageVector_0,
        intermediatePointsImageVector_0,
        rectanglesIndices_0,
        &probabilitiesVector_0,
        &msgs);

      // All probabilities amount to zero: the size of each set inside the
      // intermediatePointsImageVector_0 vector is zero
      for ( int i = 0; i < probabilitiesVector_0.size(); i++ )
      {
        EXPECT_EQ ( 0.0, probabilitiesVector_0[i] );
      }
    }


    // Histogram generation : secondary channel toggle
    for ( int sec = 1; sec < 3; sec++ )
    {
      Parameters::Histogram::secondary_channel = sec;

      // Generate the needed resources for an inflation size of value 10

      // The vector of set masks
      std::vector< cv::Mat > holesMasksImageVector_10;

      FiltersResources::createHolesMasksImageVector(
        conveyor,
        squares_,
        &holesMasksImageVector_10 );

      // The vectors of inflated rectangles and in-bounds inflated rectangles'
      // indices
      std::vector< std::vector< cv::Point2f > > rectanglesVector_10;
      std::vector< int > rectanglesIndices_10;

      FiltersResources::createInflatedRectanglesVector(
        conveyor,
        squares_,
        10,
        &rectanglesVector_10,
        &rectanglesIndices_10 );

      // The intermediate points vector of images
      std::vector< cv::Mat > intermediatePointsImageVector_10;

      FiltersResources::createIntermediateHolesPointsImageVector(
        conveyor,
        squares_,
        rectanglesVector_10,
        rectanglesIndices_10,
        &intermediatePointsImageVector_10 );

      // There should be more than zero non-zero pixels inside each mask
      for ( int i = 0; i < intermediatePointsImageVector_10.size(); i++ )
      {
        int nonZero = 0;
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for (int cols = 0; cols < WIDTH; cols++)
          {
            if (intermediatePointsImageVector_10[i].at
              < unsigned char >( rows, cols ) != 0)
            {
              nonZero++;
            }
          }
        }
        EXPECT_LT ( 0, nonZero );
      }

      // The vector of probabilities returned
      std::vector< float > probabilitiesVector_10( conveyor.size(), 0.0 );
      std::vector< std::string > msgs;

      // Set the texture threshold to 0.9, or else all probabilities would be
      // equal to zero
      Parameters::Filters::TextureDiff::mismatch_texture_threshold = 0.8;

      // Generate the histogram of walls
      std::vector<cv::MatND> histogram;
      Histogram::getHistogram
        ( &histogram, Parameters::Histogram::secondary_channel );

      // Run RgbFilters:checkHolesTextureDiff:
      RgbFilters::checkHolesTextureDiff(
        squares_,
        histogram,
        holesMasksImageVector_10,
        intermediatePointsImageVector_10,
        rectanglesIndices_10,
        &probabilitiesVector_10,
        &msgs);

      if ( sec == 1 )
      {
        // For some reason, using the saturation as the secondary channel for
        // the generation of the histograms, gives all probabilities
        // equal to zero. Crappy behaviour.
        EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[3] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[4] );
      }

      if ( sec == 2 )
      {
        // The inflated rectangles of the lower right and upper right rectangles
        // are out of the image's bounds.
        // The upper left rectangle is surrounded by a black rectangle,
        // so there goes.
        // The surrounding rectangle and the middle one are just fine
        EXPECT_EQ ( 0.0, probabilitiesVector_10[0] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[1] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[2] );
        EXPECT_LT ( 0.0, probabilitiesVector_10[3] );
        EXPECT_EQ ( 0.0, probabilitiesVector_10[4] );
      }
    }
  }


}  // namespace hole_fusion
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
