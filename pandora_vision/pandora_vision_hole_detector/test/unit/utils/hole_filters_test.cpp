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

#include "utils/hole_filters.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class HoleFiltersTest
    @brief Tests the integrity of methods of class HoleFilters
   **/
  class HoleFiltersTest : public ::testing::Test
  {
    protected:

      HoleFiltersTest () {}

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
        cv::Mat* image,
        std::vector<cv::Point2f>* vertices);

      //! Sets up one image: squares_, which features four non-zero value
      //! squares of various sizes.
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Four squares with vertices
        // A (100, 100), B (100, 200), C (200, 200), D (200, 100),
        // A2 (90, 90), B2 (90, 210), C2 (210, 210), D2 (210, 90),
        // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
        // B' (WIDTH - 1 - 100, HEIGHT - 1)
        // C' (WIDTH - 1, HEIGHT - 1)
        // D' (WIDTH - 1, HEIGHT - 1 - 100) and
        // A'' (200, 200), B'' (200, 300), C'' (300, 300), D'' (300, 200)
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );


        // The two square outlines
        cv::Mat upperLeftSquare = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        cv::Mat upperLeftSquare2 = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        cv::Mat lowerRightSquare = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        cv::Mat middleSquare = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );

        // Construct the upperLeftSquare image
        HoleFiltersTest::generateRectangle
          ( cv::Point2f ( 100, 100 ), 100, 100,
            &upperLeftSquare, &upperLeftSquareVertices_);

        // Construct the upperLeftSquare2 image
        HoleFiltersTest::generateRectangle
          ( cv::Point2f ( 90, 90 ), 120, 120,
            &upperLeftSquare2, &upperLeftSquareVertices_2_ );

        // Construct the lower right square
        HoleFiltersTest::generateRectangle
          ( cv::Point2f ( WIDTH - 100, HEIGHT - 100),
            100,
            100,
            &lowerRightSquare,
            &lowerRightSquareVertices_ );

        // Construct the middle square
        HoleFiltersTest::generateRectangle
          ( cv::Point2f ( 200, 200 ),
            100,
            100,
            &middleSquare,
            &middleSquareVertices_ );

        // Obtain the squares' outline points
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            if (upperLeftSquare.at< unsigned char >( rows, cols ) != 0)
            {
              upperLeftSquareOutline_.push_back( cv::Point2f(cols, rows) );
            }

            if (upperLeftSquare2.at< unsigned char >( rows, cols ) != 0)
            {
              upperLeftSquareOutline_2_.push_back( cv::Point2f(cols, rows) );
            }

            if (lowerRightSquare.at< unsigned char >( rows, cols ) != 0)
            {
              lowerRightSquareOutline_.push_back( cv::Point2f(cols, rows) );
            }

            if (middleSquare.at< unsigned char >( rows, cols ) != 0)
            {
              middleSquareOutline_.push_back( cv::Point2f(cols, rows) );
            }
          }
        }

        // The total squares_ image is the sum of all the square images
        squares_ = upperLeftSquare + upperLeftSquare2
          + lowerRightSquare + middleSquare;

      }


      int WIDTH;
      int HEIGHT;

      // Four squares with vertices
      // A (100, 100), B (100, 200), C (200, 200), D (200, 100),
      // A2 (90, 90), B2 (90, 210), C2 (210, 210), D2 (210, 90),
      // A' (WIDTH - 1 - 100, HEIGHT - 1 - 100),
      // B' (WIDTH - 1 - 100, HEIGHT - 1)
      // C' (WIDTH - 1, HEIGHT - 1)
      // D' (WIDTH - 1, HEIGHT - 1 - 100) and
      // A'' (200, 200), B'' (200, 300), C'' (300, 300), D'' (300, 200)
      cv::Mat squares_;

      // Each square's outline points
      std::vector< cv::Point2f > upperLeftSquareOutline_;
      std::vector< cv::Point2f > upperLeftSquareOutline_2_;
      std::vector< cv::Point2f > lowerRightSquareOutline_;
      std::vector< cv::Point2f > middleSquareOutline_;

      // Each square's vertices
      std::vector< cv::Point2f > upperLeftSquareVertices_;
      std::vector< cv::Point2f > upperLeftSquareVertices_2_;
      std::vector< cv::Point2f > lowerRightSquareVertices_;
      std::vector< cv::Point2f > middleSquareVertices_;
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
  void HoleFiltersTest::generateRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    cv::Mat* image,
    std::vector<cv::Point2f>* vertices)
  {
    // The four vertices of the rectangle
    cv::Point2f vertex_1(upperLeft.x, upperLeft.y);

    cv::Point2f vertex_2(upperLeft.x, upperLeft.y + y - 1);

    cv::Point2f vertex_3(upperLeft.x + x - 1, upperLeft.y + y - 1);

    cv::Point2f vertex_4(upperLeft.x + x - 1, upperLeft.y);

    cv::Point2f a[] = {vertex_1, vertex_2, vertex_3, vertex_4};

    vertices->push_back( vertex_1 );
    vertices->push_back( vertex_2 );
    vertices->push_back( vertex_3 );
    vertices->push_back( vertex_4 );

    for(unsigned int j = 0; j < 4; j++)
    {
      cv::line(*image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8);
    }
  }



  //! Test HoleFilters::validateKeypointsToRectangles
  TEST_F ( HoleFiltersTest, ValidateKeypointsToRectanglesTest )
  {
    // Test two keypoints VS four rectangles
    cv::KeyPoint upperLeftKeypoint ( 150, 150, 1 );
    cv::KeyPoint lowerRightKeypoint ( WIDTH - 50, HEIGHT - 50, 1 );

    // The inKeyPoints argument
    std::vector< cv::KeyPoint > inKeyPoints;
    inKeyPoints.push_back ( upperLeftKeypoint );
    inKeyPoints.push_back ( lowerRightKeypoint );

    // The inRectangles argument
    std::vector< std::vector< cv::Point2f > > inRectangles;
    inRectangles.push_back ( upperLeftSquareVertices_ );
    inRectangles.push_back ( upperLeftSquareVertices_2_ );
    inRectangles.push_back ( middleSquareVertices_ );
    inRectangles.push_back ( lowerRightSquareVertices_ );

    // The inRectanglesArea argument
    std::vector<float> inRectanglesArea;
    inRectanglesArea.push_back ( 10000.0 );
    inRectanglesArea.push_back ( 12100.0 );
    inRectanglesArea.push_back ( 10000.0 );
    inRectanglesArea.push_back ( 10000.0 );

    // The inContours argument
    std::vector<std::vector<cv::Point2f> > inContours;
    inContours.push_back ( upperLeftSquareOutline_ );
    inContours.push_back ( upperLeftSquareOutline_2_ );
    inContours.push_back ( middleSquareOutline_ );
    inContours.push_back ( lowerRightSquareOutline_ );

    // The HolesConveyor struct
    HolesConveyor conveyor;


    // Run HoleFilters::validateKeypointsToRectangles
    HoleFilters::validateKeypointsToRectangles
      ( inKeyPoints, inRectangles, inRectanglesArea, inContours, &conveyor);

    // There should be two entries in the conveyor
    ASSERT_EQ ( 2, HolesConveyorUtils::size( conveyor ) );

    // The amount of points in the outlines of the two holes should be
    // equal to 4 * 100 - 4
    EXPECT_EQ ( 396, conveyor.outlines[0].size() );
    EXPECT_EQ ( 396, conveyor.outlines[0].size() );

    // The first vertex of the first hole
    EXPECT_NEAR ( 100, conveyor.rectangles[0][0].x, 1 );
    EXPECT_NEAR ( 100, conveyor.rectangles[0][0].y, 1 );

    // The last vertex of the first hole
    EXPECT_NEAR ( 199, conveyor.rectangles[0][2].x, 1 );
    EXPECT_NEAR ( 199, conveyor.rectangles[0][2].y, 1 );

    // The first vertex of the second hole
    EXPECT_NEAR ( WIDTH - 100, conveyor.rectangles[1][0].x, 1 );
    EXPECT_NEAR ( HEIGHT - 100, conveyor.rectangles[1][0].y, 1 );

    // The last vertex of the second hole
    EXPECT_NEAR ( WIDTH - 1, conveyor.rectangles[1][2].x, 1 );
    EXPECT_NEAR ( HEIGHT - 1, conveyor.rectangles[1][2].y, 1 );

  }



  //! Test HoleFilters::validateBlobs
  TEST_F ( HoleFiltersTest, ValidateBlobsTest )
  {
    // The keyPoints argument
    std::vector< cv::KeyPoint > keyPoints;

    cv::KeyPoint upperLeftKeypoint ( 150, 150, 1 );
    cv::KeyPoint lowerRightKeypoint ( WIDTH - 50, HEIGHT - 50, 1 );

    keyPoints.push_back ( upperLeftKeypoint );
    keyPoints.push_back ( lowerRightKeypoint );

    // The HolesConveyor struct
    HolesConveyor conveyor;

    // Run HoleFilters::validateBlobs, using Brushfire
    HoleFilters::validateBlobs ( keyPoints, &squares_, 0, &conveyor);


    // There should be two entries in the conveyor
    ASSERT_EQ ( 2, HolesConveyorUtils::size( conveyor ) );

    // The amount of points in the outlines of the two holes should be
    // equal to 4 * 100 - 8
    EXPECT_EQ ( 392, conveyor.outlines[0].size() );
    EXPECT_EQ ( 392, conveyor.outlines[0].size() );

    // The first vertex of the first hole.
    // Upper right corner, going counter-clockwise
    EXPECT_NEAR ( 99, conveyor.rectangles[0][0].x, 2 );
    EXPECT_NEAR ( 199, conveyor.rectangles[0][0].y, 1 );

    // The last vertex of the first hole
    // Lower right corner, going counter-clockwise
    EXPECT_NEAR ( 199, conveyor.rectangles[0][3].x, 1 );
    EXPECT_NEAR ( 199, conveyor.rectangles[0][3].y, 1 );

    // The first vertex of the second hole
    // Upper right corner, going counter-clockwise
    EXPECT_NEAR ( WIDTH - 100, conveyor.rectangles[1][0].x, 1 );
    EXPECT_NEAR ( HEIGHT - 1, conveyor.rectangles[1][0].y, 1 );

    // The last vertex of the second hole
    // Lower right corner, going counter-clockwise
    EXPECT_NEAR ( WIDTH - 1, conveyor.rectangles[1][3].x, 1);
    EXPECT_NEAR ( HEIGHT - 1, conveyor.rectangles[1][3].y, 1 );



    // Run HoleFilters::validateBlobs, using Raycast
    // First, clear the conveyor

    HolesConveyorUtils::clear(&conveyor);

    HoleFilters::validateBlobs ( keyPoints, &squares_, 1, &conveyor);


    // There should be two entries in the conveyor
    ASSERT_EQ ( 2, HolesConveyorUtils::size( conveyor ) );

    // The amount of points in the outlines of the two holes should be
    // equal to 4 * 100 - 8
    EXPECT_EQ ( 390, conveyor.outlines[0].size() );
    EXPECT_EQ ( 390, conveyor.outlines[0].size() );

    // The first vertex of the first hole.
    // Lower right corner, going counter-clockwise
    EXPECT_NEAR ( 199, conveyor.rectangles[0][0].x, 1 );
    EXPECT_NEAR ( 199, conveyor.rectangles[0][0].y, 1 );

    // The last vertex of the first hole
    // Upper right corner, going counter-clockwise
    EXPECT_NEAR ( 199, conveyor.rectangles[0][3].x, 1 );
    EXPECT_NEAR ( 99, conveyor.rectangles[0][3].y, 1 );

    // The first vertex of the second hole
    // Lower right corner, going counter-clockwise
    EXPECT_NEAR ( WIDTH - 1, conveyor.rectangles[1][0].x, 1 );
    EXPECT_NEAR ( HEIGHT - 1, conveyor.rectangles[1][0].y, 1 );

    // The last vertex of the second hole
    // Lower right corner, going counter-clockwise
    EXPECT_NEAR ( WIDTH - 1, conveyor.rectangles[1][3].x, 1);
    EXPECT_NEAR ( HEIGHT - 100, conveyor.rectangles[1][3].y, 1 );

  }

} // namespace pandora_vision
