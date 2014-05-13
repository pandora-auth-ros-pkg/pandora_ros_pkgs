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

#include "utils/bounding_box_detection.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class BoundingBoxDetectionTest
    @brief Tests the integrity of methods of class BoundingBoxDetection
   **/
  class BoundingBoxDetectionTest : public ::testing::Test
  {
    protected:

      BoundingBoxDetectionTest () {}

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

        BoundingBoxDetectionTest::generateRectangle
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

        BoundingBoxDetectionTest::generateRectangle
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
  void BoundingBoxDetectionTest::generateRectangle (
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



  //! Test BoundingBoxDetection::findRotatedBoundingBoxesFromOutline
  TEST_F ( BoundingBoxDetectionTest, FindRotatedBoundingBoxesFromOutlineTest )
  {
    // The square's area
    std::vector< float > blobsArea;
    blobsArea.push_back( 10000 );
    blobsArea.push_back( 10000 );


    // Run BoundingBoxDetection::findRotatedBoundingBoxesFromOutline
    std::vector<std::vector<cv::Point2f> > outRectangles;
    BoundingBoxDetection::findRotatedBoundingBoxesFromOutline
      (squares_, squaresOutlinePointsVector_, blobsArea, &outRectangles);

    ASSERT_EQ ( 2, outRectangles.size() );


    // Clear the blobsArea vector and replace it with values that should mean
    // that the squares' area is lower than the accepted threshold
    blobsArea.clear();

    blobsArea.push_back( 0 );
    blobsArea.push_back( 0 );

    outRectangles.clear();

    // Run BoundingBoxDetection::findRotatedBoundingBoxesFromOutline
    BoundingBoxDetection::findRotatedBoundingBoxesFromOutline
      (squares_, squaresOutlinePointsVector_, blobsArea, &outRectangles);

    ASSERT_EQ ( 0, outRectangles.size() );
  }

} // namespace pandora_vision
