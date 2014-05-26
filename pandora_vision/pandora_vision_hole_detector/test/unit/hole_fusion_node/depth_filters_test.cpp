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

      DepthFiltersTest() {}

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
      //! (WIDTH - 1, HEIGHT - 1)
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // The image upon which the squares will be inprinted
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // The overall conveyor holding the holes
        HolesConveyor conveyor;

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
            1.2,
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
            2.2,
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
            1.8,
            &upperLeftSquare );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor);

        // Synthesize the final squares_ image
        squares_ = lowerRightSquare + upperRightSquare + upperLeftSquare;

      }


      // The images' width and height
      int WIDTH;
      int HEIGHT;

      // The image that will be used to locate blobs in
      cv::Mat squares_;

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
    if ( image->type() != CV_32FC1 )
    {
      std::cerr << "Image of invalid type. Please use CV_32FC1" << std::endl;

      return;
    }

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

} // namespace pandora_vision
