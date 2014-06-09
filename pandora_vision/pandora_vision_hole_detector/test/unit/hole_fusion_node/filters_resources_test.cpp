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

#include "hole_fusion_node/filters_resources.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class FiltersResourcesTest
    @brief Tests the integrity of methods of class FiltersResources
   **/
  class FiltersResourcesTest : public ::testing::Test
  {
    protected:

      FiltersResourcesTest() {}

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

      virtual void SetUp ()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // An image needed only for its size
        squares_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 100, HEIGHT - 100 ),
            100,
            100 ),
          &conveyor );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( WIDTH - 103, 3 ),
            100,
            100 ),
          &conveyor );

        HolesConveyorUtils::append(
          getConveyor( cv::Point2f ( 100, 100 ),
            100,
            100 ),
          &conveyor );

        // In total, there should be three holes
        ASSERT_EQ ( 3, conveyor.size() );

      }

      // An image needed only for its size
      cv::Mat squares_;

      // Dimensions of the squares_ image
      int WIDTH;
      int HEIGHT;

      // The overall conveyor holding the holes
      HolesConveyor conveyor;

  };



  /**
    @brief Constructs the internals of a rectangular hole
    of width @param x and height of @param y
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The recgangle's width
    @param[in] y [const int&] The rectangle's height
    return [HolesConveyor] A struct containing the elements of one hole
   **/
  HolesConveyor FiltersResourcesTest::getConveyor (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y )
  {
    // A single hole
    HoleConveyor hole;

    // The hole's keypoint
    cv::KeyPoint k ( upperLeft.x + x / 2, upperLeft.y + y / 2 , 1 );

    hole.keypoint = k;


    // The four vertices of the rectangle
    cv::Point2f vertex_1( upperLeft.x, upperLeft.y );

    cv::Point2f vertex_2( upperLeft.x, upperLeft.y + y - 1 );

    cv::Point2f vertex_3( upperLeft.x + x - 1, upperLeft.y + y - 1 );

    cv::Point2f vertex_4( upperLeft.x + x - 1, upperLeft.y );

    std::vector< cv::Point2f > rectangle;
    rectangle.push_back( vertex_1 );
    rectangle.push_back( vertex_2 );
    rectangle.push_back( vertex_3 );
    rectangle.push_back( vertex_4 );

    hole.rectangle = rectangle;


    // The outline points of the hole will be obtained through the depiction
    // of the points consisting the rectangle
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );

    cv::Point2f a[] = { vertex_1, vertex_2, vertex_3, vertex_4 };

    for(unsigned int j = 0; j < 4; j++)
    {
      cv::line( image, a[j], a[(j + 1) % 4], cv::Scalar(255, 0, 0), 1, 8 );
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

    // Push back hole into a HolesConveyor
    HolesConveyor conveyor;
    conveyor.holes.push_back( hole );

    return conveyor;

  }



  //! Tests FiltersResources::createCheckerRequiredVectors
  TEST_F ( FiltersResourcesTest, CreateCheckerRequiredVectorsTest )
  {
    // The needed resources
    std::vector< cv::Mat > holesMasksImageVector;
    std::vector< std::set< unsigned int > > holesMasksSetVector;
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector;
    std::vector< int > inflatedRectanglesIndices;
    std::vector< cv::Mat > intermediatePointsImageVector;
    std::vector< std::set< unsigned int > > intermediatePointsSetVector;

    for ( int a = 0; a < 2; a++ )
    {
      Parameters::HoleFusion::run_checker_color_homogeneity = a;

      for ( int b = 0; b < 2; b++ )
      {
        Parameters::HoleFusion::run_checker_luminosity_diff = b;

        for ( int c = 0; c < 2; c++ )
        {
          Parameters::HoleFusion::run_checker_texture_diff = c;

          for ( int d = 0; d < 2; d++ )
          {
            Parameters::HoleFusion::run_checker_texture_backproject = d;

            for ( int e = 0; e < 2; e++ )
            {
              Parameters::HoleFusion::run_checker_depth_diff = e;

              for ( int f = 0; f < 2; f++ )
              {
                Parameters::HoleFusion::run_checker_depth_area = f;

                for ( int g = 0; g < 2; g++ )
                {
                  Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle = g;

                  for ( int h = 0; h < 2; h++ )
                  {
                    Parameters::HoleFusion::run_checker_outline_of_rectangle = h;

                    for ( int i = 0; i < 2; i++ )
                    {
                      Parameters::HoleFusion::run_checker_depth_homogeneity = i;

                      // Run FiltersResources::createCheckerRequiredVectors
                      FiltersResources::createCheckerRequiredVectors(
                        conveyor,
                        squares_,
                        2,
                        0,
                        &holesMasksImageVector,
                        &holesMasksSetVector,
                        &inflatedRectanglesVector,
                        &inflatedRectanglesIndices,
                        &intermediatePointsImageVector,
                        &intermediatePointsSetVector );


                      // Inquire about holesMasksImageVector
                      if ( a == 1 || c == 1)
                      {
                        // There should be three images of masks of holes
                        EXPECT_EQ ( 3, holesMasksImageVector.size() );

                        for ( int j = 0; j < holesMasksImageVector.size(); j++ )
                        {
                          int nonZero = 0;
                          for ( int rows = 0; rows < squares_.rows; rows++ )
                          {
                            for ( int cols = 0; cols < squares_.cols; cols++ )
                            {
                              if ( holesMasksImageVector[j].at
                                < unsigned char > ( rows, cols ) != 0)
                              {
                                nonZero++;
                              }
                            }
                          }

                          // There should be 100 X 100 - 4 points in each mask
                          EXPECT_EQ ( 10000 - 4, nonZero );
                        }
                      }
                      else if ( a != 1 && c != 1 )
                      {
                        // No masks if the corresponding filters to variables
                        // a and c are disabled
                        EXPECT_EQ ( 0, holesMasksImageVector.size() );
                      }


                      // Inquire about holesMasksSetVector
                      if ( b == 1 || d == 1 || f == 1 || i == 1 )
                      {
                        // There should be three masks of holes
                        EXPECT_EQ ( 3, holesMasksSetVector.size() );

                        for ( int j = 0; j < holesMasksSetVector.size(); j++ )
                        {
                          // Each mask should have 100 X 100 - 4 points
                          EXPECT_EQ ( 10000 - 4, holesMasksSetVector[j].size() );
                        }
                      }
                      else if ( b != 1 && d != 1 && f != 1 && i != 1 )
                      {
                        // No masks if the corresponding filters to variables
                        // b, d, f and i are disabled
                        EXPECT_EQ ( 0, holesMasksSetVector.size() );
                      }


                      // Inquire about inflatedRectangles*
                      if ( b == 1 || c == 1 || d == 1
                        || e == 1 || g == 1 || h == 1 )
                      {
                        // The number of rectangles should be equal to the
                        // number of indices
                        EXPECT_EQ ( inflatedRectanglesVector.size(),
                          inflatedRectanglesIndices.size() );

                        // In this case, with an inflation size of value 10,
                        // there should be two holes
                        EXPECT_EQ ( 2, inflatedRectanglesVector.size() );

                        for ( int j = 0; j < inflatedRectanglesVector.size(); j++ )
                        {
                          // Each rectangle should have exactly four vertices
                          EXPECT_EQ ( 4, inflatedRectanglesVector[j].size() );
                        }
                      }
                      else if (b != 1 && c != 1 && d != 1
                        && e != 1 && g != 1 && h != 1)
                      {
                        // The number of rectangles should be equal to the
                        // number of indices
                        EXPECT_EQ ( inflatedRectanglesVector.size(),
                          inflatedRectanglesIndices.size() );

                        // No masks if the corresponding filters to variables
                        // b, c, d, e, g and h are disabled
                        EXPECT_EQ ( 0, inflatedRectanglesVector.size() );
                      }


                      // Inquire about intermediatePointsImageVector
                      if ( c == 1 )
                      {
                        // There should be two masks of intermediate points
                        EXPECT_EQ ( 2, intermediatePointsImageVector.size() );

                        for ( int j = 0;
                          j < intermediatePointsImageVector.size(); j++ )
                        {
                          int nonZero = 0;
                          for ( int rows = 0; rows < squares_.rows; rows++ )
                          {
                            for ( int cols = 0; cols < squares_.cols; cols++ )
                            {
                              if ( intermediatePointsImageVector[j].at
                                < unsigned char > ( rows, cols ) != 0)
                              {
                                nonZero++;
                              }
                            }
                          }

                          // There should be more than 400 intermediate points
                          EXPECT_LT ( 400, nonZero );
                        }
                      }
                      else
                      {
                        // No masks if the corresponding filter to variable
                        // c is disabled
                        EXPECT_EQ ( 0, intermediatePointsImageVector.size() );
                      }


                      // Inquire about intermediatePointsSetVector
                      if ( b == 1 || d == 1 || g == 1 )
                      {
                        // There should be two masks of intermediate points
                        EXPECT_EQ ( 2, intermediatePointsSetVector.size() );

                        for ( int j = 0;
                          j < intermediatePointsSetVector.size(); j++ )
                        {
                          // There should be more than 400 intermediate points
                          EXPECT_LT ( 400,
                            intermediatePointsSetVector[j].size() );
                        }
                      }
                      else
                      {
                        // No masks if the corresponding filters to variables
                        // b, d and g are disabled
                        EXPECT_EQ ( 0, intermediatePointsSetVector.size() );
                      }


                      // Clear the vectors for the next run
                      holesMasksImageVector.clear();
                      holesMasksSetVector.clear();
                      inflatedRectanglesVector.clear();
                      inflatedRectanglesIndices.clear();
                      intermediatePointsImageVector.clear();
                      intermediatePointsSetVector.clear();

                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }



  //! Tests FiltersResources::createHolesMasksVectors
  TEST_F ( FiltersResourcesTest, CreateHolesMasksVectorsTest )
  {
    // The vector of images of masks
    std::vector< cv::Mat > holesMasksImageVector;

    // The indices of points inside the holes in conveyor
    std::vector< std::set< unsigned int > > holesMasksSetVector;


    // Run FiltersResources::createHolesMasksVectors
    FiltersResources::createHolesMasksVectors(
      conveyor,
      squares_,
      &holesMasksImageVector,
      &holesMasksSetVector );

    // There should be three masks in total
    EXPECT_EQ ( 3, holesMasksImageVector.size() );
    EXPECT_EQ ( 3, holesMasksSetVector.size() );

    // The number of non-zero value pixels in all of the images
    int nonZero = 0;
    for ( int i = 0; i < holesMasksImageVector.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if ( holesMasksImageVector[i].at<unsigned char >( rows, cols ) != 0 )
          {
            nonZero++;
          }
        }
      }
    }

    // The total number of non-zero value pixels in all of the images should
    // be equal to three times as much as in any image, which is 100 X 100 minus
    // the four corners of each square, due to the nature of the brushfire
    // exploring method
    EXPECT_EQ ( 3 * (10000 - 4), nonZero );

    for ( int h = 0; h < conveyor.size(); h++ )
    {
      // Each mask should have 100 X 100 points minus the square's four corners
      // due to the nature of the brushfire exploring method
      EXPECT_EQ ( 10000 - 4, holesMasksSetVector[h].size() );
    }

  }



  //! Tests FiltersResources::createHolesMasksImageVector
  TEST_F ( FiltersResourcesTest, CreateHolesMasksImageVectorTest )
  {
    // The vector of images of masks
    std::vector< cv::Mat > holesMasksImageVector;

    // Run FiltersResources::createHolesMasksImageVector
    FiltersResources::createHolesMasksImageVector(
      conveyor,
      squares_,
      &holesMasksImageVector);

    // There should be three images in total
    EXPECT_EQ ( 3, holesMasksImageVector.size() );

    // The images' type should be CV_8UC1
    for ( int i = 0; i < holesMasksImageVector.size(); i++ )
    {
      EXPECT_EQ ( CV_8UC1, holesMasksImageVector[i].type() );
    }

    // The number of non-zero value pixels in all of the images
    int nonZero = 0;
    for ( int i = 0; i < holesMasksImageVector.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if ( holesMasksImageVector[i].at<unsigned char >( rows, cols ) != 0 )
          {
            nonZero++;
          }
        }
      }
    }

    // The total number of non-zero value pixels in all of the images should
    // be equal to three times as much as in any image, which is 100 X 100 minus
    // the four corners of each square, due to the nature of the brushfire
    // exploring method
    EXPECT_EQ ( 3 * (10000 - 4), nonZero );
  }



  //! Tests FiltersResources::createHolesMasksSetVector
  TEST_F ( FiltersResourcesTest, CreateHolesMasksSetVectorTest )
  {
    // The indices of points inside the holes in conveyor
    std::vector< std::set< unsigned int > > holesMasksSetVector;

    // Run FiltersResources::createHolesMasksSetVector
    FiltersResources::createHolesMasksSetVector(
      conveyor,
      squares_,
      &holesMasksSetVector );


    for ( int h = 0; h < conveyor.size(); h++ )
    {
      // Each mask should have 100 X 100 points minus the square's four corners
      // due to the nature of the brushfire exploring method
      EXPECT_EQ ( 10000 - 4, holesMasksSetVector[h].size() );

      // Uncomment for visual inspection
      /*
       *
       *      cv::Mat img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
       *      unsigned char* ptr = img.ptr();
       *
       *      for ( std::set<unsigned int>::iterator it = holesMasksSetVector[h].begin();
       *        it != holesMasksSetVector[h].end(); it++ )
       *      {
       *        ptr[*it] = 255;
       *      }
       *
       *      Visualization::show ( "Mask", img, 0 );
       *
       */
    }
  }



  //! Tests FiltersResources::createInflatedRectanglesVector
  TEST_F ( FiltersResourcesTest, CreateInflatedRectanglesVectorTest )
  {
    // The vector holding the inflated rectangles vertices per hole for
    // inflation size equal to zero
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;

    // The indices of holes inside the conveyor whose inflated rectangles is
    // actually inside the image's bounds for
    // inflation size equal to zero
    std::vector<int> inflatedRectanglesIndices_0;

    // Run FiltersResources::createInflatedRectanglesVector
    // with inflation size of value 0
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // All holes' inflated rectangles should be inside the image's bounds
    EXPECT_EQ ( 3, inflatedRectanglesVector_0.size() );
    EXPECT_EQ ( 3, inflatedRectanglesIndices_0.size() );

    // All rectangles should be of size four (four vertices)
    for ( int h = 0; h < inflatedRectanglesVector_0.size(); h++ )
    {
      EXPECT_EQ ( 4, inflatedRectanglesVector_0[h].size() );
    }

    // The vector holding the inflated rectangles vertices per hole for
    // inflation size equal to two
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_2;

    // The indices of holes inside the conveyor whose inflated rectangles is
    // actually inside the image's bounds for
    // inflation size equal to two
    std::vector<int> inflatedRectanglesIndices_2;

    // Run FiltersResources::createInflatedRectanglesVector
    // with inflation size of value 2
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      2,
      &inflatedRectanglesVector_2,
      &inflatedRectanglesIndices_2 );

    // The lower right hole's inflated rectangle should exceed
    // the image's bounds
    EXPECT_EQ ( 2, inflatedRectanglesVector_2.size() );
    EXPECT_EQ ( 2, inflatedRectanglesIndices_2.size() );

    // All rectangles should be of size four (four vertices)
    for ( int h = 0; h < inflatedRectanglesVector_2.size(); h++ )
    {
      EXPECT_EQ ( 4, inflatedRectanglesVector_2[h].size() );
    }

    // The first valid inflated rectangle should be conveyor[1]
    EXPECT_EQ ( 1, inflatedRectanglesIndices_2[0] );

    // The second valid inflated rectangle should be conveyor[2]
    EXPECT_EQ ( 2, inflatedRectanglesIndices_2[1] );
  }



  //! Tests FiltersResources::createIntermediateHolesPointsVectors
  TEST_F ( FiltersResourcesTest, CreateIntermediateHolesPointsVectorsTest )
  {
    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    // First, test with an inflation size value of 0
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // The intermediate points vector for all holes
    std::vector< cv::Mat > intermediatePointsImageVector_0;

    // The intermediate points vector for all holes
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    // Run FiltersResources::createIntermediateHolesPointsVectors
    FiltersResources::createIntermediateHolesPointsVectors(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsImageVector_0,
      &intermediatePointsSetVector_0 );

    // Intermediate points positions should only exist for all of the holes
    ASSERT_EQ ( 3, intermediatePointsImageVector_0.size() );
    ASSERT_EQ ( 3, intermediatePointsSetVector_0.size() );

    // The total number of intermediate points in all of the images
    int nonZeroImage = 0;

    for ( int i = 0; i < intermediatePointsImageVector_0.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if (intermediatePointsImageVector_0[i].at<unsigned char>( rows, cols ) != 0)
          {
            nonZeroImage++;
          }
        }
      }
    }

    // There shouldn't be any intermediate points for inflation size equal to 0
    EXPECT_EQ ( 0, nonZeroImage );

    // There shouldn't be any intermediate points for inflation size equal to 0
    for ( int i = 0; i < intermediatePointsSetVector_0.size(); i++ )
    {
      EXPECT_EQ ( 0, intermediatePointsSetVector_0[i].size() );
    }


    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_2;
    std::vector< int > inflatedRectanglesIndices_2;

    // First, test with an inflation size value of 2
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      2,
      &inflatedRectanglesVector_2,
      &inflatedRectanglesIndices_2 );

    // The intermediate points vector for all holes
    std::vector< cv::Mat > intermediatePointsImageVector_2;

    // The intermediate points vector for all holes
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_2;

    // Run FiltersResources::createIntermediateHolesPointsVectors
    FiltersResources::createIntermediateHolesPointsVectors(
      conveyor,
      squares_,
      inflatedRectanglesVector_2,
      inflatedRectanglesIndices_2,
      &intermediatePointsImageVector_2,
      &intermediatePointsSetVector_2 );

    // Intermediate points positions should only exist for all of the holes
    // whose iflated rectangle is within the image's bounds
    ASSERT_EQ ( 2, intermediatePointsImageVector_2.size() );
    ASSERT_EQ ( 2, intermediatePointsSetVector_2.size() );

    // The total number of intermediate points in all of the images
    nonZeroImage = 0;

    for ( int i = 0; i < intermediatePointsImageVector_2.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if (intermediatePointsImageVector_2[i].at<unsigned char>( rows, cols ) != 0)
          {
            nonZeroImage++;
          }
        }
      }
    }

    // There shouldn't be any intermediate points for inflation size equal to 0
    EXPECT_LT ( 2 * 400, nonZeroImage );

    // There shouldn't be any intermediate points for inflation size equal to 0
    for ( int i = 0; i < intermediatePointsSetVector_2.size(); i++ )
    {
      EXPECT_LT ( 400, intermediatePointsSetVector_2[i].size() );
    }

  }



  //! Tests FiltersResources::createIntermediateHolesPointsImageVector
  TEST_F ( FiltersResourcesTest, CreateIntermediateHolesPointsImageVectorTest )
  {
    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    // First, test with an inflation size value of 0
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // The intermediate points vector for all holes
    std::vector< cv::Mat > intermediatePointsImageVector_0;

    // Run FiltersResources::createIntermediateHolesPointsImageVector
    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsImageVector_0 );

    // Intermediate points positions should exist for all of the holes
    ASSERT_EQ ( 3, intermediatePointsImageVector_0.size() );

    // The total number of intermediate points in all of the images
    int nonZero = 0;

    for ( int i = 0; i < intermediatePointsImageVector_0.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if (intermediatePointsImageVector_0[i].at<unsigned char>( rows, cols ) != 0)
          {
            nonZero++;
          }
        }
      }
    }

    // There shouldn't be any intermediate points for inflation size equal to 0
    EXPECT_EQ ( 0, nonZero );


    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_2;
    std::vector< int > inflatedRectanglesIndices_2;

    // First, test with an inflation size value of 2
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      2,
      &inflatedRectanglesVector_2,
      &inflatedRectanglesIndices_2 );

    // The intermediate points vector for all holes
    std::vector< cv::Mat > intermediatePointsImageVector_2;

    // Run FiltersResources::createIntermediateHolesPointsImageVector
    FiltersResources::createIntermediateHolesPointsImageVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_2,
      inflatedRectanglesIndices_2,
      &intermediatePointsImageVector_2 );

    // Intermediate points should only exist for the two holes
    ASSERT_EQ ( 2, intermediatePointsImageVector_2.size() );

    // The total number of intermediate points in all of the images
    nonZero = 0;

    for ( int i = 0; i < intermediatePointsImageVector_2.size(); i++ )
    {
      for ( int rows = 0; rows < squares_.rows; rows++ )
      {
        for ( int cols = 0; cols < squares_.cols; cols++ )
        {
          if (intermediatePointsImageVector_2[i].at<unsigned char>( rows, cols ) != 0)
          {
            nonZero++;
          }
        }
      }
    }

    // There should be two valid inflated rectangles, so the total number
    // of intermediate points should be greater than two times the outline
    // of each hole
    EXPECT_LT ( 2 * 400 , nonZero );

  }



  //! Tests FiltersResources::createIntermediateHolesPointsSetVector
  TEST_F ( FiltersResourcesTest, CreateIntermediateHolesPointsSetVectorTest )
  {
    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_0;
    std::vector< int > inflatedRectanglesIndices_0;

    // First, test with an inflation size value of 0
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      0,
      &inflatedRectanglesVector_0,
      &inflatedRectanglesIndices_0 );

    // The intermediate points vector for all holes
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_0;

    // Run FiltersResources::createIntermediateHolesPointsSetVector
    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_0,
      inflatedRectanglesIndices_0,
      &intermediatePointsSetVector_0 );

    // Intermediate points positions should exist for all of the holes
    ASSERT_EQ ( 3, intermediatePointsSetVector_0.size() );

    // There shouldn't be any intermediate points for inflation size equal to 0
    for ( int i = 0; i < intermediatePointsSetVector_0.size(); i++ )
    {
      EXPECT_EQ ( 0, intermediatePointsSetVector_0[i].size() );
    }

    // First off, we need to obtain the inflated rectangles vector and the
    // corresponding vector of indices of holes with valid inflated rectangles
    std::vector< std::vector< cv::Point2f > > inflatedRectanglesVector_2;
    std::vector< int > inflatedRectanglesIndices_2;

    // First, test with an inflation size value of 2
    FiltersResources::createInflatedRectanglesVector(
      conveyor,
      squares_,
      2,
      &inflatedRectanglesVector_2,
      &inflatedRectanglesIndices_2 );

    // The intermediate points vector for all holes
    std::vector< std::set< unsigned int > > intermediatePointsSetVector_2;

    // Run FiltersResources::createIntermediateHolesPointsSetVector
    FiltersResources::createIntermediateHolesPointsSetVector(
      conveyor,
      squares_,
      inflatedRectanglesVector_2,
      inflatedRectanglesIndices_2,
      &intermediatePointsSetVector_2 );

    // Intermediate points should only exist for the two holes
    ASSERT_EQ ( 2, intermediatePointsSetVector_2.size() );

    // There should be more than 4 X 100 intermediate points
    for ( int i = 0; i < intermediatePointsSetVector_2.size(); i++ )
    {
      EXPECT_LT ( 400, intermediatePointsSetVector_2[i].size() );
    }

    // Uncomment for visual inspection
    /*
     *    for ( int h = 0; h < intermediatePointsSetVector_2.size(); h++ )
     *    {
     *
     *      cv::Mat img = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC1 );
     *      unsigned char* ptr = img.ptr();
     *
     *      for ( std::set<unsigned int>::iterator it =
     *        intermediatePointsSetVector_2[h].begin();
     *        it != intermediatePointsSetVector_2[h].end(); it++ )
     *      {
     *        ptr[*it] = 255;
     *      }
     *
     *      Visualization::show ( "Intermediate Points", img, 0 );
     *    }
     *
     */
  }

} // namespace pandora_vision
