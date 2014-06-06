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

#include "utils/noise_elimination.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class NoiseEliminationTest
    @brief Tests the integrity of methods of class NoiseElimination
   **/
  class NoiseEliminationTest : public ::testing::Test
  {
    protected:

      NoiseEliminationTest () {}

      /**
        @brief Constructs a filled rectangle of width @param x
        and height of @param y. All points inside the rectangle have a value of
        0.0
        @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
        rectangle to be created
        @param[in] x [const int&] The recgangle's width
        @param[in] y [const int&] The rectangle's height
        @param[out] image [cv::Mat*] The image on which the rectangle will be
        imprinted on. Its size must be set before calling this method
        return void
       **/
      void generateFilledRectangle (
        const cv::Point2f& upperLeft,
        const int& x,
        const int& y,
        cv::Mat* image );

      //! Sets up images needed for testing
      virtual void SetUp ()
      {
        WIDTH = 640;
        HEIGHT = 480;

        // Construct interpolationMethod0
        interpolationMethod0 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod0 with a value of 1.0
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod0.at< float >( rows, cols ) = 1.0;
          }
        }

        // Insert noise
        generateFilledRectangle ( cv::Point2f ( 0, 0 ),
          WIDTH - 200,
          HEIGHT - 200,
          &interpolationMethod0);

        generateFilledRectangle ( cv::Point2f ( WIDTH - 50, HEIGHT - 50 ),
          10,
          10,
          &interpolationMethod0);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod0", interpolationMethod0, 0);
         */

        // Construct interpolationMethod1
        interpolationMethod1 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod1 with a value of 0.5
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod1.at< float >( rows, cols ) = 0.5;
          }
        }

        // Insert noise
        generateFilledRectangle ( cv::Point2f ( 0, 0 ),
          WIDTH - 50,
          HEIGHT - 200,
          &interpolationMethod1);

        generateFilledRectangle ( cv::Point2f ( WIDTH - 50, HEIGHT - 50 ),
          10,
          10,
          &interpolationMethod1);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod1", interpolationMethod1, 0);
         */


        // Construct interpolationMethod2
        interpolationMethod2 = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

        // Fill interpolationMethod2 with a value of 1.0
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++)
          {
            interpolationMethod2.at< float >( rows, cols ) = 1.0;
          }
        }

        generateFilledRectangle ( cv::Point2f ( 10, 10 ),
          WIDTH - 20,
          HEIGHT - 20,
          &interpolationMethod2);

        // Uncomment for visual inspection
        /*
         *Visualization::showScaled
         *  ( "interpolationMethod2", interpolationMethod2, 0);
         */

      }

      int WIDTH;
      int HEIGHT;

      // Three images that will be used to test
      // methods of class NoiseElimination

      // An image with minimal noise (black pixels)
      cv::Mat interpolationMethod0;

      // An image with considerable amount of noise (black pixels)
      cv::Mat interpolationMethod1;

      // An image heavily noisy (black pixels)
      cv::Mat interpolationMethod2;
  };



  /**
    @brief Constructs a filled rectangle of width @param x
    and height of @param y. All points inside the rectangle have a value of 0.0
    @param[in] upperLeft [const cv::Point2f&] The upper left vertex of the
    rectangle to be created
    @param[in] x [const int&] The rectangle's width
    @param[in] y [const int&] The rectangle's height
    @param[out] image [cv::Mat*] The image on which the rectangle will be
    imprinted on. Its size must be set before calling this method
    return void
   **/
  void NoiseEliminationTest::generateFilledRectangle (
    const cv::Point2f& upperLeft,
    const int& x,
    const int& y,
    cv::Mat* image )
  {
    // Fill the inside of the desired rectangle with the @param value provided
    for( int rows = upperLeft.y; rows < upperLeft.y + y; rows++ )
    {
      for ( int cols = upperLeft.x; cols < upperLeft.x + x; cols++ )
      {
        image->at< float >( rows, cols ) = 0.0;
      }
    }
  }



  // Test NoiseElimination::brushfireNear
  TEST_F ( NoiseEliminationTest, BrushfireNearTest )
  {
    cv::Mat image;

    // Uncomment for visual inspection
    //Visualization::showScaled ( "before brushfireNear", interpolationMethod1, 0 );

    // Run NoiseElimination::brushfireNear on interpolationMethod1
    NoiseElimination::brushfireNear ( interpolationMethod1, &image );

    // Uncomment for visual inspection
    //Visualization::showScaled ( "after brushfireNear", image, 0 );

    float mean = 0.0;
    for ( int rows = 0; rows < image.rows; rows++ )
    {
      for ( int cols = 0; cols < image.cols; cols++ )
      {
        mean += image.at< float >( rows, cols );
      }
    }

    mean /= ( image.rows * image.cols );

    // The whole of the image should be at 0.5 value
    EXPECT_EQ ( 0.5, mean );


    // Run NoiseElimination::brushfireNear on interpolationMethod0
    NoiseElimination::brushfireNear ( interpolationMethod0, &image );

    mean = 0.0;
    for ( int rows = 0; rows < image.rows; rows++ )
    {
      for ( int cols = 0; cols < image.cols; cols++ )
      {
        mean += image.at< float >( rows, cols );
      }
    }

    mean /= ( image.rows * image.cols );

    // The whole of the image should be at a value of 1.0
    EXPECT_EQ ( 1.0, mean );
  }



  //! Test NoiseElimination::brushfireNearStep
  TEST_F ( NoiseEliminationTest, BrushfireNearStepTest )
  {
    // Uncomment for visual inspection
    /*
     *Visualization::showScaled
     *  ( "before brushfireNearStep", interpolationMethod1, 0);
     */

    // Run NoiseElimination::brushfireNearStep on image interpolationMethod1
    NoiseElimination::brushfireNearStep
      ( &interpolationMethod1, 200 * WIDTH + 100 );

    // Uncomment for visual inspection
    /*
     *Visualization::showScaled
     *  ( "after brushfireNearStep", interpolationMethod1, 0);
     */

    // All pixels of interpolationMethod1 should now have a value of 0.5
    float sum = 0.0;
    for ( int rows = 0; rows < interpolationMethod1.rows; rows++ )
    {
      for ( int cols = 0; cols < interpolationMethod1.cols; cols++ )
      {
        sum += interpolationMethod1.at< float >( rows, cols );
      }
    }

    // All non-zero value pixels have a value of 0.5
    ASSERT_EQ ( 0.5 * ( static_cast< float >( WIDTH * HEIGHT ) - 100 ), sum );


    // Test a blank image
    cv::Mat blank = cv::Mat::zeros ( HEIGHT, WIDTH, CV_32FC1 );

    // Run NoiseElimination::brushfireNearStep
    NoiseElimination::brushfireNearStep ( &blank, 10 * WIDTH + 10 );

    // All pixels should be still black
    ASSERT_EQ ( 0, cv::countNonZero ( blank ) );

    // Test an image with square concentrations of noise in each corner of it
    cv::Mat corners_ = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    unsigned int seed = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        corners_.at< float >( rows, cols ) =
          static_cast<float>(rand_r(&seed) % 100) / 1000;
      }
    }


    // upper left
    for ( int rows = 1; rows < 100; rows++ )
    {
      for ( int cols = 1; cols < 100; cols++ )
      {
        corners_.at< float >( rows, cols ) = 0.0;
      }
    }

    // lower left
    for ( int rows = HEIGHT - 1 - 100; rows < HEIGHT - 1; rows++ )
    {
      for ( int cols = 1; cols < 100; cols++ )
      {
        corners_.at< float >( rows, cols ) = 0.0;
      }
    }

    // upper right
    for ( int rows = 1; rows < 100; rows++ )
    {
      for ( int cols = WIDTH - 1 - 100; cols < WIDTH - 1; cols++ )
      {
        corners_.at< float >( rows, cols ) = 0.0;
      }
    }

    // lower right
    for ( int rows = HEIGHT - 1 - 100; rows < HEIGHT - 1; rows++ )
    {
      for ( int cols = WIDTH - 1 - 100; cols < WIDTH - 1; cols++ )
      {
        corners_.at< float >( rows, cols ) = 0.0;
      }
    }

    // The number of non zero pixels before calling any brushfireNearStep
    int nonZerosOne = cv::countNonZero( corners_ );

    // Run NoiseElimination::brushfireNearStep on image corners_ for the upper
    // left square concentration of noise
    NoiseElimination::brushfireNearStep
      ( &corners_, 1 * WIDTH + 1 );

    // The number of non zero pixels after removing the upper left noise
    // concentration
    int nonZerosTwo = cv::countNonZero( corners_ );

    EXPECT_LT ( nonZerosOne, nonZerosTwo );

    // Run NoiseElimination::brushfireNearStep on image corners_ for the upper
    // right square concentration of noise
    NoiseElimination::brushfireNearStep
      ( &corners_, 1 * WIDTH + WIDTH - 1 );

    // The number of non zero pixels after removing the upper right noise
    // concentration
    int nonZerosThree = cv::countNonZero( corners_ );

    EXPECT_LT ( nonZerosTwo, nonZerosThree );

    // Run NoiseElimination::brushfireNearStep on image corners_ for the lower
    // left square concentration of noise
    NoiseElimination::brushfireNearStep
      ( &corners_, ( HEIGHT - 1 ) * WIDTH + 1 );

    // The number of non zero pixels after removing the lower left noise
    // concentration
    int nonZerosFour = cv::countNonZero( corners_ );

    EXPECT_LT ( nonZerosThree, nonZerosFour );

    // Run NoiseElimination::brushfireNearStep on image corners_ for the lower
    // right square concentration of noise
    NoiseElimination::brushfireNearStep
      ( &corners_, ( HEIGHT - 1 ) * WIDTH + WIDTH - 1 );

    // The number of non zero pixels after removing the upper left noise
    // concentration
    int nonZerosFive = cv::countNonZero( corners_ );

    EXPECT_LT ( nonZerosFour, nonZerosFive );

  }


  //! Test NoiseElimination::chooseInterpolationMethod
  TEST_F ( NoiseEliminationTest, ChooseInterpolationMethodTest )
  {
    // On interpolationMethod0, Parameters::Depth::interpolation_method
    // should be equal to 0

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod0 );

    ASSERT_EQ ( 0, Parameters::Depth::interpolation_method );


    // On interpolationMethod1, Parameters::Depth::interpolation_method
    // should be equal to 1

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod1 );

    ASSERT_EQ ( 1, Parameters::Depth::interpolation_method );


    // On interpolationMethod2, Parameters::Depth::interpolation_method
    // should be equal to 2

    NoiseElimination::chooseInterpolationMethod ( interpolationMethod2 );

    ASSERT_EQ ( 2, Parameters::Depth::interpolation_method );

  }



  //! Test NoiseElimination::interpolateImageBorders
  TEST_F ( NoiseEliminationTest, InterpolateImageBordersTest )
  {
    // Create an image whose borders are non-zero but the rest of it is
    cv::Mat image = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    for ( int i = 1; i < image.cols - 1; ++i )
    {
      image.at< float >( 0, i ) = 1.0;
      image.at< float >( image.rows - 1, i ) = 1.0;
    }

    for ( int i = 1; i < image.rows - 1; ++i )
    {
      image.at< float >( i, 0 ) = 1.0;
      image.at< float >( i, image.cols - 1 ) = 1.0;
    }

    image.at< float >( 0, 0 ) = 1.0;

    image.at< float >( 0, image.cols - 1 ) = 1.0;

    image.at< float >( image.rows - 1, 0 ) = 1.0;

    image.at< float >( image.rows - 1, image.cols - 1 ) = 1.0;

    // Run NoiseElimination::interpolateImageBorders
    NoiseElimination::interpolateImageBorders( &image );

    // All border pixels should now have a value of zero
    EXPECT_EQ ( 0, cv::countNonZero( image ) );
  }



  //! Test NoiseElimination::interpolateZeroPixel
  TEST_F ( NoiseEliminationTest, InterpolateZeroPixelTest )
  {
    // The return value of NoiseElimination::interpolateZeroPixel
    int ret;

    bool endFlag = false;

    // Zero-out some pixels in image interpolationMethod0
    interpolationMethod0.at< float >( 200, 200 ) = 0.0;
    interpolationMethod0.at< float >( 1, 500 ) = 0.0;
    interpolationMethod0.at< float >( 400, 1 ) = 0.0;

    // Run NoiseElimination::interpolateZeroPixel
    ret = NoiseElimination::interpolateZeroPixel(
      interpolationMethod0, 200, 200, &endFlag);

    // At ( 200, 200 ) and around it, everything is black
    EXPECT_EQ ( 0.0, ret );
    EXPECT_EQ ( false, endFlag );

    // Run NoiseElimination::interpolateZeroPixel
    ret = NoiseElimination::interpolateZeroPixel(
      interpolationMethod0, 1, 500, &endFlag);

    // Around ( 1, 500 ), everything is at 1.0 value
    EXPECT_EQ ( 1.0, ret );
    EXPECT_EQ ( true, endFlag );

    // Run NoiseElimination::interpolateZeroPixel
    ret = NoiseElimination::interpolateZeroPixel(
      interpolationMethod0, 400, 1, &endFlag);

    // Around ( 400, 1 ), everything is at 1.0 value
    EXPECT_EQ ( 1.0, ret );
    EXPECT_EQ ( true, endFlag );

    // Run NoiseElimination::interpolateZeroPixel
    // Error should be received here
    ret = NoiseElimination::interpolateZeroPixel(
      interpolationMethod0, 0, 1, &endFlag);

    EXPECT_EQ ( 0.0, ret );
    EXPECT_EQ ( true, endFlag );

  }



  //! Test NoiseElimination::interpolation
  TEST_F ( NoiseEliminationTest, InterpolationTest )
  {
    cv::Mat interpolated = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1 );

    // Run NoiseElimination::interpolation
    NoiseElimination::interpolation( interpolationMethod0, &interpolated );

    int numZeros = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( interpolated.at< float >( rows, cols ) == 0.0 )
        {
          numZeros++;
        }
      }
    }

    EXPECT_EQ ( 0, numZeros );
  }



  //! Test NoiseElimination::interpolationIteration
  TEST_F ( NoiseEliminationTest, InterpolationIterationTest )
  {
    // The number of zero pixels before the call to interpolationIteration
    int nonZerosBefore = cv::countNonZero( interpolationMethod2 );

    // Run NoiseElimination::interpolationIteration
    NoiseElimination::interpolationIteration(&interpolationMethod2);

    // The number of zero pixels after the call to interpolationIteration
    int nonZerosAfter = cv::countNonZero( interpolationMethod2 );

    // There should be more black pixels before than after the call to
    // interpolationIteration
    EXPECT_GT ( nonZerosAfter, nonZerosBefore );
  }



  //! Test NoiseElimination::performeNoiseElimination
  TEST_F ( NoiseEliminationTest, PerformeNoiseEliminationTest )
  {
    // Remove the noise in interpolationMethod0
    Parameters::Depth::interpolation_method = 0;

    // The interpolated input image
    cv::Mat interpolated;

    // Run NoiseElimination::performeNoiseElimination
    NoiseElimination::performNoiseElimination(
      interpolationMethod0, &interpolated );

    // There shouldn't be any black pixels in interpolated
    EXPECT_EQ ( WIDTH * HEIGHT, cv::countNonZero( interpolated ) );

    // Remove the noise in interpolationMethod1
    Parameters::Depth::interpolation_method = 1;

    // Run NoiseElimination::performeNoiseElimination
    NoiseElimination::performNoiseElimination(
      interpolationMethod1, &interpolated );

    // There shouldn't be any black pixels in interpolated
    EXPECT_EQ ( WIDTH * HEIGHT, cv::countNonZero( interpolated ) );

    // Remove the noise in interpolationMethod2
    Parameters::Depth::interpolation_method = 2;

    // Run NoiseElimination::performeNoiseElimination
    NoiseElimination::performNoiseElimination(
      interpolationMethod2, &interpolated );

    // There shouldn't be any black pixels in interpolated
    EXPECT_EQ ( WIDTH * HEIGHT, cv::countNonZero( interpolated ) );
  }



  //! Test NoiseElimination::transformNoiseToWhite
  TEST_F ( NoiseEliminationTest, TransformNoiseToWhiteTest )
  {
    // Count how many noisy pixels there are on interpolationMethod0
    int numZero = 0;
    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( interpolationMethod0.at< float >( rows, cols ) == 0 )
        {
          numZero++;
        }
      }
    }

    cv::Mat out;

    // Run NoiseElimination::transformNoiseToWhite on interpolationMethod0
    NoiseElimination::transformNoiseToWhite ( interpolationMethod0, &out );

    // Count how many pixels have obtained the value dictated inside
    // NoiseElimination::transformNoiseToWhite (4.0)
    int changed = 0;

    for ( int rows = 0; rows < HEIGHT; rows++ )
    {
      for ( int cols = 0; cols < WIDTH; cols++ )
      {
        if ( out.at< float >( rows, cols ) == 4.0 )
        {
          changed++;
        }
      }
    }

    // The number of pixels changed should be equal to the intially noisy ones
    EXPECT_EQ ( numZero, changed );
  }

} // namespace pandora_vision
