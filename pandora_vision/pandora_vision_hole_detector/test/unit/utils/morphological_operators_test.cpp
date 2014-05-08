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
 * Authors: Alexandros Philotheou, Christos Tsirigotis
 *********************************************************************/

#include "utils/morphological_operators.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  class MorphologyTest : public ::testing::Test
  {
    protected:

      MorphologyTest() {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        pixel_ = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        line_ = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        square_ = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );
        frame_ = cv::Mat::zeros ( HEIGHT, WIDTH, CV_8UC1 );

        ASSERT_EQ( HEIGHT, line_.rows );
        ASSERT_EQ( WIDTH, line_.cols );

        // pixel_ holds a single non-zero pixel
        pixel_.at<unsigned char>( 100, 100 ) = 255;

        // line_ holds a horizontal white line at row 100,
        // with width of 1 pixel
        for ( int cols = 0; cols < line_.cols; ++cols )
        {
          line_.at<unsigned char>( 100, cols ) = 255;
        }

        // square_ is a 100 X 100 opaque square,
        // with its upper left corner placed at the upper left corner of the
        // image
        for ( int rows = 0; rows < 100; rows++ )
        {
          for ( int cols = 0; cols < 100; cols++ )
          {
            square_.at<unsigned char>( rows, cols ) = 255;
          }
        }

        // frame_ is a white image with only its borders in black colour
        for ( int rows = 1; rows < frame_.rows - 1; rows++ )
        {
          for ( int cols = 1; cols < frame_.cols - 1; cols++ )
          {
            frame_.at<unsigned char>( rows, cols ) = 255;
          }
        }
      }

      // A frame's width and height
      int WIDTH;
      int HEIGHT;

      cv::Mat line_;
      cv::Mat pixel_;
      cv::Mat square_;
      cv::Mat frame_;

  };



  //! Test Morphology::closing()
  TEST_F ( MorphologyTest, ClosingTest )
  {
    /***************************************************************************
     * Test pixel_
     **************************************************************************/

    // Keep the original image in the originalPixel_ image
    cv::Mat originalPixel_;
    pixel_.copyTo(originalPixel_);

    // Apply the closing operator
    Morphology::closing( &pixel_, 1 );

    // diff_pixel is the difference between the original and the
    // closed pixel_ images
    cv::Mat diff_pixel = originalPixel_ - pixel_;

    // The diff image should be filled with zero value pixels only
    for ( int rows = 0; rows < pixel_.rows; rows++ )
    {
      for ( int cols = 0; cols < pixel_.cols; cols++ )
      {
        EXPECT_EQ( 0, diff_pixel.at<unsigned char>( rows, cols ));
      }
    }

    /***************************************************************************
     * Test square_
     **************************************************************************/

    // Keep the original image in the originalsquare_ image
    cv::Mat originalsquare_;
    square_.copyTo(originalsquare_);

    // Apply the closing operator
    Morphology::closing( &square_, 1 );

    // diff_square is the difference between the original and the
    // closed square_ images
    cv::Mat diff_square = originalsquare_ - square_;

    // The diff image should be filled with zero value pixels only
    for ( int rows = 0; rows < square_.rows; rows++ )
    {
      for ( int cols = 0; cols < square_.cols; cols++ )
      {
        EXPECT_EQ( 0, diff_square.at<unsigned char>( rows, cols ));
      }
    }
  }



  //! Test Morphology::dilation()
  TEST_F ( MorphologyTest, DilationTest )
  {
    /***************************************************************************
     * Test pixel_
     **************************************************************************/

    // Perform dilation on the pixel_
    Morphology::dilation( &pixel_, 1 );

    // All pixels surrounding immediately the only non-zero one before dilation
    // should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, pixel_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( line_ );

    // Dilate once
    Morphology::dilation( &line_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < line_.cols; ++cols )
    {
      EXPECT_EQ( 255, line_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, line_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
     * Test square_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    nonZerosBefore = cv::countNonZero( square_ );

    // Dilate once
    Morphology::dilation( &square_, 1 );

    // The number of non-zero pixels after dilation
    nonZerosAfter = cv::countNonZero( square_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the dilation should be
    // increased by twice the square_'s size, plus one non-zero pixel
    EXPECT_EQ( nonZerosBefore + 200 + 1, nonZerosAfter );
  }



  //! Test Morphology::dilationRelative()
  TEST_F ( MorphologyTest, DilationRelativeTest )
  {
    /***************************************************************************
     * Test pixel_
     **************************************************************************/

    // Perform dilation on the pixel_
    Morphology::dilationRelative( &pixel_, 1 );

    // All pixels surrounding immediately the only non-zero one before dilation
    // should now have a non-zero value
    for ( int i = -1; i < 2; ++i )
    {
      for ( int j = -1; j < 2; ++j )
      {
        EXPECT_EQ( 255, pixel_.at<unsigned char>( 100 + i, 100 + j ));
      }
    }


    /***************************************************************************
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    int nonZerosBefore = cv::countNonZero( line_ );

    // Dilate once
    Morphology::dilationRelative( &line_, 1 );

    // The number of non-zero pixels after dilation
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels before the dilation should be three times
    // as many as that of the pixels after
    EXPECT_EQ( 3 * nonZerosBefore, nonZerosAfter );

    // One row higher and one row lower than 100, all pixels should now
    // have a non-zero value
    for( int cols = 0; cols < line_.cols; ++cols )
    {
      EXPECT_EQ( 255, line_.at<unsigned char>( 99, cols ));
      EXPECT_EQ( 255, line_.at<unsigned char>( 101, cols ));
    }


    /***************************************************************************
     * Test square_
     **************************************************************************/

    // The number of non-zero pixels before dilation
    nonZerosBefore = cv::countNonZero( square_ );

    // Dilate once
    Morphology::dilationRelative( &square_, 1 );

    // The number of non-zero pixels after dilation
    nonZerosAfter = cv::countNonZero( square_ );

    // The number of non-zero pixels before the dilation should be less than
    // that of the pixels after
    EXPECT_LT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the dilation should be
    // increased by twice the square_'s size, plus one non-zero pixel
    EXPECT_EQ( nonZerosBefore + 200 + 1, nonZerosAfter );
  }



  //! Test Morphology::erosion()
  TEST_F ( MorphologyTest, ErosionTest )
  {
    /***************************************************************************
     * Test line_
     **************************************************************************/

    // The number of non-zero pixels before erosion
    int nonZerosBefore = cv::countNonZero( line_ );

    // Erode once
    Morphology::erosion( &line_, 1 );

    // The number of non-zero pixels after erosion
    int nonZerosAfter = cv::countNonZero( line_ );

    // The number of non-zero pixels before the erosion should be
    // less than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be exactly zero
    EXPECT_EQ( 0, nonZerosAfter );


    /***************************************************************************
     * Test frame_
     **************************************************************************/

    // The number of non-zero pixels before erosion
    nonZerosBefore = cv::countNonZero( frame_ );

    // Erode once
    Morphology::erosion( &frame_, 1 );

    // The number of non-zero pixels after erosion
    nonZerosAfter = cv::countNonZero( frame_ );

    // The number of non-zero pixels before the erosion should be
    // greater than that of the pixels after
    EXPECT_GT( nonZerosBefore, nonZerosAfter );

    // The number of non-zero pixels after the erosion should be
    // reduced by the size of frame_, minus its four corners
    EXPECT_EQ( nonZerosBefore - 2 * ( WIDTH - 2 ) - 2 * ( HEIGHT - 2 ) + 4,
      nonZerosAfter );
  }



  //! Test Morphology::kernelCheck()
  TEST_F ( MorphologyTest, KernelCheckTest )
  {
    /***************************************************************************
     * Test pixel_
     **************************************************************************/

    // pixel_ has all of its neighbors with a zero value
    char pixelKernel[3][3] = {
      { 0, 0, 0 },
      { 0, 1, 0 },
      { 0, 0, 0 }
    };

    // This should be true: Point( 100, 100 ) has a non-zero value,
    // while it is surrounded by zero value pixels
    bool retTruePixel =
      Morphology::kernelCheck( pixelKernel, pixel_, cv::Point( 100, 100 ) );

    EXPECT_EQ ( true, retTruePixel );

    // This should be false: Point ( 200, 200 ) has a zero value and
    // it is surrounded by zero value pixels
    bool retFalsePixel =
      Morphology::kernelCheck( pixelKernel, pixel_, cv::Point( 200, 200 ) );

    EXPECT_EQ ( false, retFalsePixel );


    /***************************************************************************
     * Test line_
     **************************************************************************/

    // line_ has all of its upwards and downwards neighbors with a zero value
    char lineKernel[3][3] = {
      { 0, 0, 0 },
      { 1, 1, 1 },
      { 0, 0, 0 }
    };

    for ( int cols = 1; cols < line_.cols - 1; cols++ )
    {
      // This should be true: all points in row 100 have a non-zero value,
      // while they are surrounded by zero value pixels, upwards and downwards
      bool retTrueLine =
        Morphology::kernelCheck( lineKernel, line_, cv::Point( cols, 100 ) );

      EXPECT_EQ ( true, retTrueLine );

      // This should be false: Point ( cols, 200 ) has a zero value and
      // it is surrounded by zero value pixels
      bool retFalseLine =
        Morphology::kernelCheck( lineKernel, line_, cv::Point( cols, 200 ) );

      EXPECT_EQ ( false, retFalseLine );
    }
  }



  //! Test Morphology::opening()
  TEST_F ( MorphologyTest, OpeningTest )
  {
    /***************************************************************************
     * Test pixel_
     **************************************************************************/

    // Keep the original image in the originalPixel_ image
    cv::Mat originalPixel_;
    pixel_.copyTo(originalPixel_);

    // Apply the closing operator
    Morphology::opening( &pixel_, 1 );

    // The diff image should be filled with zero value pixels only
    for ( int rows = 0; rows < pixel_.rows; rows++ )
    {
      for ( int cols = 0; cols < pixel_.cols; cols++ )
      {
        EXPECT_EQ( 0, pixel_.at<unsigned char>( rows, cols ));
      }
    }


    /***************************************************************************
     * Test line_
     **************************************************************************/

    // Keep the original image in the originalPixel_ image
    cv::Mat originalLine_;
    line_.copyTo(originalLine_);

    // Apply the closing operator
    Morphology::opening( &line_, 1 );

    // The diff image should be filled with zero value pixels only
    for ( int rows = 0; rows < line_.rows; rows++ )
    {
      for ( int cols = 0; cols < line_.cols; cols++ )
      {
        EXPECT_EQ( 0, line_.at<unsigned char>( rows, cols ));
      }
    }


    /***************************************************************************
     * Test square_
     **************************************************************************/

    // Keep the original image in the originalsquare_ image
    cv::Mat originalsquare_;
    square_.copyTo(originalsquare_);

    // Apply the closing operator
    Morphology::opening( &square_, 1 );

    // diff_square is the difference between the original and the
    // closed square_ images
    cv::Mat diff_square = originalsquare_ - square_;

    // The diff image should be filled with zero value pixels only
    for ( int rows = 0; rows < square_.rows; rows++ )
    {
      for ( int cols = 0; cols < square_.cols; cols++ )
      {
        EXPECT_EQ( 0, diff_square.at<unsigned char>( rows, cols ));
      }
    }
  }

}  // namespace pandora_vision
