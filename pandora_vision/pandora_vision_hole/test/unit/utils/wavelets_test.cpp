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

#include "rgb_node/utils/wavelets.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace rgb
{
  /**
    @class WaveletsTest
    @brief Tests the integrity of methods of class Wavelets
   **/
  class WaveletsTest : public ::testing::Test
  {
    protected:

      WaveletsTest () {}

      virtual void SetUp()
      {
        HEIGHT = 480;
        WIDTH = 640;

        // Construct the grayscale image
        // It features a filled white square of length 100 at (100, 100)
        grayscale = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1);

        for ( int rows = 100; rows < 200; rows++ )
        {
          for ( int cols = 100; cols < 200; cols++ )
          {
            grayscale.at< float >( rows, cols ) = 2.0;
          }
        }

        // Construct the RGB image
        // It features a filled blue square of length 100 at (100, 100)
        rgb = cv::Mat::zeros( HEIGHT, WIDTH, CV_8UC3);

        for ( int rows = 100; rows < 200; rows++ )
        {
          for ( int cols = 100; cols < 200; cols++ )
          {
            rgb.at< cv::Vec3b >( rows, cols ).val[0] = 255;
          }
        }
      }

      // The images' dimensions
      int HEIGHT;
      int WIDTH;

      // A grayscale image that will be used for testing
      cv::Mat grayscale;

      // A RGB image that will be used for testing
      cv::Mat rgb;
  };


  //! Tests Wavelets::getLowLow
  TEST_F ( WaveletsTest, getLowLowDepthTest )
  {
    // Find the min and max values in image grayscale
    double min;
    double max;
    cv::minMaxIdx(grayscale, &min, &max);

    // The output image
    cv::Mat out;

    // Run Wavelets::getLowLow
    Wavelets::getLowLow(grayscale, min, max, &out);

    // The dimensions of image out should be half of those of image grayscale,
    // plus one apparently
    ASSERT_EQ ( HEIGHT / 2 + 1, out.rows );
    ASSERT_EQ ( WIDTH / 2 + 1, out.cols );

    // Count the number of non-zero value pixels inside grayscale and out.
    // The latter should be one quarter of the former. Because one more
    // column and row are added to the halved dimensions, it is expected that
    // the above number should be increased by a number of 50 + 50 + 1:
    // one halved non-zero column, one halved non-zero row plus one corner pixel
    int nonZeroGrayscale = cv::countNonZero( grayscale );
    int nonZeroOut = cv::countNonZero( out );

    EXPECT_EQ ( nonZeroGrayscale / 4 + 50 + 50 + 1, nonZeroOut );
  }



  //! Tests Wavelets::getLowLow
  TEST_F ( WaveletsTest, getLowLowRgbTest )
  {
    // The output image
    cv::Mat out;

    // Run Wavelets::getLowLow
    Wavelets::getLowLow(rgb, &out);

    // The dimensions of image out should be half of those of image grayscale,
    // plus one apparently
    ASSERT_EQ ( HEIGHT / 2 + 1, out.rows );
    ASSERT_EQ ( WIDTH / 2 + 1, out.cols );


    // Count the number of non-zero value pixels inside rgb and out.
    // The latter should be one quarter of the former. Because one more
    // column and row are added to the halved dimensions, it is expected that
    // the above number should be increased by a number of 50 + 50 + 1:
    // one halved non-zero column, one halved non-zero row plus one corner pixel.
    // In order to count the non-zero pixels, image rgb should be split into its
    // components

    std::vector< cv::Mat > rgbComponents;
    cv::split( rgb, rgbComponents );

    int nonZeroRgb = 0;
    for ( int ch = 0; ch < 3; ch++ )
    {
      nonZeroRgb += cv::countNonZero( rgbComponents[ch] );
    }

    std::vector< cv::Mat > outComponents;
    cv::split( out, outComponents );

    int nonZeroOut = 0;
    for ( int ch = 0; ch < 3; ch++ )
    {
      nonZeroOut += cv::countNonZero( outComponents[ch] );
    }

    EXPECT_EQ ( nonZeroRgb / 4 + 50 + 50 + 1, nonZeroOut );
  }
}  // namespace rgb
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
