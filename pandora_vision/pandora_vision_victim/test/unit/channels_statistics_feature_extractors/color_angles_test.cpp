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
* Author: Marios Protopapas
*********************************************************************/

#include <vector>

#include "gtest/gtest.h"

#include "pandora_vision_victim/channels_statistics_feature_extractors/color_angles_extractor.h"

namespace pandora_vision
{
  /**
    @class ColorAnglesExtractorTest
    @brief Tests the integrity of methods of class ColorAnglesExtractor
   **/
  class ColorAnglesExtractorTest : public ::testing::Test
  {
    protected:
      ColorAnglesExtractorTest() {}

      /// Sets up images needed for testing
      virtual void SetUp()
      {
        HEIGHT = 480;
        WIDTH = 640;

        // Construct a blue image
        blue = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));
        // Construct a green image
        green = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 255, 0));
        // Construct a red image
        red = cv::Mat(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(0, 0, 255));
      }

      cv::Mat blue, green, red;
      int HEIGHT, WIDTH;
  };

  /// Tests ColorAnglesExtractor::extract
  TEST_F(ColorAnglesExtractorTest, extractColorAngles)
  {
    // The output vector
    std::vector<double> out;
    ColorAnglesExtractor c1(&blue);  // , c2(&green), c3(&red);
    out = c1.extract();
    EXPECT_EQ(0, out[0]);
    EXPECT_EQ(0, out[1]);
    EXPECT_EQ(0, out[2]);
    EXPECT_EQ(0, out[3]);

    // out = c2.extract();
    // EXPECT_EQ(255, out[0]);
    // EXPECT_EQ(0, out[1]);

    // out = c3.extract();
    // EXPECT_EQ(127.5, out[0]);
    // EXPECT_EQ(127.5, out[1]);
  }
}  // namespace pandora_vision

