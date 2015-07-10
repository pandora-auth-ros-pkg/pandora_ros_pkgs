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

#include "pandora_vision_victim/channels_statistics_feature_extractors/dominant_color_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
    @class DominantColorExtractorTest
    @brief Tests the integrity of methods of class DominantColorExtractor
   **/
  class DominantColorExtractorTest : public ::testing::Test
  {
    protected:
      DominantColorExtractorTest() {}

      /// Sets up images needed for testing
      virtual void SetUp()
      {
        bins = 256;
        HEIGHT = 480;
        WIDTH = 640;

        // Construct a black histogram
        black = cv::Mat::zeros(bins, 1, CV_32FC1);
        // Construct a white histogram
        white = cv::Mat::zeros(bins, 1, CV_32FC1);
        cv::Rect rect(0, 0, bins, 1);
        cv::rectangle(white, rect, cv::Scalar(255, 0, 0), -1);

        // Construct an ascending histogram
        ascending = cv::Mat::zeros(bins, 1, CV_32FC1);
        for (int rows = 0; rows < ascending.rows; rows++)
          ascending.at<float>(rows) = rows;

        // Construct a descending histogram
        descending = cv::Mat::zeros(bins, 1, CV_32FC1);
        int i = 255;
        for (int rows = 0; rows < descending.rows; rows++)
        {
          descending.at<float>(rows) = i;
          i--;
        }
      }

      /// The histograms bins
      int bins;

      /// Image Dimensions
      int HEIGHT, WIDTH;
      /// Images that will be used for testing
      cv::Mat black, white, ascending, descending;
  };

  /// Tests DominantColorExtractor::extract
  TEST_F(DominantColorExtractorTest, extractDominantColor)
  {
    // The output vector
    std::vector<double> out;
    DominantColorExtractor d1(&black), d2(&white), d3(&ascending), d4(&descending);
    out = d1.extract();
    EXPECT_EQ(0, out[0]);
    EXPECT_EQ(0, out[1]);

    out = d2.extract();
    EXPECT_EQ(0, out[0]);
    EXPECT_EQ(255.0 / cv::sum(white).val[0], out[1]);

    out = d3.extract();
    EXPECT_EQ(255, out[0]);
    EXPECT_EQ(255.0 / cv::sum(ascending).val[0], out[1]);

    out = d4.extract();
    EXPECT_EQ(0 , out[0]);
    EXPECT_EQ(255.0 / cv::sum(descending).val[0], out[1]);
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision
