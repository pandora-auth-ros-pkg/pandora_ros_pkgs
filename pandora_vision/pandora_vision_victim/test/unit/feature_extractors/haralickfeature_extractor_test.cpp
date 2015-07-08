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

#include "pandora_vision_victim/feature_extractors/haralickfeature_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
    @class HaralickFeaturesExtractorTest
    @brief Tests the integrity of methods of class HaralickFeaturesExtractor
   **/
  class HaralickFeaturesExtractorTest : public ::testing::Test
  {
    protected:
      HaralickFeaturesExtractorTest() {}

      /// Sets up images needed for testing
      virtual void SetUp()
      {
        HEIGHT = 4;
        WIDTH = 4;

        // Construct a black image
        black = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);

        // set the blackGLCM matrix
        blackGLCM = cv::Mat::zeros(256, 256, CV_64FC1);
        int occurence = black.rows * (black.cols - 1);
        blackGLCM.at<double>(0, 0) = occurence;
        cv::Scalar totalSum = cv::sum(blackGLCM);
        blackGLCM.at<double>(0, 0) = occurence/totalSum[0];

        // std::cout << "blackGLCM= " << std::endl << " " << blackGLCM << std::endl << std::endl;

        // Construct a white image
        white = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
        cv::Rect rect(0, 0, WIDTH, HEIGHT);
        cv::rectangle(white, rect, cv::Scalar(255, 0, 0), -1);

        // std::cout << "white= " << std::endl << " " << white << std::endl << std::endl;

        // set the whiteGLCM matrix
        whiteGLCM = cv::Mat::zeros(256, 256, CV_64FC1);
        occurence = white.rows * (white.cols - 1);
        whiteGLCM.at<double>(255, 255) = occurence;
        totalSum = cv::sum(whiteGLCM);
        whiteGLCM.at<double>(255, 255) = occurence/totalSum[0];

        // std::cout << "whiteGLCM= " << std::endl << " " << whiteGLCM << std::endl << std::endl;

        // Construct a horizontal edge image
        horizontal = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
        cv::Rect rect1(0, 0, WIDTH, HEIGHT/2);
        cv::rectangle(horizontal, rect1, cv::Scalar(255, 0, 0), -1);

        // Construct a vertical edge image
        vertical = cv::Mat::zeros(HEIGHT, WIDTH, CV_8U);
        cv::Rect rect2(0, 0, WIDTH/2, HEIGHT);
        cv::rectangle(vertical, rect2, cv::Scalar(255, 0, 0), -1);
      }

      // Image Dimensions
      int HEIGHT, WIDTH;

      // Images that will be used for testing
      cv::Mat black, blackGLCM, white, whiteGLCM, horizontal, vertical, diagonal45, diagonal135, circle, noisy;
  };

  /// Tests HaralickFeaturesExtractor::calculateGLCM()
  TEST_F(HaralickFeaturesExtractorTest, calculateGLCMwhite)
  {
    // The output image
    cv::Mat out;
    out = HaralickFeaturesExtractor::calculateGLCM(0, 1, white);
    ASSERT_EQ(256, out.rows);
    ASSERT_EQ(256, out.cols);

    bool equal = true;
    for (int ii = 0; ii < out.rows; ii++)
      for (int jj = 0; jj< out.cols; jj++)
        if (out.at<double>(ii, jj) != whiteGLCM.at<double>(ii, jj))
          equal= false;

    // std::cout << "out= " << std::endl << " " << out << std::endl << std::endl;
    EXPECT_EQ(true, equal);
  }

  TEST_F(HaralickFeaturesExtractorTest, calculateGLCMblack)
  {
    // The output image
    cv::Mat out;
    out = HaralickFeaturesExtractor::calculateGLCM(0, 1, black);
    ASSERT_EQ(256, out.rows);
    ASSERT_EQ(256, out.cols);

    bool equal = true;
    for (int ii = 0; ii < out.rows; ii++)
      for (int jj = 0; jj < out.cols; jj++)
        if (out.at<double>(ii, jj) != blackGLCM.at<double>(ii, jj))
          equal= false;

    // std::cout << "out= " << std::endl << " " << out << std::endl << std::endl;
    EXPECT_EQ(true , equal);
  }

  TEST_F(HaralickFeaturesExtractorTest, AngularSecondMoment)
  {
    // The output feature
    double out;
    out = HaralickFeaturesExtractor::getAngularSecondMoment(blackGLCM);

    // std::cout << "out= " << std::endl << " " << out << std::endl << std::endl;
    // compute angular second moment
    double ang = 0;
    for (int ii = 0; ii < blackGLCM.rows; ii++)
      for (int jj = 0; jj < blackGLCM.cols; jj++)
        ang += pow(blackGLCM.at<double>(ii, jj), 2.0);

    EXPECT_EQ(ang, out);
  }

  TEST_F(HaralickFeaturesExtractorTest, Contrast)
  {
    // The output feature
    double out;
    out = HaralickFeaturesExtractor::getContrast(blackGLCM);

    // std::cout << "out= " << std::endl << " " << out << std::endl << std::endl;
    // compute angular second moment
    double sum = 0;
    for (int ii = 0; ii < blackGLCM.rows; ii++)
      for (int jj = 0; jj < blackGLCM.cols; jj++)
        sum += pow((ii - jj), 2.0) * blackGLCM.at<double>(ii, jj);

    EXPECT_EQ(sum, out);
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision
