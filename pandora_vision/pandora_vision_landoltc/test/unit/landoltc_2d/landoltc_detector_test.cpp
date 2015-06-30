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
 * Author: Angelos Triantafyllidis
 *********************************************************************/

#include <string>

#include <gtest/gtest.h>

#include "pandora_vision_landoltc/landoltc_2d/landoltc_detector.h"


namespace pandora_vision
{
  /**
    @class LandoltcDetectorTest
    @brief Tests the integrity of methods of class LandoltcDetector
   **/
  class LandoltcDetectorTest : public ::testing::Test
  {
    public:
      LandoltcDetectorTest()
      {
        packagePath = ros::package::getPath("pandora_vision_landoltc");
      }
      virtual ~LandoltcDetectorTest()
      {
      }
      void fillGrad(const cv::Mat& input);
      void getCenter(cv::Point *point, const int& index);
      int giveVotingData(cv::Point a, cv::Point b, int y, int i);

    protected:
      LandoltCDetector landoltCDetector;
      std::string packagePath;
  };

  // process input and call findcenters
  void LandoltcDetectorTest::fillGrad(const cv::Mat& input)
  {
    cv::Mat gradX, gradY;
    cv::Mat gray;

    if (input.channels() > 1)
      cv::cvtColor(input, gray, CV_BGR2GRAY);
    else
      gray = input;

    cv::Sobel(gray, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(gray, gradY, CV_32F, 0, 1, 3);

    float* gradXF = reinterpret_cast<float*>(gradX.data);
    float* gradYF = reinterpret_cast<float*>(gradY.data);

    landoltCDetector._voting = cv::Mat::zeros(input.rows, input.cols, CV_16U);
    landoltCDetector.findCenters(input.rows, input.cols, gradXF, gradYF);
  }

  // returns possible center
  void LandoltcDetectorTest::getCenter(cv::Point *point, const int& index)
  {
    int vectorSize = landoltCDetector._centers.size();
    ASSERT_NE(0, static_cast<int>(vectorSize));
    ASSERT_GT(vectorSize, index);
    *point = landoltCDetector._centers[index];
    return;
  }

  // y=column,i=row
  int LandoltcDetectorTest::giveVotingData(cv::Point a, cv::Point b , int y ,
      int i)
  {
    cv::Mat image;

    image = cv::imread(this->packagePath + "/bold.jpg");

    landoltCDetector._voting = cv::Mat::zeros(image.rows, image.cols, CV_16U);
    landoltCDetector.rasterizeLine(a, b);

    const uint16_t* readVoting = reinterpret_cast<const uint16_t*>(
        landoltCDetector._voting.data);
    int columns = image.cols;

    return readVoting[columns * y + i];
  }

  /** test cases **/

  TEST_F(LandoltcDetectorTest, findCentersTest)
  {
    cv::Mat image;
    image = cv::imread(packagePath + "/bold.jpg");
    if (!image.data)
    {
      ROS_FATAL("Could not read test image!");
      ASSERT_EQ(1, 0);
    }

    fillGrad(image);

    cv::Point point(400, 300);
    cv::Point detectedCenters;
    getCenter(&detectedCenters, 0);
    EXPECT_NEAR(point.x, detectedCenters.x, 0.1);
    EXPECT_NEAR(point.y, detectedCenters.y, 0.1);

    image = cv::imread(packagePath + "/index.png", 0);

    fillGrad(image);

    cv::Point point2(400, 300);
    getCenter(&detectedCenters, 0);
    EXPECT_NEAR(point2.x, detectedCenters.x, 0.1);
    EXPECT_NEAR(point2.y, detectedCenters.y, 0.1);
  }

  // TEST_F(LandoltcDetectorTest, rasterizeLineTest)
  // {
    // cv::Point a(1,1);
    // cv::Point b(2,2);
    // cv::Point c(4,5);
    // cv::Point d(6,3);
    // cv::Point e(2,4);

    // EXPECT_EQ(1,giveVotingData(a, b, 1, 1));
    // EXPECT_EQ(1,giveVotingData(b, c, 2, 2));
    // EXPECT_EQ(1,giveVotingData(b, c, 3, 3));
    // EXPECT_EQ(1,giveVotingData(b, c, 4, 3));
    // EXPECT_EQ(1,giveVotingData(d, c, 5, 4));
    // EXPECT_EQ(1,giveVotingData(d, c, 4, 5));
    // EXPECT_EQ(1,giveVotingData(b, e, 2, 2));
    // EXPECT_EQ(1,giveVotingData(b, e, 3, 2));
  // }

}  // namespace pandora_vision
