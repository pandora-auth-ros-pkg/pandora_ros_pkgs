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
 * Authors:
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#include <vector>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/soft_obstacle_detection/soft_obstacle_detector.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class SoftObstacleDetectorTest : public ::testing::Test
  {
    public:
      SoftObstacleDetectorTest() : detector_(new SoftObstacleDetector)
      {
        detector_->sValueThreshold_ = 100;
        detector_->vValueThreshold_ = 130;

        detector_->minLineLength_ = 100;
        detector_->gaussianKernelSize_ = 13;
        detector_->gradientThreshold_ = 2.0;
        detector_->betaThreshold_ = 3.0;

        detector_->minDepthThreshold_ = 0.3;
        detector_->maxDepthThreshold_ = 0.5;

        detector_->erodeKernelSize_ = cv::Size(3, 3);
        detector_->dilateKernelSize_ = cv::Size(3, 3);

        detector_->centerWidth_ = 5;
        detector_->centerHeight_ = 10;

        detector_->showOriginalImage_ = false;
        detector_->showDWTImage_ = false;
        detector_->showOtsuImage_ = false;
        detector_->showDilatedImage_ = false;
        detector_->showVerticalLines_ = false;
        detector_->showROI_ = false;

        float invRootTwo = 1.0f / static_cast<float>(std::sqrt(2));
        cv::Mat kernelLow = (cv::Mat_<float>(2, 1) << invRootTwo, invRootTwo);
        cv::Mat kernelHigh = (cv::Mat_<float>(2, 1) << invRootTwo, - invRootTwo);

        detector_->dwtPtr_.reset(new pandora_vision_common::DiscreteWaveletTransform(
              kernelLow, kernelHigh));
      }

      bool findNonIdenticalLines(const std::vector<cv::Vec2f> lineCoeffs,
          float grad, float beta)
      {
        return detector_->findNonIdenticalLines(lineCoeffs, grad, beta);
      }

      bool detectLineIntersection(const std::vector<cv::Vec4i>& verticalLines,
          const cv::Vec4i& line)
      {
        return detector_->detectLineIntersection(verticalLines, line);
      }

      bool pickLineColor(const cv::Mat& hsvImage, const cv::Vec4i& line,
          int level = 1)
      {
        return detector_->pickLineColor(hsvImage, line, level);
      }

      std::vector<cv::Vec4i> performProbHoughLines(const cv::Mat& rgbImage,
          const cv::Mat& binaryImage, int level = 1)
      {
        return detector_->performProbHoughLines(rgbImage, binaryImage, level);
      }

      float detectROI(const std::vector<cv::Vec4i>& verticalLines,
            int frameHeight, const boost::shared_ptr<cv::Rect>& roiPtr)
      {
        return detector_->detectROI(verticalLines, frameHeight, roiPtr);
      }

      float calculateLineMedian(const cv::Mat& depthImage, const cv::Vec4i& line,
          int level = 1)
      {
        return detector_->calculateLineMedian(depthImage, line, level);
      }

      bool findDifferentROIDepth(const cv::Mat& depthImage,
        const std::vector<cv::Vec4i>& verticalLines, const cv::Rect& roi,
        int level = 1)
      {
        return detector_->findDifferentROIDepth(depthImage, verticalLines, roi, level);
      }

    protected:
      boost::shared_ptr<SoftObstacleDetector> detector_;
  };

  TEST_F(SoftObstacleDetectorTest, areIdenticalLinesDetected)
  {
    std::vector<cv::Vec2f> lineCoeffs;
    lineCoeffs.push_back(cv::Vec2f(89, 20));
    lineCoeffs.push_back(cv::Vec2f(85, 30));

    ASSERT_FALSE(findNonIdenticalLines(lineCoeffs, 90, 22));
    ASSERT_TRUE(findNonIdenticalLines(lineCoeffs, 85, 35));
    ASSERT_TRUE(findNonIdenticalLines(lineCoeffs, 95, 28));
  }

  TEST_F(SoftObstacleDetectorTest, doLinesIntersect)
  {
    std::vector<cv::Vec4i> lines;

    lines.push_back(cv::Vec4i(1, 1, 1, 10));
    lines.push_back(cv::Vec4i(5, 1, 5, 15));
    lines.push_back(cv::Vec4i(12, 20, 10, 5));
    lines.push_back(cv::Vec4i(15, 18, 16, 2));

    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(0, 1, 2, 10)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(2, 1, 2, 10)));

    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(5, 2, 6, 15)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(8, 1, 15, 10)));

    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(11, 5, 11, 20)));
    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(12, 5, 10, 20)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(10, 10, 11, 20)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(12, 15, 15, 5)));

    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(15, 2, 15, 18)));
    ASSERT_TRUE(detectLineIntersection(lines, cv::Vec4i(16, 18, 16, 2)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(15, 15, 14, 1)));
    ASSERT_FALSE(detectLineIntersection(lines, cv::Vec4i(16, 5, 18, 20)));
  }

  TEST_F(SoftObstacleDetectorTest, isLineWhite)
  {
    cv::Mat rgbImage(40, 40, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::line(rgbImage, cv::Point(5, 5), cv::Point(6, 30),
        cv::Scalar(255, 255, 255), 2, 8);
    cv::line(rgbImage, cv::Point(10, 5), cv::Point(12, 35),
        cv::Scalar(0, 255, 255), 2, 8);
    cv::line(rgbImage, cv::Point(15, 10), cv::Point(16, 35),
        cv::Scalar(255, 0, 255), 2, 8);
    cv::line(rgbImage, cv::Point(20, 5), cv::Point(22, 25),
        cv::Scalar(0, 0, 255), 2, 8);
    cv::line(rgbImage, cv::Point(30, 5), cv::Point(32, 35),
        cv::Scalar(255, 255, 255), 2, 8);

    cv::Mat hsvImage;
    cv::cvtColor(rgbImage, hsvImage, CV_BGR2HSV);

    ASSERT_TRUE(pickLineColor(hsvImage, cv::Vec4i(2, 2, 3, 15)));
    ASSERT_FALSE(pickLineColor(hsvImage, cv::Vec4i(5, 2, 6, 17)));
    ASSERT_FALSE(pickLineColor(hsvImage, cv::Vec4i(7, 5, 8, 17)));
    ASSERT_FALSE(pickLineColor(hsvImage, cv::Vec4i(10, 2, 11, 12)));
    ASSERT_TRUE(pickLineColor(hsvImage, cv::Vec4i(15, 2, 16, 17)));
  }

  TEST_F(SoftObstacleDetectorTest, areVerticalLinesDetectedCorrectly)
  {
    cv::Mat rgbImage(240, 240, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::line(rgbImage, cv::Point(5, 10), cv::Point(7, 200),
        cv::Scalar(255, 255, 255), 3, 8);
    cv::line(rgbImage, cv::Point(45, 10), cv::Point(46, 190),
        cv::Scalar(255, 255, 255), 3, 8);
    cv::line(rgbImage, cv::Point(47, 15), cv::Point(44, 210),
        cv::Scalar(255, 255, 255), 3, 8);
    cv::line(rgbImage, cv::Point(100, 20), cv::Point(102, 85),
        cv::Scalar(255, 255, 255), 3, 8);
    cv::line(rgbImage, cv::Point(150, 10), cv::Point(175, 140),
        cv::Scalar(255, 255, 255), 3, 8);
    cv::line(rgbImage, cv::Point(70, 30), cv::Point(72, 195),
        cv::Scalar(0, 0, 255), 3, 8);
    cv::line(rgbImage, cv::Point(182, 15), cv::Point(180, 215),
        cv::Scalar(255, 255, 255), 3, 8);

    cv::Mat resizedImage;
    cv::resize(rgbImage, resizedImage, cv::Size(480, 480));

    cv::Mat grayImage;
    cv::cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
    cv::Mat binaryImage = (grayImage > 0);

    std::vector<cv::Vec4i> verticalLines = performProbHoughLines(
        resizedImage, binaryImage);

    ASSERT_EQ(2, verticalLines.size());

    EXPECT_NEAR(verticalLines[0][0], 180, 2);
    EXPECT_NEAR(verticalLines[0][1], 215, 2);
    EXPECT_NEAR(verticalLines[0][2], 182, 2);
    EXPECT_NEAR(verticalLines[0][3], 15, 2);

    EXPECT_NEAR(verticalLines[1][0], 45, 2);
    EXPECT_NEAR(verticalLines[1][1], 210, 2);
    EXPECT_NEAR(verticalLines[1][2], 47, 2);
    EXPECT_NEAR(verticalLines[1][3], 15, 2);
  }

  TEST_F(SoftObstacleDetectorTest, isROIDetectedCorrectly)
  {
    std::vector<cv::Vec4i> lines;
    lines.push_back(cv::Vec4i(1, 1, 1, 3));
    lines.push_back(cv::Vec4i(2, 1, 2, 3));
    lines.push_back(cv::Vec4i(4, 1, 4, 3));

    boost::shared_ptr<cv::Rect> roi(new cv::Rect);

    ASSERT_EQ(0.5, detectROI(lines, 4, roi));

    EXPECT_EQ(1, roi->x);
    EXPECT_EQ(1, roi->y);
    // Considering that a pixel has width and height equal to 1
    EXPECT_EQ(4, roi->width);
    EXPECT_EQ(3, roi->height);
  }

  TEST_F(SoftObstacleDetectorTest, isLineMedianCorrectlyCalculated)
  {
    cv::Mat depthImage = cv::Mat::zeros(15, 15, CV_32FC1);

    for (int ii = 0; ii < depthImage.cols; ii++)
    {
      if (ii % 2)
      {
        depthImage.at<float>(10, ii) = ii;
      }
      depthImage.at<float>(7, ii) = 0.2 * ii;
    }
    float median = calculateLineMedian(depthImage, cv::Vec4i(4, 3, 5, 6));

    ASSERT_NEAR(median, 1.8, 1e-5);
  }

  TEST_F(SoftObstacleDetectorTest, isROIDepthDifferent)
  {
    cv::Mat depthImage = cv::Mat::ones(20, 20, CV_32FC1);

    std::vector<cv::Vec4i> lines;
    lines.push_back(cv::Vec4i(2, 2, 3, 8));
    lines.push_back(cv::Vec4i(4, 3, 3, 9));
    lines.push_back(cv::Vec4i(8, 4, 9, 8));

    cv::Rect roi(2, 2, 8, 8);
    ASSERT_FALSE(findDifferentROIDepth(depthImage, lines, roi));

    cv::line(depthImage, cv::Point(4, 4), cv::Point(6, 16), cv::Scalar(0.4), 1, 8);
    ASSERT_FALSE(findDifferentROIDepth(depthImage, lines, roi));

    cv::line(depthImage, cv::Point(8, 6), cv::Point(6, 18), cv::Scalar(0.4), 1, 8);
    ASSERT_FALSE(findDifferentROIDepth(depthImage, lines, roi));

    cv::line(depthImage, cv::Point(16, 8), cv::Point(18, 16), cv::Scalar(0.4), 1, 8);
    ASSERT_TRUE(findDifferentROIDepth(depthImage, lines, roi));

    cv::line(depthImage, cv::Point(4, 4), cv::Point(6, 16), cv::Scalar(0.6), 1, 8);
    cv::line(depthImage, cv::Point(8, 6), cv::Point(6, 18), cv::Scalar(0.6), 1, 8);
    cv::line(depthImage, cv::Point(16, 8), cv::Point(18, 16), cv::Scalar(0.6), 1, 8);
    ASSERT_FALSE(findDifferentROIDepth(depthImage, lines, roi));

    cv::line(depthImage, cv::Point(4, 4), cv::Point(6, 16), cv::Scalar(0.2), 1, 8);
    cv::line(depthImage, cv::Point(8, 6), cv::Point(6, 18), cv::Scalar(0.2), 1, 8);
    cv::line(depthImage, cv::Point(16, 8), cv::Point(18, 16), cv::Scalar(0.2), 1, 8);
    ASSERT_TRUE(findDifferentROIDepth(depthImage, lines, roi));
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
