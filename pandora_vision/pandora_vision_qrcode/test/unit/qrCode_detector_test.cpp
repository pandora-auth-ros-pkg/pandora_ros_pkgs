/*********************************************************************
 * *
 * * Software License Agreement (BSD License)
 * *
 * * Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 * * All rights reserved.
 * *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * *
 * * * Redistributions of source code must retain the above copyright
 * * notice, this list of conditions and the following disclaimer.
 * * * Redistributions in binary form must reproduce the above
 * * copyright notice, this list of conditions and the following
 * * disclaimer in the documentation and/or other materials provided
 * * with the distribution.
 * * * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 * * contributors may be used to endorse or promote products derived
 * * from this software without specific prior written permission.
 * *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * * POSSIBILITY OF SUCH DAMAGE.
 * *
 * * Author: Vasilis Bosdelekidis
 * *********************************************************************/

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>

#include "pandora_vision_qrcode/qrcode_detector.h"

namespace pandora_vision
{
  /**
   * @class QrCodeDetectorTests
   * @brief Tests the integrity of methods of class QrCodeDetector
   */
  class QrCodeDetectorTest : public ::testing::Test
  {
    public:
      QrCodeDetectorTest() : qrCodeDetectorPtr_() {}

      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;

        std::string pkgPath = ros::package::getPath("pandora_vision_qrcode");
        std::string paramsPath = pkgPath + "/" + "config/qrcode_params.yaml";

        int gaussianSharpenBlur = 5;
        float gaussianSharpenWeight = 0.8;
        bool debugCode = false;

        qrCodeDetectorPtr_ = new QrCodeDetector(gaussianSharpenBlur,
            gaussianSharpenWeight, debugCode);

        ASSERT_FALSE(qrCodeDetectorPtr_ == false) << "Could not initialize"
          << " the QR Code detector!";
      }

      std::vector<POIPtr> detectQrCode(cv::Mat frame);
      int* locateQrCode(cv::Point2f center);

      void drawChessboard(int blocksNumberH, int blocksNumberV,
          cv::Mat& image);

      int WIDTH;
      int HEIGHT;

      QrCodeDetector* qrCodeDetectorPtr_;
  };

  int* QrCodeDetectorTest::locateQrCode(cv::Point2f qrcode_center)
  {
    int* center = new int[2];
    center[0] = round(qrcode_center.y);
    center[1] = round(qrcode_center.x);
    return center;
  }

  /**
   * @brief Constructs a chessboard with specific number of blocks.
   * @param blocksNumberH [int] The number of chessboard blocks, horizontal direction
   * @param blocksNumberV [int] The number of chessboard blocks, vertical direction
   * @param image [cv::Mat&] The final chessboard image
   * @return void
   */
  void QrCodeDetectorTest::drawChessboard(int blocksNumberH, int blocksNumberV,
    cv::Mat& image)
  {
    int imageSize = WIDTH * HEIGHT;
    int blockWidth = static_cast<int>(WIDTH / blocksNumberH);
    int blockHeight = static_cast<int>(HEIGHT / blocksNumberV);
    cv::Mat chessBoard(HEIGHT, WIDTH, CV_8UC3, cv::Scalar::all(0));
    unsigned char color = 255;
    for (int i = 0; i < WIDTH - blockWidth; i += blockWidth)
    {
      for (int j = 0; j < HEIGHT - blockHeight; j += blockHeight)
      {
        cv::Mat ROI = chessBoard(cv::Rect(i, j, blockWidth, blockHeight));
        ROI.setTo(cv::Scalar::all(color));
        color = abs(color - 255);
      }
    }
    chessBoard.copyTo(image);
  }

  /// Tests QrCodeDetector::detectQrCode
  TEST_F(QrCodeDetectorTest, detectQrCodeBlackImage)
  {
    cv::Mat blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    std::vector<POIPtr> qrcode_list =
      qrCodeDetectorPtr_->detectQrCode(blackFrame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    // neither when 3 channels are used
    blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    qrcode_list = qrCodeDetectorPtr_->detectQrCode(blackFrame);
    EXPECT_EQ(0, qrcode_list.size());
  }

  TEST_F(QrCodeDetectorTest, detectQrCodeWhiteImage)
  {
    cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    whiteFrame.setTo(cv::Scalar(255, 255, 255));
    std::vector<POIPtr> qrcode_list =
      qrCodeDetectorPtr_->detectQrCode(whiteFrame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
  }

  TEST_F(QrCodeDetectorTest, detectQrCodeWhiteBlackMixImage)
  {
    // Vertically concatenated
    cv::Mat blackFrame = cv::Mat::zeros(HEIGHT / 2, WIDTH / 2, CV_8UC3);
    cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT / 2, WIDTH / 2, CV_8UC3);
    whiteFrame.setTo(cv::Scalar(255, 255, 255));
    cv::Mat H, V;
    cv::hconcat(blackFrame, whiteFrame, H);
    cv::vconcat(blackFrame, whiteFrame, V);
    std::vector<POIPtr> qrcode_list = qrCodeDetectorPtr_->detectQrCode(H);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    qrcode_list = qrCodeDetectorPtr_->detectQrCode(V);
    EXPECT_EQ(0, qrcode_list.size());
  }

  TEST_F(QrCodeDetectorTest, detectQrCodeRandomChessboardImage)
  {
    cv::Mat frame;
    int blocksNumberH = 10;
    int blocksNumberV = 10;
    drawChessboard(blocksNumberH, blocksNumberV, frame);
    std::vector<POIPtr> qrcode_list = qrCodeDetectorPtr_->detectQrCode(frame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    blocksNumberH = 100;
    blocksNumberV = 100;
    drawChessboard(blocksNumberH, blocksNumberV, frame);
    qrcode_list = qrCodeDetectorPtr_->detectQrCode(frame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
  }
}  // namespace pandora_vision
