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
#include "pandora_vision_qrcode/qrCode_detector.h"
#include "gtest/gtest.h"
#include "ros/ros.h"
#include "ros/package.h"

namespace pandora_vision
{
  /**
   *     @class QrCodeDetectorTests
   *     @brief Tests the integrity of methods of class QrCodeDetector
   **/
  class QrCodeDetectorTest : public ::testing::Test
  {
    public:
      QrCodeDetectorTest() : qrCodeDetector_() {}

      virtual void SetUp()
      {
        WIDTH = 640;  // getParam
        HEIGHT = 480;
      }

      std::vector<POIPtr> detectQrCode(cv::Mat frame);
      int* locateQrCode(cv::Point2f center);

      void drawChessboard(int blocksNumberH, int blocksNumberV, cv::Mat& image);

      int WIDTH;
      int HEIGHT;

    private:
      QrCodeDetector qrCodeDetector_;
  };

  std::vector<POIPtr> QrCodeDetectorTest::detectQrCode(cv::Mat frame)
  {
    return qrCodeDetector_.detectQrCode(frame);
  }

  int* QrCodeDetectorTest::locateQrCode(cv::Point2f qrcode_center)
  {
    int* center = new int[2];
    center[0] = round(qrcode_center.y);
    center[1] = round(qrcode_center.x);
    return center;
  }

  /**
    @brief Constructs a chessboard with specific number of blocks.
    @param blocksNumberH [int] The number of chessboard blocks, horizontal direction
    @param blocksNumberV [int] The number of chessboard blocks, vertical direction
    @param image [cv::Mat&] The final chessboard image
    @return void
   **/
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
  
  //!< Tests QrCodeDetector::detectQrCode
  TEST_F(QrCodeDetectorTest, detectQrCodeBlackImage)
  {
    cv::Mat blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    std::vector<POIPtr> qrcode_list = detectQrCode(blackFrame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    // neither when 3 channels are used
    blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    qrcode_list = detectQrCode(blackFrame);
    EXPECT_EQ(0, qrcode_list.size());
  }

  TEST_F(QrCodeDetectorTest, detectQrCodeWhiteImage)
  {
    cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    whiteFrame.setTo(cv::Scalar(255, 255, 255));
    std::vector<POIPtr> qrcode_list = detectQrCode(whiteFrame);
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
    std::vector<POIPtr> qrcode_list = detectQrCode(H);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    qrcode_list = detectQrCode(V);
    EXPECT_EQ(0, qrcode_list.size());
  }

  TEST_F(QrCodeDetectorTest, detectQrCodeRandomChessboardImage)
  {
    cv::Mat frame;
    int blocksNumberH = 10;
    int blocksNumberV = 10;
    drawChessboard(blocksNumberH, blocksNumberV, frame);
    std::vector<POIPtr> qrcode_list = detectQrCode(frame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
    blocksNumberH = 100;
    blocksNumberV = 100;
    drawChessboard(blocksNumberH, blocksNumberV, frame);
    qrcode_list = detectQrCode(frame);
    // there shouldn't be any qrcodes
    EXPECT_EQ(0, qrcode_list.size());
  }

  // TEST_F (QrCodeDetectorTest, locateQrCodeInImage)
  // {
  //    std::stringstream fileName;
  //    std::vector<QrCode> qrcode_list;
  //    cv::Mat inputFrame;
  //    int* center;
  //    int  cArray[9];
  //    FILE *fpr;
  //    fileName.str("");
  //    fileName << ros::package::getPath("pandora_vision_qrcode");
  //    fileName << "/test/unit/data/" << "test_qr_centers.txt"; 
  //    fpr = fopen(fileName.str().c_str(), "r");
  //    fileName.str("");
  //    for(int i = 1; i < 13; i ++){
  //        fileName << ros::package::getPath("pandora_vision_qrcode");
  //        fileName << "/test/unit/data/" << "test_qr_" << i << ".jpg"; 
  //        inputFrame = cv::imread(fileName.str());
  //        fscanf(fpr, " %d", &cArray[0]);
  //        for( int j = 1; j < (1 + cArray[0] * 4); j ++)
  //        {
  //            fscanf(fpr, " %d", &cArray[j]);
  //        }
  //        if(!inputFrame.data)
  //        {
  //            ROS_ERROR("Cannot open image.");
  //            fileName.str("");
  //            continue;
  //        }
  //        qrcode_list = detectQrCode(inputFrame);
  //        EXPECT_EQ(cArray[0], qrcode_list.size());
  //        if(cArray[0] == qrcode_list.size())
  //        {
  //            for( int j = 0; j < qrcode_list.size(); j ++)
  //            {
  //                center = locateQrCode(qrcode_list[j].qrcode_center);
  //                EXPECT_LE(cArray[1 + 0 + j * 4], center[0]);   
  //                EXPECT_GE(cArray[1 + 1 + j * 4], center[0]);   
  //                EXPECT_LE(cArray[1 + 2 + j * 4], center[1]);   
  //                EXPECT_GE(cArray[1 + 3 + j * 4], center[1]);   
  //            }
  //        }
  //        fileName.str("");
  //    }
  //    fclose(fpr);
  // }
}  // namespace pandora_vision
