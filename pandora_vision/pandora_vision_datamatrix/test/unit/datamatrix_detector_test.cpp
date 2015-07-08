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
#include "pandora_vision_datamatrix/datamatrix_detector.h"

#include "gtest/gtest.h"

namespace pandora_vision
{
namespace pandora_vision_datamatrix
{
    /**
     *     @class DatamatrixDetectorTest
     *     @brief Tests the integrity of methods of class DatamatrixDetector
    **/
  
    class DatamatrixDetectorTest : public ::testing::Test
    {
      public:
        DatamatrixDetectorTest() {}

        virtual void SetUp()
        {
          WIDTH = 640;
          HEIGHT = 480;
        }

        std::vector<POIPtr> detectDatamatrix(cv::Mat frame);
        int* locateDatamatrix(cv::Point2f center);

        void drawChessboard (
          int blocksNumberH,
          int blocksNumberV,
          cv::Mat &image
        );

        int WIDTH;
        int HEIGHT;

      private:
        DatamatrixDetector datamatrixDetector_;
    };

    std::vector<POIPtr> DatamatrixDetectorTest::detectDatamatrix(cv::Mat frame)
    {
      return datamatrixDetector_.detect_datamatrix(frame);
    }

    int* DatamatrixDetectorTest::locateDatamatrix(cv::Point2f datamatrix_center)
    {
      int* center = new int[2];
      center[0] = round(datamatrix_center.y);
      center[1] = round(datamatrix_center.x);
      return center;
    }

    /**
      @brief Constructs a chessboard with specific number of blocks.
      @param blocksNumberH [int] The number of chessboard blocks, horizontal direction
      @param blocksNumberV [int] The number of chessboard blocks, vertical direction
      @param image [cv::Mat&] The final chessboard image
      @return void
    **/
    void DatamatrixDetectorTest::drawChessboard (
      int blocksNumberH,
      int blocksNumberV,
      cv::Mat &image
    )
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
          color = abs(color-255);
        }
      }
      chessBoard.copyTo(image);
    }
    //! Tests DatamatrixDetector::detect_datamatrix
    TEST_F (DatamatrixDetectorTest, detect_datamatrixBlackImage)
    {
      cv::Mat blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
      std::vector<POIPtr> datamatrix_list = detectDatamatrix(blackFrame);
      // there shouldn't be any datamatrices
      EXPECT_EQ(0, datamatrix_list.size());
      // neither when 3 channels are used
      blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
      datamatrix_list = detectDatamatrix(blackFrame);
      EXPECT_EQ(0, datamatrix_list.size());
    }

    TEST_F (DatamatrixDetectorTest, detect_datamatrixWhiteImage)
    {
      cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
      whiteFrame.setTo(cv::Scalar(255, 255, 255));
      std::vector<POIPtr> datamatrix_list = detectDatamatrix(whiteFrame);
      // there shouldn't be any datamatrices
      EXPECT_EQ(0, datamatrix_list.size());
    }

    TEST_F (DatamatrixDetectorTest, detect_datamatrixWhiteBlackMixImage)
    {
      // Vertically concatenated
      cv::Mat blackFrame = cv::Mat::zeros(HEIGHT/2, WIDTH/2, CV_8UC3);
      cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT/2, WIDTH/2, CV_8UC3);
      whiteFrame.setTo(cv::Scalar(255, 255, 255));
      cv::Mat H, V;
      cv::hconcat(blackFrame, whiteFrame, H);
      cv::vconcat(blackFrame, whiteFrame, V);
      std::vector<POIPtr> datamatrix_list = detectDatamatrix(H);
      // there shouldn't be any datamatrices
      EXPECT_EQ(0, datamatrix_list.size());
      datamatrix_list = detectDatamatrix(V);
      EXPECT_EQ(0, datamatrix_list.size());
    }


    TEST_F (DatamatrixDetectorTest, detect_datamatrixRandomChessboardImage)
    {
      cv::Mat frame;
      int blocksNumberH = 10;
      int blocksNumberV = 10;
      drawChessboard( blocksNumberH, blocksNumberV, frame);
      std::vector<POIPtr> datamatrix_list = detectDatamatrix(frame);
      // there shouldn't be any qrcodes
      EXPECT_EQ(0, datamatrix_list.size());
      blocksNumberH = 100;
      blocksNumberV = 100;
      drawChessboard( blocksNumberH, blocksNumberV, frame);
      datamatrix_list = detectDatamatrix(frame);
      // there shouldn't be any qrcodes
      EXPECT_EQ(0, datamatrix_list.size());
    }
    
    //TEST_F (DatamatrixDetectorTest, detect_datamatrixFromImage)
    //{
    //  cv::Mat inputFrame;
    //  inputFrame = cv::imread("/home/v/Documents/PANDORA/Vision/Qr_Datamatrix_Testing/datamatrix1.jpg");
    //  //cv::resize(inputFrame, inputFrame, cv::Size(WIDTH, HEIGHT));
    //  std::vector<POIPtr> datamatrix_list = detectDatamatrix(inputFrame);
    //  // there should be one datamatrix
    //  ASSERT_EQ(1, datamatrix_list.size());
    //  int* center = locateDatamatrix(datamatrix_list[0].datamatrix_center);
    //  EXPECT_LE(130, center[0]);   
    //  EXPECT_GE(160, center[0]);   
    //  EXPECT_LE(263, center[1]);   
    //  EXPECT_GE(293, center[1]);   
    //  // inputFrame = cv::imread("");
    //  // datamatrix_list = detectDatamatrix(inputFrame);
    //  // // there should be four datamatrices
    //  // EXPECT_EQ(4, datamatrix_list.size());
    //  // inputFrame = cv::imread("");
    //  // datamatrix_list = detectDatamatrix(inputFrame);
    //  // // there should be one datamatrix
    //  // EXPECT_EQ(1, datamatrix_list.size());
    //}
      

}  // namepsace pandora_vision_datamatrix
}  // namespace pandora_vision
