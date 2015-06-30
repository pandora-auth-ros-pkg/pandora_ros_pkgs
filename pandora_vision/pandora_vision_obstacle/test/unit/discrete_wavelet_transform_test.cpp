/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
#include <cmath>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/discrete_wavelet_transform.h"

namespace pandora_vision
{
  class DiscreteWaveletTransformTest : public ::testing::Test
  {
    public:
      DiscreteWaveletTransformTest()
      {
        cv::Mat kernelLow = (cv::Mat_<float>(2, 1) << 1 / sqrt(2), 1 / sqrt(2));
        cv::Mat kernelHigh = (cv::Mat_<float>(2, 1) << 1 / sqrt(2), - 1 / sqrt(2));

        dwtPtr_.reset(new DiscreteWaveletTransform(kernelLow, kernelHigh));
      }

      cv::Mat convRows(const cv::Mat& image, bool low)
      {
        return dwtPtr_->convRows(image, low);
      }

      cv::Mat convCols(const cv::Mat& image, bool low)
      {
        return dwtPtr_->convCols(image, low);
      }

      void subSample(const cv::Mat& image, bool rows, const MatPtr& subImage)
      {
        return dwtPtr_->subSample(image, rows, subImage);
      }

    protected:
      virtual void SetUp() {}

      boost::shared_ptr<DiscreteWaveletTransform> dwtPtr_;
  };

  TEST_F(DiscreteWaveletTransformTest, isPadSizeInConvolutionCorrect)
  {
    cv::Mat image = cv::Mat::ones(3, 3, CV_32FC1);

    cv::Mat imageConvRows = convRows(image, true);
    ASSERT_EQ(4, imageConvRows.cols);
    ASSERT_EQ(3, imageConvRows.rows);

    cv::Mat imageConvCols = convCols(image, true);
    ASSERT_EQ(3, imageConvCols.cols);
    ASSERT_EQ(4, imageConvCols.rows);
  }

  TEST_F(DiscreteWaveletTransformTest, isConvolutionResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);

    cv::Mat imageConvRows = convRows(image, true);
    cv::Mat rowsResult = (cv::Mat_<float>(3, 4) << 1, 3, 5, 6, 4, 9, 11,
        12, 7, 15, 17, 18);
    rowsResult /= static_cast<float>(std::sqrt(2));

    ASSERT_TRUE(imageConvRows.rows == rowsResult.rows);
    ASSERT_TRUE(imageConvRows.cols == rowsResult.cols);
    for (int ii = 0; ii < rowsResult.rows; ii++)
    {
      for (int jj = 0; jj < rowsResult.cols; jj++)
      {
        EXPECT_NEAR(rowsResult.at<float>(ii, jj), imageConvRows.at<float>(ii, jj), 1e-6);
      }
    }

    cv::Mat imageConvCols = convCols(image, true);
    cv::Mat colsResult = (cv::Mat_<float>(4, 3) << 1, 2, 3, 5, 7, 9,
        11, 13, 15, 14, 16, 18);
    colsResult /= static_cast<float>(std::sqrt(2));

    ASSERT_TRUE(imageConvCols.rows == colsResult.rows);
    ASSERT_TRUE(imageConvCols.cols == colsResult.cols);
    for (int ii = 0; ii < colsResult.rows; ii++)
    {
      for (int jj = 0; jj < colsResult.cols; jj++)
      {
        EXPECT_NEAR(colsResult.at<float>(ii, jj), imageConvCols.at<float>(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isSubSampleRowsResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 1, 1, 2, 2, 2,
        3, 3, 3);

    MatPtr subImage(new cv::Mat());

    subSample(image, true, subImage);

    ASSERT_EQ(1, subImage->rows);
    ASSERT_EQ(3, subImage->cols);

    for (int ii = 0; ii < subImage->cols; ii++)
    {
      EXPECT_EQ(2, subImage->at<float>(0, ii));
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isSubSampleColsResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(4, 4) << 1, 2, 3, 4, 1, 2, 3,
        4, 1, 2, 3, 4, 1, 2, 3, 4);

    MatPtr subImage(new cv::Mat());

    subSample(image, false, subImage);

    ASSERT_EQ(2, subImage->cols);
    ASSERT_EQ(4, subImage->rows);

    for (int ii = 0; ii < subImage->rows; ii++)
    {
      EXPECT_EQ(2, subImage->at<float>(ii, 0));
      EXPECT_EQ(4, subImage->at<float>(ii, 1));
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isLowLowResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5,
        6, 7, 8, 9);

    std::vector<MatPtr> llImages = dwtPtr_->getLowLow(image);

    ASSERT_EQ(1, llImages.size());

    ASSERT_EQ(2, llImages[0]->rows);
    ASSERT_EQ(2, llImages[0]->cols);

    EXPECT_NEAR(llImages[0]->at<float>(0, 0), 6, 1e-5);
    EXPECT_NEAR(llImages[0]->at<float>(0, 1), 9, 1e-5);
    EXPECT_NEAR(llImages[0]->at<float>(1, 0), 15, 1e-5);
    EXPECT_NEAR(llImages[0]->at<float>(1, 1), 18, 1e-5);
  }

  TEST_F(DiscreteWaveletTransformTest, isLowHighResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5,
        6, 7, 8, 9);

    std::vector<MatPtr> lhImages = dwtPtr_->getLowHigh(image);

    ASSERT_EQ(1, lhImages.size());

    ASSERT_EQ(2, lhImages[0]->rows);
    ASSERT_EQ(2, lhImages[0]->cols);

    EXPECT_NEAR(lhImages[0]->at<float>(0, 0), - 1, 1e-5);
    EXPECT_NEAR(lhImages[0]->at<float>(0, 1), 0, 1e-5);
    EXPECT_NEAR(lhImages[0]->at<float>(1, 0), - 1, 1e-5);
    EXPECT_NEAR(lhImages[0]->at<float>(1, 1), 0, 1e-5);
  }

  TEST_F(DiscreteWaveletTransformTest, isHighLowResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5,
        6, 7, 8, 9);

    std::vector<MatPtr> hlImages = dwtPtr_->getHighLow(image);

    ASSERT_EQ(1, hlImages.size());

    ASSERT_EQ(2, hlImages[0]->rows);
    ASSERT_EQ(2, hlImages[0]->cols);

    EXPECT_NEAR(hlImages[0]->at<float>(0, 0), - 3, 1e-5);
    EXPECT_NEAR(hlImages[0]->at<float>(0, 1), - 3, 1e-5);
    EXPECT_NEAR(hlImages[0]->at<float>(1, 0), 0, 1e-5);
    EXPECT_NEAR(hlImages[0]->at<float>(1, 1), 0, 1e-5);
  }

  TEST_F(DiscreteWaveletTransformTest, isHighHighResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5,
        6, 7, 8, 9);

    std::vector<MatPtr> hhImages = dwtPtr_->getHighHigh(image);

    ASSERT_EQ(1, hhImages.size());

    ASSERT_EQ(2, hhImages[0]->rows);
    ASSERT_EQ(2, hhImages[0]->cols);

    EXPECT_NEAR(hhImages[0]->at<float>(0, 0), 0, 1e-5);
    EXPECT_NEAR(hhImages[0]->at<float>(0, 1), 0, 1e-5);
    EXPECT_NEAR(hhImages[0]->at<float>(1, 0), 0, 1e-5);
    EXPECT_NEAR(hhImages[0]->at<float>(1, 1), 0, 1e-5);
  }

  TEST_F(DiscreteWaveletTransformTest, isDWTResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(4, 4) << 1, 2, 3, 4, 5,
        6, 7, 8, 9, 1, 2, 3, 4, 5, 6, 7);

    std::vector<MatPtr> dwtImages = dwtPtr_->dwt2D(image);

    ASSERT_EQ(4, dwtImages.size());

    for (int ii = 0; ii < dwtImages.size(); ii++)
    {
      ASSERT_EQ(2, dwtImages[ii]->rows);
      ASSERT_EQ(2, dwtImages[ii]->cols);
    }

    cv::Mat llImage = (cv::Mat_<float>(2, 2) << 7, 11,
        9.5, 9);

    for (int ii = 0; ii < llImage.rows; ii++)
    {
      for (int jj = 0; jj < llImage.cols; jj++)
      {
        EXPECT_NEAR(llImage.at<float>(ii, jj),
            dwtImages[0]->at<float>(ii, jj), 1e-5);
      }
    }

    cv::Mat lhImage = (cv::Mat_<float>(2, 2) << - 1, - 1,
        3.5, - 1);

    for (int ii = 0; ii < lhImage.rows; ii++)
    {
      for (int jj = 0; jj < lhImage.cols; jj++)
      {
        EXPECT_NEAR(lhImage.at<float>(ii, jj),
            dwtImages[1]->at<float>(ii, jj), 1e-5);
      }
    }

    cv::Mat hlImage = (cv::Mat_<float>(2, 2) << - 4, - 4,
        0.5, - 4);

    for (int ii = 0; ii < hlImage.rows; ii++)
    {
      for (int jj = 0; jj < hlImage.cols; jj++)
      {
        EXPECT_NEAR(hlImage.at<float>(ii, jj),
            dwtImages[2]->at<float>(ii, jj), 1e-5);
      }
    }

    cv::Mat hhImage = (cv::Mat_<float>(2, 2) << 0, 0,
        4.5, 0);

    for (int ii = 0; ii < hhImage.rows; ii++)
    {
      for (int jj = 0; jj < hhImage.cols; jj++)
      {
        EXPECT_NEAR(hhImage.at<float>(ii, jj),
            dwtImages[3]->at<float>(ii, jj), 1e-5);
      }
    }
  }
}  // namespace pandora_vision
