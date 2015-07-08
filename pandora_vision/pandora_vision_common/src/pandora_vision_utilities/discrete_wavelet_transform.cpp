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
#include "pandora_vision_common/pandora_vision_utilities/discrete_wavelet_transform.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  DiscreteWaveletTransform::DiscreteWaveletTransform(int kernelSize)
  {
  }

  DiscreteWaveletTransform::DiscreteWaveletTransform(const cv::Mat& columnKernelLow,
      const cv::Mat& columnKernelHigh)
  {
    columnKernelLow_ = columnKernelLow;
    cv::transpose(columnKernelLow, rowKernelLow_);

    columnKernelHigh_ = columnKernelHigh;
    cv::transpose(columnKernelHigh, rowKernelHigh_);
  }

  DiscreteWaveletTransform::~DiscreteWaveletTransform()
  {
  }

  cv::Mat DiscreteWaveletTransform::optionalConv(const cv::Mat& inImage,
      bool low, bool rows)
  {
    cv::Mat paddedImage, result;
    if (rows)
    {
      cv::copyMakeBorder(inImage, paddedImage, 0, 0, 0, rowKernelLow_.cols - 1, cv::BORDER_REPLICATE);
      if (low)
      {
        cv::filter2D(paddedImage, result, -1, rowKernelLow_, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
      }
      else
      {
        cv::filter2D(paddedImage, result, -1, rowKernelHigh_, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
      }
    }
    else
    {
      cv::copyMakeBorder(inImage, paddedImage, 0, columnKernelLow_.rows - 1, 0, 0, cv::BORDER_REPLICATE);
      if (low)
      {
        cv::filter2D(paddedImage, result, -1, columnKernelLow_, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
      }
      else
      {
        cv::filter2D(paddedImage, result, -1, columnKernelHigh_, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
      }
    }
    return result;
  }

  cv::Mat DiscreteWaveletTransform::convCols(const cv::Mat& inImage, bool low)
  {
    return optionalConv(inImage, low, false);
  }

  cv::Mat DiscreteWaveletTransform::convRows(const cv::Mat& inImage, bool low)
  {
    return optionalConv(inImage, low, true);
  }

  void DiscreteWaveletTransform::subSample(const cv::Mat& image, bool rows,
      const MatPtr& subImage)
  {
    if (rows)
    {
      for (int jj = 1; jj < image.rows; jj += 2)
      {
        subImage->push_back(image.row(jj));
      }
    }
    else
    {
      *subImage = cv::Mat::zeros(image.rows, static_cast<int>(
          std::floor(image.cols / 2.0f)), CV_32FC(image.channels()));
      int imageIndex = 0;
      for (int jj = 1; jj < image.cols; jj += 2)
      {
        image.col(jj).copyTo(subImage->col(imageIndex));
        imageIndex += 1;
      }
    }
  }

  void DiscreteWaveletTransform::convAndSubSample(const cv::Mat& inImage, bool rows,
      const MatPtr& subImageLow, const MatPtr& subImageHigh)
  {
    cv::Mat imageConvLow = optionalConv(inImage, true, rows);
    cv::Mat imageConvHigh = optionalConv(inImage, false, rows);

    subSample(imageConvLow, !rows, subImageLow);
    subSample(imageConvHigh, !rows, subImageHigh);
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getLowLow(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> LLImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      cv::Mat imageConvLow = convCols(input, true);

      MatPtr subImageL(new cv::Mat);
      subSample(imageConvLow, true, subImageL);

      imageConvLow = convRows(*subImageL, true);

      MatPtr subImageLL(new cv::Mat);
      subSample(imageConvLow, false, subImageLL);

      LLImages.push_back(subImageLL);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return LLImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getLowHigh(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> LHImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      cv::Mat imageConvLow = convCols(input, true);

      MatPtr subImageL(new cv::Mat);
      subSample(imageConvLow, true, subImageL);

      MatPtr subImageLL(new cv::Mat), subImageLH(new cv::Mat);
      convAndSubSample(*subImageL, true, subImageLL, subImageLH);

      LHImages.push_back(subImageLH);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return LHImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getHighLow(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> HLImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      cv::Mat imageConvLow = convRows(*subImageH, true);

      MatPtr subImageHL(new cv::Mat);
      subSample(imageConvLow, false, subImageHL);

      HLImages.push_back(subImageHL);

      if (ii < level - 1)
      {
        imageConvLow = convRows(*subImageL, true);

        MatPtr subImageLL(new cv::Mat);
        subSample(imageConvLow, false, subImageLL);

        input = subImageLL->clone();
      }
    }
    return HLImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getHighHigh(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> HHImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      cv::Mat imageConvHigh = convRows(*subImageH, false);

      MatPtr subImageHH(new cv::Mat);
      subSample(imageConvHigh, false, subImageHH);

      HHImages.push_back(subImageHH);

      if (ii < level - 1)
      {
        cv::Mat imageConvLow = convRows(*subImageL, true);

        MatPtr subImageLL(new cv::Mat);
        subSample(imageConvLow, false, subImageLL);

        input = subImageLL->clone();
      }
    }
    return HHImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::dwt2D(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> dwtImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      MatPtr subImageLL(new cv::Mat), subImageLH(new cv::Mat);
      convAndSubSample(*subImageL, true, subImageLL, subImageLH);

      dwtImages.push_back(subImageLL);
      dwtImages.push_back(subImageLH);

      MatPtr subImageHL(new cv::Mat), subImageHH(new cv::Mat);
      convAndSubSample(*subImageH, true, subImageHL, subImageHH);

      dwtImages.push_back(subImageHL);
      dwtImages.push_back(subImageHH);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return dwtImages;
  }

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
