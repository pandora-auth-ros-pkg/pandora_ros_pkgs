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
 * Authors: Victor Daropoulos, Alexandros Philotheou
 *********************************************************************/

#include "utils/wavelets.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  cv::Mat Wavelets::convCols(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    #ifdef DEBUG_TIME
    Timer::start("convCols", "getLowLow");
    #endif

    int length = in.rows + kernel.size() - 1;

    cv::Mat temp = cv::Mat(length, in.cols, CV_32FC1);

    for (int k = 0; k < in.cols; k++)
    {
      for (int i = 0; i < length; i++)
      {
        float y = 0;

        for (int j = 0; j < kernel.size(); j++)
        {
          if ((i - j) >= 0 && (i - j) < in.rows)
          {
            y += in.at<float>(i - j, k) * kernel.at(j);
          }
        }

        temp.at<float>(i, k) = y;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("convCols");
    #endif

    return temp;
  }



  cv::Mat Wavelets::convRows(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    #ifdef DEBUG_TIME
    Timer::start("convRows", "getLowLow");
    #endif

    int length = in.cols + kernel.size() - 1;

    cv::Mat temp = cv::Mat(in.rows, length, CV_32FC1);

    for (int k = 0; k < in.rows; k++)
    {
      for (int i = 0; i < length; i++)
      {
        float y = 0;

        for (int j = 0; j < kernel.size(); j++)
        {
          if ((i - j) >= 0 && (i - j) < in.cols)
          {
            y += in.at<float>(k, i - j) * kernel.at(j);
          }
        }

        temp.at<float>(k, i) = y;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("convRows");
    #endif

    return temp;
  }



  cv::Mat Wavelets::getLowLow(const cv::Mat& in,
    const std::vector<float>& kernel)
  {
    cv::Mat temp0 = convCols(in, kernel);

    cv::Mat tempy0;

    for (int i = 0; i < temp0.rows; i += 2)
    {
      tempy0.push_back(temp0.row(i));
    }

    cv::Mat tempy00 = convRows(tempy0, kernel);

    cv::Mat tempy000;

    cv::transpose(tempy00, tempy00);

    for (int i = 0; i < tempy00.rows; i += 2)
    {
      tempy000.push_back(tempy00.row(i));
    }

    cv::transpose(tempy000, tempy000);

    double minVal;
    double maxVal;

    cv::minMaxLoc(tempy000, &minVal, &maxVal);

    tempy000 = tempy000 / maxVal;

    return tempy000;
  }


  /**
    @brief Returns a CV_32FC1 image containing the low-low part of the
    input image, which is also in CV_32FC1 format
    @param[in] inImage [const cv::Mat&] The input image in CV_32FC1 format
    @param[in] min [const double&] The inImage's min value
    @param[in] max [const double&] The inImage's max value
    @param[out] outImage [cv::Mat*] The low-low part of the inImage in
    CV_32FC1 format
    @return void
   **/
  void Wavelets::getLowLow(const cv::Mat& inImage,
    const double& min, const double& max,
    cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("getLowLow", "inputDepthImageCallback");
    #endif

    cv::Mat temp = cv::Mat(inImage.size(), CV_8UC1);

    temp = Visualization::scaleImageForVisualization(inImage, 0);

    Wavelets wave;

    std::vector<float> H0;

    for (int i = 0; i < 2; i++)
    {
      H0.push_back(1 / sqrt(2));
    }

    cv::Mat doubled = cv::Mat(temp.rows, temp.cols, CV_32FC1);

    for (int y = 0; y < doubled.rows; y++)
    {
      for (int x = 0; x < doubled.cols; x++)
      {
        doubled.at<float>(y, x) =
          static_cast<float>(temp.at<unsigned char>(y, x) / 255.0);
      }
    }

    // LowLow contains the inImage's low low frequencies, in CV_32FC1 format
    // What we will return will be this image scaled to the actual proportions
    // of values of the inImage (also in CV_32FC1 format).
    // After obtaining the low-low, reverse the scale operation, in an
    // attempt to approximate the initial depth image's values
    *outImage = wave.getLowLow(doubled, H0) * (max - min);

    #ifdef DEBUG_TIME
    Timer::tick("getLowLow");
    #endif
  }



  /**
    @brief Returns a RGB CV_8UC3 image containing the low-low part of the
    input RGB image, which is also in CV_8UC3 format
    @param[in] inImage [const cv::Mat&] The input image in CV_8UC3 format
    @param[out] outImage [cv::Mat*] The low-low part of the inImage in
    CV_8UC3 format
    @return void
   **/
  void Wavelets::getLowLow(const cv::Mat& inImage, cv::Mat* outImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("getLowLow", "inputRgbImageCallback");
    #endif

    Wavelets wave;

    std::vector<float> H0;

    for (int i = 0; i < 2; i++)
    {
      H0.push_back(1 / sqrt(2));
    }

    cv::Mat lowLow[3];

    for (int i = 0; i < 3; i++)
    {
      cv::Mat doubled = cv::Mat(inImage.rows, inImage.cols, CV_32FC1);

      for (int y = 0; y < doubled.rows; y++)
      {
        for (int x = 0; x < doubled.cols; x++)
        {
          doubled.at<float>(y, x) =
            inImage.at<unsigned char>(y, 3 * x + i) / 255.0;
        }
      }

      // LowLow contains the inImage's low low frequencies, in CV_32FC1 format
      // What we will return will be this image scaled to the actual
      // proportions of values of the inImage (also in CV_32FC1 format).
      lowLow[i] = wave.getLowLow(doubled, H0);
    }

    // The outImage. out has to be assigned to *outImage but cannot be done
    // in the following loops immediately
    cv::Mat out = cv::Mat(lowLow[0].size(), CV_8UC3);

    for (int i = 0; i < 3; i++)
    {
      for (int y = 0; y < lowLow[i].rows; y++)
      {
        for (int x = 0; x < lowLow[i].cols; x++)
        {
          out.at<unsigned char>(y, 3 * x + i) = lowLow[i].at<float>(y, x) * 255;
        }
      }
    }

    // Copy out to the output image
    out.copyTo(*outImage);

    #ifdef DEBUG_TIME
    Timer::tick("getLowLow");
    #endif
  }

}  // namespace pandora_vision_hole
}  // namespace pandora_vision
