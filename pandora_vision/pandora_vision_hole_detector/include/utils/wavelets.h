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

#ifndef UTILS_WAVELETS_H
#define UTILS_WAVELETS_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "utils/defines.h"
#include "utils/visualization.h"

namespace pandora_vision
{
  class Wavelets
  {
    public:

      Wavelets();

      std::vector<float> getH0(int index);

      std::vector<float> getG0(const std::vector<float>& H0);

      std::vector<float> getG1(const std::vector<float>& H0);

      std::vector<float> getH1(const std::vector<float>& G1);

      static cv::Mat convCols(const cv::Mat& in,
        const std::vector<float>& kernel);

      static cv::Mat convRows(const cv::Mat& in,
        const std::vector<float>& kernel);

      static cv::Mat getLowLow(const cv::Mat& in,
        const std::vector<float>& kernel);

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
      static void getLowLow(const cv::Mat& inImage,
        const double& min, const double& max,
        cv::Mat* outImage);

      /**
        @brief Returns a RGB CV_8UC1 image containing the low-low part of the
        input RGB image, which is also in CV_8UC1 format
        @param[in] inImage [const cv::Mat&] The input image in CV_8UC1 format
        @param[out] outImage [cv::Mat*] The low-low part of the inImage in
        CV_8UC1 format
        @return void
       **/
      static void getLowLow(const cv::Mat& inImage, cv::Mat* outImage);

      static cv::Mat getLowHigh(const cv::Mat& in,
        const std::vector<float>& kernel0,
        const std::vector<float>& kernel1);

      static cv::Mat getHighLow(const cv::Mat& in,
        const std::vector<float>& kernel0,
        const std::vector<float>& kernel1);

      static cv::Mat getHighHigh(const cv::Mat& in,
        const std::vector<float>& kernel);
  };

} // namespace pandora_vision

#endif  // UTILS_WAVELETS_H
