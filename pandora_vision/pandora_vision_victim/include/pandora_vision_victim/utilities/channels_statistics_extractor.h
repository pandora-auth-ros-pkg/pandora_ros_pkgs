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
#ifndef PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H

#include "pandora_vision_victim/feature_extractors/mean_std_dev.h"
#include "pandora_vision_victim/feature_extractors/dominant_color.h"
#include "pandora_vision_victim/feature_extractors/dft_coeffs.h"
#include "pandora_vision_victim/feature_extractors/color_angles.h"

namespace pandora_vision
{
  class ChannelsStatisticsExtractor 
  {
      public:

      /**
      @brief This function returns the histogram of one color component from 
      the src image.
      @param planes [cv::Mat] contains the pixel values of a color component.
      @param bins [int] num of bins where the histogram will be divided.
      @param histRange [const float*] the range of the histogram.
      @return [cv::Mat] the calculated histogram.
      **/ 
      static cv::Mat computeHist(cv::Mat planes, int bins, const float* histRange);
      
      /**
      @brief This is the main function which calls all other for the 
      computation of the color features.
      @param src [cv::Mat] current frame to be processed
      @return void
      **/ 
      static void findChannelsStatisticsFeatures(const cv::Mat& src, std::vector<double>* rgbStatisticsVector);
      
      /**
      @brief This is the main function which calls all other for the 
      computation of the statistics feature for depth image.
      @param src [cv::Mat] depth image to be processed
      @return void
      **/ 
      static void findDepthChannelsStatisticsFeatures(const cv::Mat& src,  std::vector<double>* depthStatisticsVector);
  };
  
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H
