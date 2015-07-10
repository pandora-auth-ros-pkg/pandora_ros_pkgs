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
 *   Marios Protopapas <protopapas_marios@hotmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/
#ifndef PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_EXTRACTOR_H

#include <vector>

#include "pandora_vision_victim/channels_statistics_feature_extractors/mean_std_dev_extractor.h"
#include "pandora_vision_victim/channels_statistics_feature_extractors/dominant_color_extractor.h"
#include "pandora_vision_victim/channels_statistics_feature_extractors/dft_coeffs_extractor.h"
#include "pandora_vision_victim/channels_statistics_feature_extractors/color_angles_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class ChannelsStatisticsExtractor
  {
    public:
      /**
       * @brief This function extracts color related statistic features from a
       * color image.
       * @param src [const cv::Mat&] Color image to be processed.
       * @param colorStatisticsVector [std::vector<double>*] The color
       * statistics vector.
       * @return void
       */
      static void findColorChannelsStatisticsFeatures(const cv::Mat& src,
          std::vector<double>* colorStatisticsVector);

      /**
       * @brief This function extracts color related statistic features from a
       * depth image.
       * @param src [const cv::Mat&] Depth image to be processed.
       * @param depthStatisticsVector [std::vector<double>*] The depth
       * statistics vector.
       * @return void
       */
      static void findDepthChannelsStatisticsFeatures(const cv::Mat& src,
          std::vector<double>* depthStatisticsVector);
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_EXTRACTOR_H
