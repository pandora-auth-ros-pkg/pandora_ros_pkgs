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

#ifndef PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_FEATURE_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_FEATURE_EXTRACTOR_H

#include <vector>

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class ChannelsStatisticsFeatureExtractor
  {
    public:
      /**
       * @brief Constructor
       */
      explicit ChannelsStatisticsFeatureExtractor(cv::Mat* img)
      {
        img_ = img;
      }

      /**
       * @brief Destructor
       */
      virtual ~ChannelsStatisticsFeatureExtractor()
      {
      }

      /**
       * @brief This function extracts features from an image.
       * @return [std::vector<double>] The feature vector.
       */
      virtual std::vector<double> extract(void) = 0;

    protected:
      /// In case the extractor is not purely functional, multiple
      /// sub-extractors exist
      std::vector<ChannelsStatisticsFeatureExtractor*> extractors;

      cv::Mat* img_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_FEATURE_EXTRACTORS_CHANNELS_STATISTICS_FEATURE_EXTRACTOR_H

