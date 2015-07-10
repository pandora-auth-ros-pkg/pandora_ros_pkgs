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
 * Author: Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_VICTIM_UTILITIES_FEATURE_EXTRACTION_UTILITIES_H
#define PANDORA_VISION_VICTIM_UTILITIES_FEATURE_EXTRACTION_UTILITIES_H

#include <vector>
#include <opencv2/opencv.hpp>

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @class FeatureExtractionUtilities
   * @brief This class provides utilities for feature extraction, feature
   * selection and feature normalization.
   */
  class FeatureExtractionUtilities
  {
    public:
      /**
       * @brief Constructor
       */
      FeatureExtractionUtilities();

      /**
       * @brief Destructor
       */
      ~FeatureExtractionUtilities();

      /**
       * @brief
       * @param newMax [double]
       * @param newMin [double]
       * @param image [cv::Mat*]
       * @param minVec [std::vector<double>*]
       * @param maxVec [std::vector<double>*]
       * @return void
       */
      void findMinMaxParameters(double newMax, double newMin,
          cv::Mat* image, std::vector<double>* minVec,
          std::vector<double>* maxVec);

      /**
       * @brief
       * @param newMax [double]
       * @param newMin [double]
       * @param image [cv::Mat*]
       * @param minVec [std::vector<double>&]
       * @param maxVec [std::vector<double>&]
       * @return void
       */
      void performMinMaxNormalization(double newMax, double newMin,
          cv::Mat* image, const std::vector<double>& minVec,
          const std::vector<double>& maxVec);

      /**
       * @brief
       * @param image [cv::Mat*]
       * @param meanVec [std::vector<double>*]
       * @param stdDevVec [std::vector<double>*]
       * @return void
       */
      void findZScoreParameters(cv::Mat* image,
          std::vector<double>* meanVec, std::vector<double>* stdDevVec);

      /**
       * @brief
       * @param image [cv::Mat*]
       * @param meanVec [std::vector<double>&]
       * @param stdDevVec [std::vector<double>&]
       * @return void
       */
      void performZScoreNormalization(cv::Mat* image,
          const std::vector<double>& meanVec,
          const std::vector<double>& stdDevVec);
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_UTILITIES_FEATURE_EXTRACTION_UTILITIES_H
