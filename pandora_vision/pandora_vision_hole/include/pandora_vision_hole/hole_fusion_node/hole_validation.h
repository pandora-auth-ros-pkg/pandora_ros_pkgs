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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_VALIDATION_H
#define PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_VALIDATION_H

#include "utils/defines.h"
#include "utils/holes_conveyor.h"
#include "utils/parameters.h"

// The hole's validation process identifiers
#define VALIDATION_VIA_THRESHOLDING 0
#define VALIDATION_VIA_WEIGHTING 1
#define VALIDATION_VIA_THRESHOLDED_WEIGHTING 2

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
  /**
    @class HoleValidation
    @brief Provides methods for validating filter-extracted validity
    probabilities about holes
   **/
  class HoleValidation
  {
    public:
      /**
        @brief Validates candidate holes, meaning that having a two dimensional
        array that is the product of a series of validity ascertainers that
        in their turn produce a probability hinting to the confidence level
        that a particular candidate hole is indeed a hole, this method fuses
        all the probabilities produced by various hole checkers and responds
        affirmatively to the question of the purpose of this package:
        which of the things that the image sensor of the pandora ugv
        locates as potential holes are indeed holes.
        @param[in] probabilitiesVector2D
        [const std::vector<std::vector<float> >&]
        A two dimensional vector containing the probabilities of
        validity of each candidate hole. Each row of it pertains to a specific
        filter applied, each column to a particular hole
        @param[in] filteringMode [const int&] 0 for when Depth and RGB analysis
        is applicable, 1 for when only RGB analysis is applicable
        @return [std::map<int, float>] The indices of the valid holes and their
        respective validity probabilities
       **/
      static std::map<int, float> validateHoles(
        const std::vector<std::vector<float> >& probabilitiesVector2D,
        const int& filteringMode);

      /**
        @brief Validates candidate holes by giving weights to each
        probability from the set of per-hole probabilities set.
        Each weight is a power of two. Two weights shall not have the
        same value for any number of probabilities. The exponent of 2 used
        per weight corresponds to the execution order - weighting order of
        a particular filter.
        @param[in] probabilitiesVector2D
        [const std::vector<std::vector<float> >&]
        A two dimensional vector containing the probabilities of
        validity of each candidate hole. Each row of it pertains to a specific
        filter applied, each column to a particular hole
        @param[in] filteringMode [const int&] 0 for when Depth and RGB analysis
        is applicable, 1 for when only RGB analysis is applicable
        @return [std::map<int, float>] The indices of the valid holes and their
        respective validity probabilities
       **/
      static std::map<int, float> validateHolesViaWeighting(
        const std::vector<std::vector<float> >& probabilitiesVector2D,
        const int& filteringMode);

      /**
        @brief Validates candidate holes by giving weights to each
        probability from the set of per-hole probabilities set.
        Additionally, each probability obtained is compared against
        individually-set thresholds per source of probability.
        Altough, theoretically, all probabilities are set in the [0, 1]
        interval, not all can reach the value 1 in practice and individual
        thresholds have to be empirically set. Each hole, if determined valid,
        is assigned a validity probability equal to the mean of the set
        of its corresponding probabilities.
        Each weight is a power of two. Two weights shall not have the
        same value for any number of probabilities. The exponent of 2 used
        per weight corresponds to the execution order - weighting order of
        a particular filter.
        @param[in] probabilitiesVector2D
        [const std::vector<std::vector<float> >&]
        A two dimensional vector containing the probabilities of
        validity of each candidate hole. Each row of it pertains to a specific
        filter applied, each column to a particular hole
        @param[in] filteringMode [const int&] 0 for when Depth and RGB analysis
        is applicable, 1 for when only RGB analysis is applicable
        @return [std::map<int, float>] The indices of the valid holes and their
        respective validity probabilities
       **/
      static std::map<int, float> validateHolesViaThresholdedWeighting(
        const std::vector<std::vector<float> >& probabilitiesVector2D,
        const int& filteringMode);

      /**
        @brief Validates candidate holes by checking each set of
        probabilities obtained against individually-set thresholds
        per source of probability.
        Altough, theoretically, all probabilities are set in the [0, 1]
        interval, not all can reach the value 1 in practice and individual
        thresholds have to be empirically set. Each hole, if determined valid,
        is assigned a validity probability equal to the mean of the set
        of its corresponding probabilities.
        @param[in] probabilitiesVector2D
        [const std::vector<std::vector<float> >&]
        A two dimensional vector containing the probabilities of
        validity of each candidate hole. Each row of it pertains to a specific
        filter applied, each column to a particular hole
        @param[in] filteringMode [const int&] 0 for when Depth and RGB analysis
        is applicable, 1 for when only RGB analysis is applicable
        @return [std::map<int, float>] The indices of the valid holes and their
        respective validity probabilities
       **/
      static std::map<int, float> validateHolesViaThresholding(
        const std::vector<std::vector<float> >& probabilitiesVector2D,
        const int& filteringMode);
  };

}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_HOLE_FUSION_NODE_HOLE_VALIDATION_H
