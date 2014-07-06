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
 * Author: Alexandros Philotheou
 *********************************************************************/

#include "hole_fusion_node/hole_validation.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
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
  std::map<int, float> HoleValidation::validateHoles(
    const std::vector<std::vector<float> >& probabilitiesVector2D,
    const int& filteringMode)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateHoles", "processCandidateHoles");
    #endif

    // The map of holes' indices that are valid and
    // their respective validity probability that will be returned
    std::map<int, float> valid;

    switch (Parameters::HoleFusion::Validation::validation_process)
    {
      case VALIDATION_VIA_THRESHOLDING :
        {
          valid = validateHolesViaThresholding(
            probabilitiesVector2D, filteringMode);

          break;
        }
      case VALIDATION_VIA_WEIGHTING :
        {
          valid = validateHolesViaWeighting(
            probabilitiesVector2D, filteringMode);

          break;
        }
      case VALIDATION_VIA_THRESHOLDED_WEIGHTING :
        {
          valid = validateHolesViaThresholdedWeighting(
            probabilitiesVector2D, filteringMode);

          break;
        }
      default:
        {
          ROS_ERROR_NAMED(PKG_NAME,
            "[Hole Fusion node] Invalid validation process selected.");

          break;
        }
    }

    #ifdef DEBUG_TIME
    Timer::tick("validateHoles");
    #endif

    return valid;
  }



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
    @return [std::map<int, float>] The indices of the valid holes and their
    respective validity probabilities
   **/
  std::map<int, float> HoleValidation::validateHolesViaThresholdedWeighting(
    const std::vector<std::vector<float> >& probabilitiesVector2D,
    const int& filteringMode)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateHolesViaThresholdedWeighting",
      "processCandidateHoles");
    #endif

    // The map of holes' indices that are valid and
    // their respective validity probability that will be returned
    std::map<int, float> valid;

    for (int i = 0; i < probabilitiesVector2D[0].size(); i++)
    {
      int exponent = 0;
      float sum = 0.0;

      // Commence setting of priorities given to hole checkers.
      // Each priority given is not fixed,
      // but there is an apparent hierarchy witnessed here.
      // In order to reach a valid conclusion, an analytical method had to be
      // used, which is one analogous to the one presented in
      // {insert link of Manos Tsardoulias's PHD thesis}

      // Apply a weight to each probability according to its weight order.
      // If depth analysis was not possible, use the urgent weight order.
      if (filteringMode == RGBD_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgbd_priority - 1][i] <
            Parameters::Filters::ColourHomogeneity::rgbd_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::ColourHomogeneity::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgbd_priority - 1][i];

          exponent++;
        }

        // Depth / area
        if (Parameters::Filters::DepthArea::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthArea::priority - 1][i] <
            Parameters::Filters::DepthArea::threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::DepthArea::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::DepthArea::priority - 1][i];

          exponent++;
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgbd_priority - 1][i] <
            Parameters::Filters::LuminosityDiff::rgbd_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::LuminosityDiff::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgbd_priority - 1][i];

          exponent++;
        }

        // Outline of rectangle plane constitution
        if (Parameters::Filters::RectanglePlaneConstitution::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::RectanglePlaneConstitution::priority - 1][i] <
            Parameters::Filters::RectanglePlaneConstitution::threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::RectanglePlaneConstitution::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::RectanglePlaneConstitution::priority - 1][i];

          exponent++;
        }

        // Depth homogeneity
        if (Parameters::Filters::DepthHomogeneity::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthHomogeneity::priority - 1][i] <
            Parameters::Filters::DepthHomogeneity::threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::DepthHomogeneity::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::DepthHomogeneity::priority - 1][i];

          exponent++;
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgbd_priority - 1][i] <
            Parameters::Filters::TextureBackprojection::rgbd_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::TextureBackprojection::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgbd_priority - 1][i];

          exponent++;
        }

        // Intermediate points plane constitution
        if (Parameters::Filters::IntermediatePointsPlaneConstitution::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1][i] <
            Parameters::Filters::IntermediatePointsPlaneConstitution::threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1][i];

          exponent++;
        }

        // Depth diff
        if (Parameters::Filters::DepthDiff::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthDiff::priority - 1][i] <
            Parameters::Filters::DepthDiff::threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::DepthDiff::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::DepthDiff::priority - 1][i];

          exponent++;
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgbd_priority - 1][i] <
            Parameters::Filters::TextureDiff::rgbd_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::TextureDiff::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgbd_priority - 1][i];

          exponent++;
        }
      }
      else if (filteringMode == RGB_ONLY_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgb_priority - 1][i] <
            Parameters::Filters::ColourHomogeneity::rgb_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::ColourHomogeneity::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgb_priority - 1][i];

          exponent++;
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgb_priority - 1][i] <
            Parameters::Filters::LuminosityDiff::rgb_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::LuminosityDiff::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgb_priority - 1][i];

          exponent++;
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgb_priority - 1][i] <
            Parameters::Filters::TextureBackprojection::rgb_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::TextureBackprojection::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgb_priority - 1][i];

          exponent++;
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgb_priority - 1][i] <
            Parameters::Filters::TextureDiff::rgb_threshold)
          {
            continue;
          }

          sum +=
            pow(2, Parameters::Filters::TextureDiff::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgb_priority - 1][i];

          exponent++;
        }
      }
      else
      {
        ROS_ERROR_NAMED(PKG_NAME,
          "[Hole Fusion node] Validation process failure");
      }

      // The total validity probability of the i-th hole
      sum /= (pow(2, exponent) - 1);

      // The validity acceptance threshold
      float threshold = 0.0;

      if (filteringMode == RGBD_MODE)
      {
        threshold = Parameters::HoleFusion::Validation::rgbd_validity_threshold;
      }
      else if (filteringMode == RGB_ONLY_MODE)
      {
        threshold = Parameters::HoleFusion::Validation::rgb_validity_threshold;
      }
      else
      {
        ROS_ERROR_NAMED(PKG_NAME,
          "[Hole Fusion node] Validation process failure");
      }

      // If the total validity probability of the i-th hole exceeds a
      // pre-determined threshold, we consider the i-th hole to be valid
      if (sum > threshold)
      {
        valid[i] = sum;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("validateHolesViaThresholdedWeighting");
    #endif

    return valid;
  }



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
    @return [std::map<int, float>] The indices of the valid holes and their
    respective validity probabilities
   **/
  std::map<int, float> HoleValidation::validateHolesViaThresholding(
    const std::vector<std::vector<float> >& probabilitiesVector2D,
    const int& filteringMode)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateHolesViaThresholding", "processCandidateHoles");
    #endif

    // The map of holes' indices that are valid and
    // their respective validity probability that will be returned
    std::map<int, float> valid;

    for (int i = 0; i < probabilitiesVector2D[0].size(); i++)
    {
      float validityProbability = 0.0;

      // RGB + Depth mode. All RGB and Depth active filters produce
      // probabilities that will have to be checked on a per-filter basis,
      // against individually-set thresholds.
      if (filteringMode == RGBD_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgbd_priority - 1][i] <
            Parameters::Filters::ColourHomogeneity::rgbd_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
              Parameters::Filters::ColourHomogeneity::rgbd_priority - 1][i];
          }
        }

        // Depth / area
        if (Parameters::Filters::DepthArea::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthArea::priority - 1][i] <
            Parameters::Filters::DepthArea::threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::DepthArea::priority - 1][i];
          }
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgbd_priority - 1][i] <
            Parameters::Filters::LuminosityDiff::rgbd_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgbd_priority - 1][i];
          }
        }

        // Outline of rectangle plane constitution
        if (Parameters::Filters::RectanglePlaneConstitution::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::RectanglePlaneConstitution::priority - 1][i] <
            Parameters::Filters::RectanglePlaneConstitution::threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::RectanglePlaneConstitution::priority - 1][i];
          }
        }

        // Depth homogeneity
        if (Parameters::Filters::DepthHomogeneity::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthHomogeneity::priority - 1][i] <
            Parameters::Filters::DepthHomogeneity::threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::DepthHomogeneity::priority - 1][i];
          }
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgbd_priority - 1][i] <
            Parameters::Filters::TextureBackprojection::rgbd_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgbd_priority - 1][i];
          }
        }

        // Intermediate points plane constitution
        if (Parameters::Filters::IntermediatePointsPlaneConstitution::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1][i] <
            Parameters::Filters::IntermediatePointsPlaneConstitution::threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1][i];
          }
        }

        // Depth diff
        if (Parameters::Filters::DepthDiff::priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::DepthDiff::priority - 1][i] <
            Parameters::Filters::DepthDiff::threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::DepthDiff::priority - 1][i];
          }
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgbd_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgbd_priority - 1][i] <
            Parameters::Filters::TextureDiff::rgbd_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgbd_priority - 1][i];
          }
        }
      }
      else if (filteringMode == RGB_ONLY_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgb_priority - 1][i] <
            Parameters::Filters::ColourHomogeneity::rgb_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgb_priority - 1][i];
          }
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgb_priority - 1][i] <
            Parameters::Filters::LuminosityDiff::rgb_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgb_priority- 1][i];
          }
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgb_priority- 1][i] <
            Parameters::Filters::TextureBackprojection::rgb_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgb_priority - 1][i];
          }
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgb_priority > 0)
        {
          if (probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgb_priority - 1][i] <
            Parameters::Filters::TextureDiff::rgb_threshold)
          {
            continue;
          }
          else
          {
            validityProbability += probabilitiesVector2D[
            Parameters::Filters::TextureDiff::rgb_priority - 1][i];
          }
        }
      }
      else
      {
        ROS_ERROR_NAMED(PKG_NAME,
          "[Hole Fusion node] Validation process failure");
      }

      // If the i-th candidate hole has passed the above checks,
      // it surely is valid. Its validity probability will amount to the
      // mean value of its separate validity probabilities.
      if (probabilitiesVector2D.size() > 0)
      {
        valid[i] = validityProbability / probabilitiesVector2D.size();
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("validateHolesViaThresholding");
    #endif

    // Return the valid set
    return valid;
  }



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
    @return [std::map<int, float>] The indices of the valid holes and their
    respective validity probabilities
   **/
  std::map<int, float> HoleValidation::validateHolesViaWeighting(
    const std::vector<std::vector<float> >& probabilitiesVector2D,
    const int& filteringMode)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateHolesViaWeighting", "validateHoles");
    #endif

    // The map of holes' indices that are valid and
    // their respective validity probability that will be returned
    std::map<int, float> valid;

    for (int i = 0; i < probabilitiesVector2D[0].size(); i++)
    {
      int exponent = 0;
      float sum = 0.0;

      // Commence setting of priorities given to hole checkers.
      // Each priority given is not fixed,
      // but there is an apparent hierarchy witnessed here.
      // In order to reach a valid conclusion, an analytical method had to be
      // used, which is one analogous to the one presented in
      // {insert link of Manos Tsardoulias's PHD thesis}

      // Apply a weight to each probability according to its weight order.
      // If depth analysis was not possible, use the urgent weight order.
      if (filteringMode == RGBD_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgbd_priority > 0)
        {
          sum += pow(2, Parameters::Filters::ColourHomogeneity::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgbd_priority - 1][i];

          exponent++;
        }

        // Depth / area
        if (Parameters::Filters::DepthArea::priority > 0)
        {
          sum += pow(2, Parameters::Filters::DepthArea::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::DepthArea::priority - 1][i];

          exponent++;
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgbd_priority > 0)
        {
          sum += pow(2, Parameters::Filters::LuminosityDiff::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgbd_priority - 1][i];

          exponent++;
        }

        // Outline of rectangle plane constitution
        if (Parameters::Filters::RectanglePlaneConstitution::priority > 0)
        {
          sum += pow(2, Parameters::Filters::RectanglePlaneConstitution::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::RectanglePlaneConstitution::priority - 1][i];

          exponent++;
        }

        // Depth homogeneity
        if (Parameters::Filters::DepthHomogeneity::priority > 0)
        {
          sum += pow(2, Parameters::Filters::DepthHomogeneity::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::DepthHomogeneity::priority - 1][i];

          exponent++;
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgbd_priority > 0)
        {
          sum += pow(2, Parameters::Filters::TextureBackprojection::rgbd_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgbd_priority - 1][i];

          exponent++;
        }

        // Intermediate points plane constitution
        if (Parameters::Filters::IntermediatePointsPlaneConstitution::priority > 0)
        {
          sum += pow(2, Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::IntermediatePointsPlaneConstitution::priority - 1][i];

          exponent++;
        }

        // Depth diff
        if (Parameters::Filters::DepthDiff::priority > 0)
        {
          sum += pow(2, Parameters::Filters::DepthDiff::priority - 1)
            * probabilitiesVector2D[Parameters::Filters::DepthDiff::priority - 1][i];

          exponent++;
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgbd_priority > 0)
        {
          sum += pow(2, Parameters::Filters::TextureDiff::rgbd_priority - 1)
            * probabilitiesVector2D[Parameters::Filters::TextureDiff::rgbd_priority - 1][i];

          exponent++;
        }
      }
      else if (filteringMode == RGB_ONLY_MODE)
      {
        // Color homogeneity
        if (Parameters::Filters::ColourHomogeneity::rgb_priority > 0)
        {
          sum += pow(2, Parameters::Filters::ColourHomogeneity::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::ColourHomogeneity::rgb_priority - 1][i];

          exponent++;
        }

        // Luminosity diff
        if (Parameters::Filters::LuminosityDiff::rgb_priority > 0)
        {
          sum += pow(2, Parameters::Filters::LuminosityDiff::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::LuminosityDiff::rgb_priority - 1][i];

          exponent++;
        }

        // Texture backprojection
        if (Parameters::Filters::TextureBackprojection::rgb_priority > 0)
        {
          sum += pow(2, Parameters::Filters::TextureBackprojection::rgb_priority - 1)
            * probabilitiesVector2D[
            Parameters::Filters::TextureBackprojection::rgb_priority - 1][i];

          exponent++;
        }

        // Texture diff
        if (Parameters::Filters::TextureDiff::rgb_priority > 0)
        {
          sum += pow(2, Parameters::Filters::TextureDiff::rgb_priority - 1)
            * probabilitiesVector2D[Parameters::Filters::TextureDiff::rgb_priority - 1][i];

          exponent++;
        }
      }
      else
      {
        ROS_ERROR_NAMED(PKG_NAME,
          "[Hole Fusion node] Validation process failure");
      }

      // The total validity probability of the i-th hole
      sum /= (pow(2, exponent) - 1);

      // The validity acceptance threshold
      float threshold = 0.0;

      if (filteringMode == RGBD_MODE)
      {
        threshold = Parameters::HoleFusion::Validation::rgbd_validity_threshold;
      }
      else if (filteringMode == RGB_ONLY_MODE)
      {
        threshold = Parameters::HoleFusion::Validation::rgb_validity_threshold;
      }
      else
      {
        ROS_ERROR_NAMED(PKG_NAME,
          "[Hole Fusion node] Validation process failure");
      }

      // If the total validity probability of the i-th hole exceeds a
      // pre-determined threshold, we consider the i-th hole to be valid
      if (sum > threshold)
      {
        valid[i] = sum;
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("validateHolesViaWeighting");
    #endif

    return valid;
  }

} // namespace pandora_vision
