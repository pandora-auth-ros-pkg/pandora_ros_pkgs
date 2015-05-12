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
*   Kofinas Miltiadis <mkofinas@gmail.com>
*   Protopapas Marios <protopapas_marios@hotmail.com>
*********************************************************************/

#include <string>

#include <ros/console.h>

#include "pandora_vision_victim/svm_classifier/depth_svm_validator.h"
#include "pandora_vision_victim/victim_parameters.h"
#include "pandora_vision_victim/feature_extractors/depth_feature_extraction.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Constructor. Initializes SVM classifier parameters and loads
   * classifier model. The classifier is to be used with Depth images.
   * @param classifierPath [const std::string&] The path to the classifier
   * model.
   */
  DepthSvmValidator::DepthSvmValidator(const VictimParameters& params)
    : SvmValidator()
  {
    ROS_INFO("Enter Depth SVM Validator");
    classifierPath_ = params.depth_classifier_path;

    ROS_INFO_STREAM(params.depth_classifier_path.c_str());
    svmValidator_.load(params.depth_classifier_path.c_str());
    
    probabilityScaling_ = params.depth_svm_prob_scaling;
    probabilityTranslation_ = params.depth_svm_prob_translation;

    svmParams_.C = params.depth_svm_C;
    svmParams_.gamma = params.depth_svm_gamma;

    std::string filesDirectory = packagePath_ + "/data/";

    std::string normalizationParamOne = "depth_mean_values.xml";
    std::stringstream normalizationParamOnePath;
    normalizationParamOnePath << filesDirectory << normalizationParamOne;
    std::string normalizationParamTwo = "depth_standard_deviation_values.xml";
    std::stringstream normalizationParamTwoPath;
    normalizationParamTwoPath << filesDirectory << normalizationParamTwo;

    normalizationParamOneVec_ = file_utilities::loadFiles(
        normalizationParamOnePath.str(), "mean");
    normalizationParamTwoVec_ = file_utilities::loadFiles(
        normalizationParamTwoPath.str(), "std_dev");

    std::string paramFile = packagePath_ + "/config/depth_svm_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    typeOfNormalization_ = static_cast<int>(fs["type_of_normalization"]);

    fs.release();

    featureExtraction_ = new DepthFeatureExtraction();

    bool vocabularyNeeded = featureExtraction_->bagOfWordsVocabularyNeeded();
    if (vocabularyNeeded)
    {
      const std::string bagOfWordsFile = "depth_bag_of_words.xml";
      const std::string bagOfWordsFilePath = filesDirectory + bagOfWordsFile;
      cv::Mat vocabulary = file_utilities::loadFiles(bagOfWordsFilePath,
          "bag_of_words");
      featureExtraction_->setBagOfWordsVocabulary(vocabulary);
    }
    ROS_INFO("Initialized Depth SVM Validator");
  }

  /**
   * @brief Default Destructor.
   */
  DepthSvmValidator::~DepthSvmValidator()
  {
  }
}  // namespace pandora_vision
