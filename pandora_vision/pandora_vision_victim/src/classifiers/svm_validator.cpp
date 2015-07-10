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

#include "pandora_vision_victim/classifiers/svm_validator.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor. Initializes SVM classifier parameters and loads
   * classifier model.
   * @param classifierPath [const std::string&] The path to the classifier
   * model.
   */
  SvmValidator::SvmValidator(const ros::NodeHandle& nh,
      const std::string& imageType,
      const std::string& classifierType)
      : AbstractValidator(nh, imageType, classifierType)
  {
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Creating " << imageType
        << " " << classifierType << " Validator instance");

    svmValidator_.load(classifierPath_.c_str());

    double probabilityScaling;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/probability_scaling", probabilityScaling))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve"
          << " the probability scaling parameter!");
      ROS_BREAK();
    }

    double probabilityTranslation;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/probability_translation", probabilityTranslation))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the probability translation parameter!");
      ROS_BREAK();
    }

    probabilityScaling_ = probabilityScaling;
    probabilityTranslation_ = probabilityTranslation;

    std::string paramFile = packagePath_ + "/config/" + imageType + "_victim_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    std::cout << paramFile << std::endl;
    if (!fs.isOpened())
    {
      ROS_ERROR("Could not open : %s !", paramFile.c_str());
      ROS_ERROR("The training node will now shut down!");
      ROS_BREAK();
    }

    cv::FileNode classifierNode = fs[classifierType_];
    std::string usePlattScaling = classifierNode["use_platt_scaling"];
    usePlattScaling_ = usePlattScaling.compare("true") == 0;

    std::string doPcaAnalysis = classifierNode["do_pca_analysis"];
    doPcaAnalysis_ = doPcaAnalysis.compare("true") == 0;

    fs.release();

    if (usePlattScaling_)
    {
      plattScalingPtr_.reset(new PlattScaling());
      std::string plattParametersFile = packagePath_ + "/data/" + imageType_ +
          "_" + classifierType_ + "_platt_scaling.xml";
      plattScalingPtr_->load(plattParametersFile);
    }

    if (doPcaAnalysis_)
    {
      pcaPtr_.reset(new PrincipalComponentAnalysis());
      std::string pcaParametersFile = packagePath_ + "/data/" + imageType_ +
          "_" + classifierType_ + "_pca_parameters.xml";
      pcaPtr_->load(pcaParametersFile);
    }

    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Initialized " << imageType_ << " "
        << classifierType_ << " Validator instance");
  }

  /**
   * @brief Default Destructor.
   */
  SvmValidator::~SvmValidator()
  {
  }

  /**
   * @brief Function that loads the trained classifier and makes a prediction
   * according to the feature vector given for each image
   * @return void
   */
  void SvmValidator::predict(const cv::Mat& featuresMat,
      float* classLabel, float* probability)
  {
    cv::Mat projectedFeaturesMat;
    if (doPcaAnalysis_)
    {
      pcaPtr_->project(featuresMat, &projectedFeaturesMat);
    }
    else
    {
      projectedFeaturesMat = featuresMat.clone();
    }

    *classLabel = svmValidator_.predict(projectedFeaturesMat, false);
    float prediction;
    prediction = svmValidator_.predict(projectedFeaturesMat, true);

    *probability = transformPredictionToProbability(prediction, *classLabel);
  }

  /**
   * @brief This function calculates the classification probability according to
   * the classifier prediction.
   * @param prediction [float] The classifier prediction.
   * @return [float] The classification probability.
   */
  float SvmValidator::transformPredictionToProbability(float prediction,
      float classLabel)
  {
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Prediction = " << prediction);

    double absPrediction = fabs(prediction);
    float probability;
    if (usePlattScaling_)
    {
      ROS_INFO_STREAM(nodeMessagePrefix_ << ": Use Platt Scaling to estimate probability");
      probability = plattScalingPtr_->sigmoidPredict(absPrediction * classLabel);
    }
    else
    {
      // Normalize probability to [-1,1]
      probability = static_cast<float>(std::tanh(
          probabilityScaling_ * absPrediction - probabilityTranslation_));
      // Normalize probability to [0,1]
      probability = (1.0f + probability) / 2.0f;
    }
    return probability;
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
