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
*********************************************************************/

#include <cmath>
#include <string>

#include <ros/console.h>

#include "pandora_vision_victim/classifiers/abstract_validator.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  AbstractValidator::AbstractValidator(const ros::NodeHandle& nh,
      const std::string& imageType,
      const std::string& classifierType)
  {
    nodeMessagePrefix_ = "[PANDORA_VISION_VICTIM_" + boost::to_upper_copy<std::string>(imageType)
        + "_" + boost::to_upper_copy<std::string>(classifierType) + "]";

    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Creating Abstract Validator instance");

    imageType_ = imageType;
    classifierType_ = classifierType;

    std::string classifierPath;
    if (!nh.getParam(classifierType_ + "/" + imageType_ + "/classifier_path", classifierPath))
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Could not retrieve "
          << "the classifier path parameter!");
      ROS_BREAK();
    }

    packagePath_ = ros::package::getPath("pandora_vision_victim");

    classifierPath_ = packagePath_ + classifierPath;
    ROS_INFO_STREAM(classifierPath_);

    std::string paramFile = packagePath_ + "/config/" + imageType + "_victim_training_params.yaml";
    cv::FileStorage fs(paramFile, cv::FileStorage::READ);
    fs.open(paramFile, cv::FileStorage::READ);

    std::cout << paramFile << std::endl;
    if (!fs.isOpened())
    {
      ROS_ERROR("Could not open : %s !", paramFile.c_str());
      ROS_ERROR("The validation node will now shut down!");
      ROS_BREAK();
    }

    cv::FileNode classifierNode = fs[classifierType_];
    typeOfNormalization_ = static_cast<int>(classifierNode["type_of_normalization"]);

    fs.release();

    std::string filesDirectory = packagePath_ + "/data/";
    if (typeOfNormalization_ == 1)
    {
      std::string normalizationParamOne = imageType_ + "_" + classifierType_ + "_min_values.xml";
      std::string normalizationParamOnePath = filesDirectory + normalizationParamOne;
      std::string normalizationParamTwo = imageType_ + "_" + classifierType_ + "_max_values.xml";
      std::string normalizationParamTwoPath = filesDirectory + normalizationParamTwo;

      std::string normalizationParamOneTag = "min";
      std::string normalizationParamTwoTag = "max";
      normalizationParamOneVec_ = file_utilities::loadFiles(
          normalizationParamOnePath, normalizationParamOneTag);
      normalizationParamTwoVec_ = file_utilities::loadFiles(
          normalizationParamTwoPath, normalizationParamTwoTag);
    }
    else if (typeOfNormalization_ == 2)
    {
      std::string normalizationParamOne = imageType_ + "_" + classifierType_ + "_mean_values.xml";
      std::string normalizationParamOnePath = filesDirectory + normalizationParamOne;
      std::string normalizationParamTwo = imageType_ + "_" + classifierType_ + "_standard_deviation_values.xml";
      std::string normalizationParamTwoPath = filesDirectory + normalizationParamTwo;

      std::string normalizationParamOneTag = "mean";
      std::string normalizationParamTwoTag = "std_dev";
      normalizationParamOneVec_ = file_utilities::loadFiles(
          normalizationParamOnePath, normalizationParamOneTag);
      normalizationParamTwoVec_ = file_utilities::loadFiles(
          normalizationParamTwoPath, normalizationParamTwoTag);
    }

    if (boost::iequals(imageType_ , "rgb"))
    {
      featureExtraction_ = new RgbFeatureExtraction(classifierType_);
    }
    else if (boost::iequals(imageType_, "depth"))
    {
      featureExtraction_ = new DepthFeatureExtraction(classifierType_);
    }
    else
    {
      ROS_ERROR_STREAM(nodeMessagePrefix_ << ": Wrong image type! "
          << "Cannot implement validator!");
      ROS_BREAK();
    }

    bool vocabularyNeeded = featureExtraction_->bagOfWordsVocabularyNeeded();
    if (vocabularyNeeded)
    {
      const std::string bagOfWordsFile = imageType_ + "_" + classifierType_ + "_bag_of_words.xml";
      const std::string bagOfWordsFilePath = filesDirectory + bagOfWordsFile;
      cv::Mat vocabulary = file_utilities::loadFiles(bagOfWordsFilePath,
          "bag_of_words");
      featureExtraction_->setBagOfWordsVocabulary(vocabulary);
    }
    featureExtractionUtilities_ = new FeatureExtractionUtilities();
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Initialized Abstract Validator instance");
  }

  AbstractValidator::~AbstractValidator()
  {
  }

  void AbstractValidator::extractFeatures(const cv::Mat& inImage)
  {
    featureExtraction_->extractFeatures(inImage);
  }

  void AbstractValidator::calculatePredictionProbability(const cv::Mat& inImage,
      float* classLabel, float* probability)
  {
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Extracting features");
    extractFeatures(inImage);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Extracted Features");
    if (!featureVector_.empty())
      featureVector_.clear();
    featureVector_ = featureExtraction_->getFeatureVector();

    cv::Mat featuresMat = cv::Mat(featureVector_);
    // Make features matrix a row vector.
    if (featuresMat.cols == 1)
      transpose(featuresMat, featuresMat);
    /// Normalize the data
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Normalize features");
    if (typeOfNormalization_ == 1)
    {
      double newMin = -1.0;
      double newMax = 1.0;
      featureExtractionUtilities_->performMinMaxNormalization(newMax, newMin,
          &featuresMat, normalizationParamOneVec_, normalizationParamTwoVec_);
    }
    else if (typeOfNormalization_ == 2)
    {
      featureExtractionUtilities_->performZScoreNormalization(
          &featuresMat, normalizationParamOneVec_, normalizationParamTwoVec_);
    }

    featuresMat.convertTo(featuresMat, CV_32FC1);

    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Predict image class and probability");
    predict(featuresMat, classLabel, probability);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Class Label = " << *classLabel);
    ROS_INFO_STREAM(nodeMessagePrefix_ << ": Probability = " << *probability);
  }
}  // namespace pandora_vision
