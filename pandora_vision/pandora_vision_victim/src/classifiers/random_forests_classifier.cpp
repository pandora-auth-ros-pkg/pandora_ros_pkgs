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

#include <string>

#include <ros/ros.h>

#include "pandora_vision_victim/classifiers/random_forests_classifier.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor for the Random Forests Classifier Wrapper Class.
   * @param ns [const std::string&] The namespace of the node.
   * @param numFeatures [int] The number of input features to the classifier.
   * @param datasetPath [const std::string&] The path to the training dataset.
   * @param classifierType[const std::string&] The type of the classifier.
   * @param imageType[const std::string&] The type of input images given to
   * the classifier (RGB or Depth).
   */
  RandomForestsClassifier::RandomForestsClassifier(const ros::NodeHandle& nh,
      const std::string& datasetPath,
      const std::string& classifierType,
      const std::string& imageType)
      : AbstractClassifier(nh, datasetPath, classifierType, imageType)
  {
    ROS_INFO_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Creating " << imageType_
        << " Random Forests training instance");

    int maxDepth;
    if (!nh.getParam(imageType_ + "/max_depth", maxDepth))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the max depth parameter for Random Forests!");
      maxDepth = 10;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << maxDepth);
    }

    int minSampleCount;
    if (!nh.getParam(imageType_ + "/min_sample_count", minSampleCount))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the min sample count parameter for Random Forests!");
      minSampleCount = 10;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << minSampleCount);
    }

    double regressionAccuracy;
    if (!nh.getParam(imageType_ + "/regression_accuracy", regressionAccuracy))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the regression accuracy parameter for Random Forests!");
      regressionAccuracy = 0.0;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << regressionAccuracy);
    }

    bool useSurrogates;
    if (!nh.getParam(imageType_ + "/use_surrogates", useSurrogates))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the surrogates parameter for Random Forests!");
      useSurrogates = false;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << useSurrogates);
    }

    int maxCategories;
    if (!nh.getParam(imageType_ + "/max_categories", maxCategories))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the max categories parameter for Random Forests!");
      maxCategories = 15;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << maxCategories);
    }

    bool calcVarImportance;
    if (!nh.getParam(imageType_ + "/calc_var_importance", calcVarImportance))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << " the calculate variable importance parameter for Random Forests!");
      calcVarImportance = true;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << calcVarImportance);
    }

    int nactiveVars;
    if (!nh.getParam(imageType_ + "/nactive_vars", nactiveVars))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the number of active variables parameter for Random Forests!");
      nactiveVars = 4;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << nactiveVars);
    }

    int maxNumOfTreesInTheForest;
    if (!nh.getParam(imageType_ + "/max_num_of_trees_in_the_forest", maxNumOfTreesInTheForest))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the max number of trees parameter for Random Forests!");
      maxNumOfTreesInTheForest = 1000;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << maxNumOfTreesInTheForest);
    }

    double forestAccuracy;
    if (!nh.getParam(imageType_ + "/forest_accuracy", forestAccuracy))
    {
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Could not retrieve "
          << "the forest accuracy parameter for Random Forests!");
      forestAccuracy = 0.001;
      ROS_DEBUG_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Setting variable value to "
          << forestAccuracy);
    }

    // const float priors[] = {1, 1};
    randomForestsParams_ = CvRTParams(maxDepth, minSampleCount,
                                      static_cast<float>(regressionAccuracy), useSurrogates,
                                      maxCategories, 0, calcVarImportance, nactiveVars,
                                      maxNumOfTreesInTheForest, static_cast<float>(forestAccuracy),
                                      CV_TERMCRIT_ITER + CV_TERMCRIT_EPS);

    // Initialize the pointer to the Random Forests Classifier object.
    classifierPtr_.reset(new CvRTrees());

    ROS_INFO_STREAM("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Successfully created "
        << imageType_ << " " << classifierType_ << " classifier instance");
  }

  /**
   * @brief Destructor
   */
  RandomForestsClassifier::~RandomForestsClassifier()
  {
    ROS_DEBUG("[PANDORA_VISION_VICTIM_RANDOM_FORESTS]: Destroying Random Forests training instance");
  }

  /**
   * @brief Trains the corresponding classifier using the input features and training labels.
   * @param trainingFeatures[const cv::Mat&] The matrix containing the features that describe the
   * training set
   * @param trainingLabels[const cv::Mat&] The corresponding labels that define the class of each
   * training sample.
   * @param classifierFileDest[const std::string&] The file where the classifier will be stored.
   * @return bool True on successfull completions, false otherwise.
   */
  bool RandomForestsClassifier::train(const cv::Mat& trainingSetFeatures, const cv::Mat& trainingSetLabels,
      const std::string& classifierFileDest)
  {
    if (trainingSetFeatures.empty())
    {
      ROS_ERROR("The features matrix is empty!");
      return false;
    }
    if (trainingSetLabels.empty())
    {
      ROS_ERROR("The labels matrix is empty!");
      return false;
    }

    classifierPtr_->train(trainingSetFeatures, CV_ROW_SAMPLE, trainingSetLabels,
          cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat(), randomForestsParams_);
    classifierPtr_->save(classifierFileDest.c_str());
    return true;
  }

  /**
   * @brief Validates the resulting classifier using the given features
   * extracted from the test set.
   * @param testSetFeatures[const cv::Mat&] The test set features matrix
   * @param validationResults[cv::Mat*] The results for the test set.
   * @return void
   */
  void RandomForestsClassifier::validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults)
  {
    for (int ii = 0; ii < testSetFeatures.rows; ii++)
      validationResults->at<float>(ii) = classifierPtr_->predict(testSetFeatures.row(ii));
    return;
  }

}  // namespace pandora_vision_victim
}  // namespace pandora_vision

