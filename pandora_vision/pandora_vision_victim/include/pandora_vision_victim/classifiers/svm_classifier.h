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

#ifndef PANDORA_VISION_VICTIM_CLASSIFIERS_SVM_CLASSIFIER_H
#define PANDORA_VISION_VICTIM_CLASSIFIERS_SVM_CLASSIFIER_H

#include <string>

#include <opencv2/opencv.hpp>

#include "pandora_vision_victim/classifiers/abstract_classifier.h"
#include "pandora_vision_victim/utilities/platt_scaling.h"
#include "pandora_vision_victim/utilities/principal_component_analysis.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class SvmClassifier : public AbstractClassifier
  {
    public:
      /**
       * @brief The Constructor
       */
      SvmClassifier(const ros::NodeHandle& nh, const std::string& datasetPath,
          const std::string& classifierType,
          const std::string& imageType);

      /**
       * @brief The Destructor
       */
      virtual ~SvmClassifier();

      /**
       * @brief Trains the corresponding classifier using the input features and training labels.
       * @param trainingSetFeatures[const cv::Mat&] The matrix containing the features that describe the
       * training set
       * @param trainingSetLabels[const cv::Mat&] The corresponding labels that define the class of each
       * training sample.
       * @param classifierFileDest[const std::string&] The file where the classifier will be stored.
       * @return bool True on successfull completions, false otherwise.
       */
      virtual bool train(const cv::Mat& trainingSetFeatures, const cv::Mat& trainingSetLabels,
          const std::string& classifierFileDest);

      /**
       * @brief Validates the resulting classifier using the given features
       * extracted from the test set.
       * @param testSetFeatures[const cv::Mat&] The test set features matrix
       * @param validationResults[cv::Mat*] The results for the test set.
       * @return void
       */
      virtual void validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults);

      /**
       * @brief Saves the classifier to a file
       * @param classifierFile[const std::string&] The name of the file where the classifier will be stored
       * @return void
       */
      virtual void load(const std::string& classifierFile)
      {
        classifierPtr_->load(classifierFile.c_str());
      }

    private:
      /// SVM parameters
      CvSVMParams svmParams_;
      CvParamGrid CvParamGrid_gamma, CvParamGrid_C;

      bool autoTrain_;

      bool usePlattScaling_;

      /// The Pointer to the classifier object
      boost::shared_ptr<CvSVM> classifierPtr_;

      /// The pointers to the Platt Scaling object.
      boost::shared_ptr<PlattScaling> plattScalingPtr_;

      /// The pointer to the PCA object.
      boost::shared_ptr<PrincipalComponentAnalysis> pcaPtr_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision

#endif  // PANDORA_VISION_VICTIM_CLASSIFIERS_SVM_CLASSIFIER_H
