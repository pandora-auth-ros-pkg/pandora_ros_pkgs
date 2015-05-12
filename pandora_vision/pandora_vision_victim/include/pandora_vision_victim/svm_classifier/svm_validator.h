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
#ifndef PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_VALIDATOR_H
#define PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_VALIDATOR_H

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @class SvmValidator
   * @brief This class classifies images using an SVM classifier model.
   */
  class SvmValidator
  {
    public:
      /**
       * @brief Constructor. Initializes SVM classifier parameters and loads
       * classifier model.
       */
      explicit SvmValidator();

      /**
       * @brief Default Destructor.
       */
      virtual ~SvmValidator();

      /**
       * @brief This function extracts features according to the predefined
       * feature extraction algorithms.
       * @param inImage [const cv::Mat&] Frame to extract features from.
       * @return void
       */
      void extractFeatures(const cv::Mat& inImage);

      /**
       * @brief This function extract features according to the
       * predifined features for the rgb image
       * @param inImage [cv::Mat] current rgb frame to be processed
       * @return void
      */
      void calculatePredictionProbability(const cv::Mat& inImage, float* classLabel, float* probability);

      /**
      @brief Function that loads the trained classifier and makes a prediction
      according to the featurevector given for each image
      @return void
      **/
      void predict(float* classLabel, float* prediction);

      /**
       * @brief This function calculates the classification probability
       * according to the SVM prediction.
       * @param prediction [float] The SVM prediction.
       * @return [float] The classification probability.
       */
      float predictionToProbability(float prediction);

    protected:
      /// The path to this node's package.
      std::string packagePath_;
      /// Feature vector
      std::vector<double> featureVector_;

      /// The OpenCV SVM classifier.
      CvSVM svmValidator_;

      /// Parameters for the OpenCV SVM classifier.
      CvSVMParams svmParams_;

      /// The path to the classifier model.
      std::string classifierPath_;

      /// Variables used for the transformation of the classifier prediction to
      /// probabilities.
      double probabilityScaling_;
      double probabilityTranslation_;

      /// Variable used to decide the type of feature normalization to perform.
      int typeOfNormalization_;

      /// Extractor of all the neccesary features for the classification.
      FeatureExtraction* featureExtraction_;

      /// Feature Extraction Utilities used to perform feature normalization
      /// and/or feature selection.
      FeatureExtractionUtilities* featureExtractionUtilities_;

      /// Vector used for normalization. If z-score normalization is used, this
      /// vector contains mean values. If min-max normalization is used, this
      /// vector contains min values.
      std::vector<double> normalizationParamOneVec_;
      /// Vector used for normalization. If z-score normalization is used, this
      /// vector contains standard values. If min-max normalization is used,
      /// this vector contains max values.
      std::vector<double> normalizationParamTwoVec_;
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_VALIDATOR_H
