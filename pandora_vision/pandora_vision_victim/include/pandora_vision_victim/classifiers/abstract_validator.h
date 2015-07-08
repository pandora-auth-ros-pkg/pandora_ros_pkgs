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

#ifndef PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_VALIDATOR_H
#define PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_VALIDATOR_H

#include <vector>
#include <string>
#include <map>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/feature_extractors/rgb_feature_extraction.h"
#include "pandora_vision_victim/feature_extractors/depth_feature_extraction.h"
#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @class AbstractValidator
   * @brief This class classifies images using a classifier model.
   */
  class AbstractValidator
  {
    public:
      /**
       * @brief Constructor. Initializes abstract classifier parameters.
       * @param nh [const ros::NodeHandle&] The node handle.
       * @param imageType [const std:string&] The type of the images used for
       * classification.
       * @param classifierType [const std::string&] The type of the classifier.
       */
      AbstractValidator(const ros::NodeHandle& nh,
          const std::string& imageType,
          const std::string& classififierType);

      /**
       * @brief Default Destructor.
       */
      virtual ~AbstractValidator();

      /**
       * @brief This function classifies an image and calculates the probability
       * of it belonging to that class.
       * @param inImage [const cv::Mat&] The frame to be processed.
       * @param classLabel [float*] The predicted class label.
       * @param probability [float*] The classification probability.
       * @return void
       */
      void calculatePredictionProbability(const cv::Mat& inImage, float* classLabel, float* probability);

      /**
       * @brief This function classifies an image and calculates the probability
       * of it belonging to that class.
       * @param rgbImage [const cv::Mat&] The rgb frame to be processed.
       * @param depthImage [const cv::Mat&] The depth frame to be processed.
       * @param classLabel [float*] The predicted class label.
       * @param probability [float*] The classification probability.
       * @return void
       */
      void calculatePredictionProbability(const cv::Mat& rgbImage, const cv::Mat& depthImage,
                                          float* classLabel, float* probability);

      /**
       * @brief This function returns the type of the classifier.
       * @return [const std:;string&] The type of the classifier.
       */
      const std::string& getClassifierType(void)
      {
        return classifierType_;
      }

    protected:
      /**
       * @brief This function extracts features according to the predefined
       * feature extraction algorithms.
       * @param inImage [const cv::Mat&] Frame to extract features from.
       * @return void
       */
      void extractFeatures(const cv::Mat& inImage);

      /**
       * @brief This function extracts features according to the predefined
       * feature extraction algorithms.
       * @param inImage [const cv::Mat&] Frame to extract features from.
       * @param imageType [const std::string&] the type of image
       * @return void
       */
      void extractFeatures(const cv::Mat& inImage, const std::string& imageType);

      /**
       * @brief This function predicts the class label and the classification
       * probability of an image's feature vector.
       * @param featuresMat [const cv::Mat&] The feature vector of an image.
       * @param classLabel [float*] The predicted class label.
       * @param probability [float*] The classification probability.
       * @return void
       */
      virtual void predict(const cv::Mat& featuresMat, float* classLabel, float* probability) = 0;

    protected:
      std::string imageType_;

      std::string classifierType_;

      std::string nodeMessagePrefix_;

      /// The path to this node's package.
      std::string packagePath_;
      /// Feature vector
      std::vector<double> featureVector_;

      /// The path to the classifier model.
      std::string classifierPath_;

      /// Variable used to decide the type of feature normalization to perform.
      int typeOfNormalization_;

       std::map<std::string, boost::shared_ptr<FeatureExtraction> > featureExtraction_;
      /// Feature Extraction Utilities used to perform feature normalization
      /// and/or feature selection.
      boost::shared_ptr<FeatureExtractionUtilities> featureExtractionUtilities_;

      /// Vector used for normalization. If z-score normalization is used, this
      /// vector contains mean values. If min-max normalization is used, this
      /// vector contains min values.
      std::vector<double> normalizationParamOneVec_;
      /// Vector used for normalization. If z-score normalization is used, this
      /// vector contains standard values. If min-max normalization is used,
      /// this vector contains max values.
      std::vector<double> normalizationParamTwoVec_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_VALIDATOR_H
