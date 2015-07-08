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

#ifndef PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_CLASSIFIER_H
#define PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_CLASSIFIER_H

#include <string>
#include <map>

#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"


namespace pandora_vision
{
namespace pandora_vision_victim
{
  class AbstractClassifier
  {
    public:
      /**
       * @brief The Constructor
       */
      AbstractClassifier(const ros::NodeHandle& nh, const std::string& datasetPath,
          const std::string& classifierType, const std::string& imageType);

      /**
       * @brief The Destructor
       */
      virtual ~AbstractClassifier();

      /**
       * @brief Function that implements the training for the subsystems
       * according to the given training sets. It applies a classification
       * algorithm and extracts a suitable model.
       * @return void
       */
      virtual void trainAndValidate();

      /**
       * @brief This function normalizes the features and saves normalization
       * parameters in a file.
       * @param featuresMatrix [cv::Mat*] The features matrix to be normalized.
       */
      void normalizeFeaturesAndSaveNormalizationParameters(
          cv::Mat* featuresMatrix);

      /**
       * @brief This function loads normalization parameters and normalizes the
       * input features matrix.
       * @param featuresMatrix [cv::Mat*] The features matrix to be normalized.
       */
      void loadNormalizationParametersAndNormalizeFeatures(
          cv::Mat* featuresMatrix);

      /**
       * @brief
       */
      bool constructBagOfWordsVocabulary(
          const boost::filesystem::path& directory,
          const std::string& annotationsFile);

      /**
       * @brief This function constructs the features matrix, i.e. the feature
       * vectors of a set of images.
       * @param directory [const boost::filesystem::path&] The directory that
       * contains the set of images for the feature extraction.
       * @param annotationsFile [const std::string&] The name of the file that
       * contains the class attributes of the images to be processed.
       * @param featuresMat [cv::Mat*] The features matrix.
       * @param labelsMat [cv::Mat*] The matrix that contains the class
       * attributes for the processed set of images.
       * @return void
       */
      void constructFeaturesMatrix(const boost::filesystem::path& directory,
                                   const std::string& annotationsFile,
                                   cv::Mat* featuresMat, cv::Mat* labelsMat);

      /**
       * @brief This function evaluates the classifier model, based on the
       * predicted and the actual class attributes.
       * @param predicted [const cv::Mat&] The predicted class attributes.
       * @param actual [const cv::Mat&] The actual class attributes.
       * @return void
       */
      void evaluate(const cv::Mat& predicted, const cv::Mat& actual);

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
          const std::string& classifierFileDest) = 0;

      /**
       * @brief Validates the resulting classifier using the given features
       * extracted from the test set.
       * @param testSetFeatures[const cv::Mat&] The test set features matrix
       * @param validationResults[cv::Mat*] The results for the test set.
       * @return void
       */
      virtual void validate(const cv::Mat& testSetFeatures, cv::Mat* validationResults) = 0;

      /**
       * @brief Saves the classifier to a file
       * @param classifierFile[const std::string&] The name of the file where the classifier will be stored
       * @return void
       */
      virtual void load(const std::string& classifierFile) = 0;

    protected:
      /// String used in node messages.
      std::string nodeMessagePrefix_;

      /// String containing the type of the images used in the feature
      /// extraction process.
      std::string imageType_;

      /// String indicating the type of the classifier.
      std::string classifierType_;

      /// Variable used for State Managing
      bool trainingNowON;

      /// Variable used to decide whether to perform the feature extraction for
      /// the training set or to read features from a file.
      bool trainingSetFeatureExtraction_;

      /// Variable used to decide whether to perform the feature extraction for
      /// the test set or to read features from a file.
      bool testSetFeatureExtraction_;

      /// Variable used to decide whether to train a classifier or to load the
      /// classifier model from a file.
      bool loadClassifierModel_;

      /// Variable used to decide whether to perform PCA analysis on the
      /// features or not.
      bool doPcaAnalysis_;

      /// Variable used to decide whether to perform feature normalization or
      /// not and what type of normalization should be chosen. 0 stands for no
      /// normalization, 1 stands for min-max normalization and 2 stands for
      /// z-Score normalization.
      int typeOfNormalization_;

      std::string datasetPath_;
      std::string packagePath_;

      int numFeatures_;
      float accuracy_;
      float precision_;
      float recall_;
      float fmeasure_;

      cv::Mat trainingFeaturesMat_;
      cv::Mat trainingLabelsMat_;
      cv::Mat testFeaturesMat_;
      cv::Mat testLabelsMat_;

      /// The directory in which the necessary files will be stored or retrieved
      /// from.
      std::string filesDirectory_;

      /// String indicating the file in which to save the extracted training set
      /// features or from which to load them.
      std::string trainingFeaturesMatrixFile_;

      /// String indicating the file in which to save the extracted test set
      /// features or from which to load them.
      std::string testFeaturesMatrixFile_;

      /// String indicating the file in which to save the extracted training set
      /// class attributes or from which to load them.
      std::string trainingLabelsMatrixFile_;

      /// String indicating the file in which to save the extracted test set class
      /// attributes or from which to load them.
      std::string testLabelsMatrixFile_;

      /// String indicating the file in which to save the classification algorithm
      /// results.
      std::string resultsFile_;

      /// String indicating the file of the classifier model built through the
      /// training process.
      std::string classifierFile_;

      /// The path to the training set images.
      boost::filesystem::path trainingDirectory_;

      /// The name of the annotations file for the training set.
      std::string trainingAnnotationsFile_;

      /// The path to the test set images.
      boost::filesystem::path testDirectory_;

      /// The name of the annotations file for the test set.
      std::string testAnnotationsFile_;

      /// Feature Extractor
      std::map<std::string, boost::shared_ptr<FeatureExtraction> > featureExtraction_;
      /// Feature Extraction Utilities used to perform feature normalization
      /// and/or feature selection.
      boost::shared_ptr<FeatureExtractionUtilities> featureExtractionUtilities_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CLASSIFIERS_ABSTRACT_CLASSIFIER_H
