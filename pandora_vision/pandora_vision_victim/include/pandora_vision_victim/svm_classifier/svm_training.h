/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_TRAINING_H
#define PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_TRAINING_H

#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "pandora_vision_victim/victim_parameters.h"
#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/utilities/file_utilities.h"
#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"

namespace pandora_vision
{
  class SvmTraining
  {
    public:
      /**
       * @brief The Constructor
       */
      SvmTraining(const std::string& ns, int numFeatures,
                  const std::string& datasetPath);

      /**
       * @brief The Destructor
       */
      virtual ~SvmTraining();

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
       * @brief Function that implements the training for the subsystems
       * according to the given training sets. It applies SVM and extracts
       * a suitable model.
       * @return void
       */
      virtual void trainSubSystem()
      {
      }

      /**
       * @brief This function evaluates the classifier model, based on the
       * predicted and the actual class attributes.
       * @param predicted [const cv::Mat&] The predicted class attributes.
       * @param actual [const cv::Mat&] The actual class attributes.
       * @return void
       */
      void evaluate(const cv::Mat& predicted, const cv::Mat& actual);

      // Platt's binary SVM Probablistic Output: an improvement from Lin et al.
      /**
       * @brief Function that computes the vectors A,B necessary for the
       * computation of the probablistic output of the SVM bases on Platt's
       * binary SVM probablistic Output
       * @param dec_values [cv::Mat] the distance from the hyperplane of the
       * predicted results of the given test dataset
       * @param labels [cv::Mat] the true labels of the dataset
       * @param A [double*] the vector A to be computed
       * @param B [double*] the vector B to be computed
       * @return void
       */
      void sigmoid_train(cv::Mat dec_values, cv::Mat labels, double* A,
                         double* B);

    protected:
      /// The NodeHandle
      ros::NodeHandle nh_;

      VictimParameters vparams;

      /// String containing the type of the images used in the feature
      /// extraction process.
      std::string imageType_;

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

      std::string path_to_samples;
      std::string package_path;

      int numFeatures_;
      float accuracy_;
      float precision_;
      float recall_;
      float fmeasure_;

      cv::Mat trainingFeaturesMat_;
      cv::Mat trainingLabelsMat_;
      cv::Mat testFeaturesMat_;
      cv::Mat testLabelsMat_;

      /// Feature Extractor
      FeatureExtraction* featureExtraction_;
      /// Feature Extraction Utilities used to perform feature normalization
      /// and/or feature selection.
      FeatureExtractionUtilities* featureExtractionUtilities_;
      /// Set up SVM's parameters
      //CvSVMParams params;
      CvParamGrid CvParamGrid_gamma, CvParamGrid_C;

      /// Train the SVM
      CvSVM SVM;
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_SVM_CLASSIFIER_SVM_TRAINING_H
