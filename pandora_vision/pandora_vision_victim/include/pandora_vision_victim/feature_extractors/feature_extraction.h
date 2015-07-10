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
 * Author: Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_FEATURE_EXTRACTION_H
#define PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_FEATURE_EXTRACTION_H

#include <string>
#include <vector>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include "pandora_vision_victim/utilities/bag_of_words_trainer.h"
#include "pandora_vision_victim/feature_extractors/feature_extractor_factory.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @class FeatureExtraction
   * @brief This class extracts features from images.
   */
  class FeatureExtraction
  {
    public:
      /**
       * @brief Default Constructor
       */
      explicit FeatureExtraction(const std::string& classifierType);

      /**
       * @brief Default Destructor
       */
      virtual ~FeatureExtraction();

      /**
       * @brief This function extracts features according to the predefined
       * feature extraction algorithms.
       * @param inImage [const cv::Mat&] Frame to extract features from.
       * @ return void
       */
      virtual void extractFeatures(const cv::Mat& inImage);

      /**
       * @brief
       */
      void addDescriptorsToBagOfWords(const cv::Mat& inImage);

      /**
       * @brief
       */
      void addDescriptorsToBagOfWords(
          const std::vector<cv::Mat>& descriptorsVec);

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
      void constructFeaturesMatrix(
          const boost::filesystem::path& directory,
          const std::string& annotationsFile,
          cv::Mat* featuresMat, cv::Mat* labelsMat);

      /**
       * @brief This function constructs a Bag of Words vocabulary.
       * @param directory [const boost::filesystem::path&] The directory that
       * contains the set of images for the creation of the vocabulary.
       * @param annotationsFile [const std::string&] The name of the file that
       * contains the class attributes and the regions of interest of the
       * images to be processed.
       * @return [bool] Variable declaring whether a bag of words vocabulary is
       * needed and thus constructed or not.
       */
      bool constructBagOfWordsVocabulary(
          const boost::filesystem::path& directory,
          const std::string& annotationsFile);

      /**
       * @brief
       */
      std::vector<double> getFeatureVector() const;

      /**
       * @brief
       */
      std::vector<std::vector<double> > getFeatureMatrix() const;

      /**
       * @brief
       */
      cv::Mat getBagOfWordsVocabulary() const;

      /**
       * @brief
       */
      void setBagOfWordsVocabulary(const cv::Mat& vocabulary);

      /**
       * @brief This function checks whether a Bag of Words vocabulary is
       * needed.
       * @return [bool] Variable declaring whether a bag of words vocabulary is
       * needed or not.
       */
      bool bagOfWordsVocabularyNeeded();

      int getDictionarySize(void)
      {
        return dictionarySize_;
      }

      /**
       * @brief Getter for the number of features
       * @return [int] The number of features extracted.
      */
      int getFeatureNumber();


      /**
       * @brief
       */
      std::vector<cv::Mat> getBagOfWordsDescriptors() const;

    protected:
      /// The number of features used.
      int numFeatures_;

      /// Flag for displaying the calculated descriptors.
      bool visualization_;

      /// Flag indicating whether the image descriptors should be saved or not.
      bool saveDescriptors_;

      /// Flag indicating whether the image descriptors should be loaded or not.
      bool loadDescriptors_;

      /// Dictionary size used for bag of words model.
      int dictionarySize_;

      /// String containing the package path.
      std::string packagePath_;

      /// String indicating the type of the processed images.
      std::string imageType_;

      /// Vector containing the features extracted from a single image.
      std::vector<double> featureVector_;

      /// Matrix containing the feature vectors extracted from a set of images.
      std::vector<std::vector<double> > featureMatrix_;

      /// Vector containing the feature types to be extracted from a set of
      /// images.
      std::map<std::string, bool> chosenFeatureTypesMap_;

      ///
      std::map<std::string, boost::shared_ptr<FeatureExtractorFactory> > featureFactoryPtrMap_;

      ///
      boost::shared_ptr<BagOfWordsTrainer> bowTrainerPtr_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_FEATURE_EXTRACTION_H
