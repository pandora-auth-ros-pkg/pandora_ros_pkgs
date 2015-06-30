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

#include <vector>
#include <string>

#include "pandora_vision_victim/feature_extractors/feature_extraction.h"
#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Default Constructor
   */
  FeatureExtraction::FeatureExtraction(const std::string& classifierType)
  {
    if (!featureVector_.empty())
      featureVector_.clear();
    if (!featureMatrix_.empty())
      featureMatrix_.clear();
    if (!chosenFeatureTypesMap_.empty())
      chosenFeatureTypesMap_.clear();

    packagePath_ = ros::package::getPath("pandora_vision_victim");
  }

  /**
   * @brief Default Destructor
   */
  FeatureExtraction::~FeatureExtraction()
  {
  }

  /**
   * @brief This function extracts features according to the predefined
   * feature extraction algorithms.
   * @param inImage [const cv::Mat&] Frame to extract features from.
   * @return void
   */
  void FeatureExtraction::extractFeatures(const cv::Mat& inImage)
  {
  }

  /**
   * @brief
   */
  void FeatureExtraction::addDescriptorsToBagOfWords(const cv::Mat& inImage)
  {
    if (chosenFeatureTypesMap_["sift"] == true)
    {
      cv::Mat descriptors;
      featureFactoryPtrMap_["sift"]->extractFeatures(inImage, &descriptors);
      std::cout << "SIFT descriptors : " << descriptors.rows << std::endl;
      if (!descriptors.empty())
        bowTrainerPtr_->addDescriptors(descriptors);
    }
  }

  /**
   * @brief
   */
  void FeatureExtraction::addDescriptorsToBagOfWords(
      const std::vector<cv::Mat>& descriptorsVec)
  {
    if (chosenFeatureTypesMap_["sift"] == true)
    {
      for (int ii = 0; ii < descriptorsVec.size(); ii++)
      {
        if (!descriptorsVec[ii].empty())
          bowTrainerPtr_->addDescriptors(descriptorsVec[ii]);
      }
    }
  }

  /**
   * @brief This function constructs the features matrix, i.e. the feature
   * vectors of a set of images.
   * @param directory [const boost::filesystem::path&] The directory that
   * contains the set of images for the feature extraction.
   * @param annotationsFile [const std::string&] The name of the file that
   * contains the class attributes of the images to be processed.
   * @param featuresMat [cv::Mat*] The features matrix.
   * @param labelsMat [cv::Mat*] The matrix that contains the class attributes
   * for the processed set of images.
   * @return void
   */
  void FeatureExtraction::constructFeaturesMatrix(
      const boost::filesystem::path& directory,
      const std::string& annotationsFile,
      cv::Mat* featuresMat, cv::Mat* labelsMat)
  {
    cv::Mat image, imageROI;
    std::vector<std::string> annotatedImages;
    std::vector<cv::Rect> boundingBox;
    std::vector<int> classAttributes;

    bool successfulFileLoad = file_utilities::loadAnnotationsFromFile(
        annotationsFile, &boundingBox, &annotatedImages, &classAttributes);
    std::cout << "Iterate over dataset images" << std::endl;

    if (successfulFileLoad)
    {
      std::cout << "Read class attributes from annotation file." << std::endl;
      for (int ii = 0; ii < classAttributes.size(); ii++)
        labelsMat->at<double>(ii) =  classAttributes[ii];

      for (int ii = 0; ii < annotatedImages.size(); ii++)
      {
        std::string imageAbsolutePath = directory.string() + "/" + annotatedImages[ii];
        image = cv::imread(imageAbsolutePath);
        if (!image.data)
        {
          std::cout << "Error reading file " << annotatedImages[ii] << std::endl;
          continue;
        }

        imageROI = image(boundingBox[ii]);
        extractFeatures(imageROI);

        std::cout << "Feature vector of image " << annotatedImages[ii]
                  << ": Size = " << featureVector_.size()
                  << ", Class = " << classAttributes[ii] << std::endl;

        for (int jj = 0; jj < featureVector_.size(); jj++)
          featuresMat->at<double>(ii, jj) = featureVector_[jj];
      }
    }
    else
    {
      std::cout << "Unsuccessful annotations file load. Exiting!";
      exit(1);
    }
  }

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
  bool FeatureExtraction::constructBagOfWordsVocabulary(
      const boost::filesystem::path& directory,
      const std::string& annotationsFile)
  {
    if (!chosenFeatureTypesMap_["sift"])
      return false;
    std::cout << "Constructing Bag Of Words Vocabulary" << std::endl;

    cv::Mat image, imageROI;
    std::vector<std::string> annotatedImages;
    std::vector<cv::Rect> boundingBox;
    std::vector<int> classAttributes;

    bool successfulFileLoad = file_utilities::loadAnnotationsFromFile(
        annotationsFile, &boundingBox, &annotatedImages, &classAttributes);

    std::cout << "Number of annotated images: " << annotatedImages.size()
              << std::endl;

    std::vector<cv::Mat> descriptorsVec;
    const std::string descriptorsFile = imageType_ + "sift_descriptors.xml";
    const std::string descriptorsFilePath = packagePath_ + "/data/" +
                                            descriptorsFile;

    std::vector<std::string> annotatedImageNames;
    for (int jj = 0; jj < annotatedImages.size(); jj++)
    {
      int lastIndex = annotatedImages[jj].find_last_of(".");
      std::string rawName = annotatedImages[jj].substr(0, lastIndex);
      annotatedImageNames.push_back(rawName);
    }
    if (loadDescriptors_)
    {
      std::cout << "Trying to load descriptors from file" << std::endl;
      bool descriptorsAvailable = file_utilities::loadDescriptorsFromFile(
          descriptorsFilePath, annotatedImageNames, &descriptorsVec);

      if (descriptorsAvailable)
      {
        std::cout << "Load descriptors to create vocabulary" << std::endl;
        addDescriptorsToBagOfWords(descriptorsVec);
      }
    }
    else
    {
      if (successfulFileLoad)
      {
        std::cout << "Find descriptors from bounding boxes." << std::endl;
        for (int ii = 0; ii < annotatedImages.size(); ii++)
        {
          std::cout << "Processing file " << annotatedImages[ii] << std::endl;
          std::string imageAbsolutePath = directory.string() + "/" + annotatedImages[ii];
          image = cv::imread(imageAbsolutePath);
          if (!image.data)
          {
            std::cout << "Error reading file " << annotatedImages[ii] << std::endl;
            continue;
          }

          imageROI = image(boundingBox[ii]);
          addDescriptorsToBagOfWords(imageROI);
        }
      }
      else
      {
        std::cout << "Unsuccessful annotations file load for bag of words "
                  << "vocabulary. Exiting!" << std::endl;
        exit(1);
      }

      if (saveDescriptors_)
      {
        std::cout << "Saving descriptors to file" << std::endl;
        std::cout << "Number of annotated images: "
                  << annotatedImageNames.size()
                  << std::endl;
        std::cout << "Number of descriptors: "
                  << getBagOfWordsDescriptors().size()
                  << std::endl;
        file_utilities::saveDataToFile(descriptorsFilePath, annotatedImageNames,
            getBagOfWordsDescriptors());
      }
    }


    std::cout << "Creating Visual Vocabulary" << std::endl;


    struct timeval startwtime, endwtime;
    gettimeofday(&startwtime , NULL);
    bowTrainerPtr_->createVocabulary();
    gettimeofday(&endwtime , NULL);
    double vocCreationTime = static_cast<double>((endwtime.tv_usec -
          startwtime.tv_usec) / 1.0e6
        + endwtime.tv_sec - startwtime.tv_sec);
    std::cout << "The vocabulary was created after " << vocCreationTime << " seconds" << std::endl;

    return true;
  }

  /**
   * @brief
   */
  std::vector<double> FeatureExtraction::getFeatureVector() const
  {
    return featureVector_;
  }

  /**
   * @brief
   */
  std::vector<std::vector<double> > FeatureExtraction::getFeatureMatrix() const
  {
    return featureMatrix_;
  }

  /**
   * @brief
   */
  cv::Mat FeatureExtraction::getBagOfWordsVocabulary() const
  {
    return bowTrainerPtr_->getVocabulary();
  }

  int FeatureExtraction::getFeatureNumber()
  {
    return numFeatures_;
  }

  /**
   * @brief
   */
  void FeatureExtraction::setBagOfWordsVocabulary(const cv::Mat& vocabulary)
  {
    bowTrainerPtr_->setVocabulary(vocabulary);
  }


  /**
   * @brief This function checks whether a Bag of Words vocabulary is needed.
   * @return [bool] Variable declaring whether a bag of words vocabulary is
   * needed or not.
   */
  bool FeatureExtraction::bagOfWordsVocabularyNeeded()
  {
    if (!chosenFeatureTypesMap_["sift"])
      return false;
    else
      return true;
  }

  /**
   * @brief
   */
  std::vector<cv::Mat> FeatureExtraction::getBagOfWordsDescriptors() const
  {
    return bowTrainerPtr_->getDescriptors();
  }
}  // namespace pandora_vision
