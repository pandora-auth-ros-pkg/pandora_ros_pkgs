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

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "pandora_vision_victim/utilities/bag_of_words_trainer.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  BagOfWordsTrainer::BagOfWordsTrainer()
  {
    featureDetectorType_ = "SIFT";
    descriptorExtractorType_ = "SIFT";
    descriptorMatcherType_ = "FlannBased";

    featureDetector_ = cv::FeatureDetector::create(featureDetectorType_);
    descriptorExtractor_ = cv::DescriptorExtractor::create(
        descriptorExtractorType_);
    descriptorMatcher_ = cv::DescriptorMatcher::create(descriptorMatcherType_);

    int termCriteriaType = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    int termCriteriaMaxIterations = 1000;
    double termCriteriaEpsilon = 1e-3;

    int dictionarySize = 1000;
    cv::TermCriteria termCrit(termCriteriaType, termCriteriaMaxIterations,
        termCriteriaEpsilon);

    int retries = 1;
    int flags = cv::KMEANS_PP_CENTERS;

    bowKmeansTrainer_ = new cv::BOWKMeansTrainer(dictionarySize, termCrit,
        retries, flags);

    bowVocabulary_ = cv::Mat::zeros(dictionarySize, 1, CV_32FC1);

    bowDescriptorExtractor_ = new cv::BOWImgDescriptorExtractor(
        descriptorExtractor_, descriptorMatcher_);
  }

  /**
   * @brief Constructor
   */
  BagOfWordsTrainer::BagOfWordsTrainer(
      const std::string& featureDetectorType,
      const std::string& descriptorExtractorType,
      const std::string& descriptorMatcherType,
      int dictionarySize)
  {
    featureDetectorType_ = featureDetectorType;
    descriptorExtractorType_ = descriptorExtractorType;
    descriptorMatcherType_ = descriptorMatcherType;

    featureDetector_ = cv::FeatureDetector::create(featureDetectorType_);
    descriptorExtractor_ = cv::DescriptorExtractor::create(
        descriptorExtractorType_);
    descriptorMatcher_ = cv::DescriptorMatcher::create(descriptorMatcherType_);
    /*
     *factoryExtractor_ = new SiftExtractor(featureDetectorType,
     *    descriptorExtractorType);
     */

    int termCriteriaType = CV_TERMCRIT_ITER + CV_TERMCRIT_EPS;
    int termCriteriaMaxIterations = 100;
    double termCriteriaEpsilon = 1e-3;

    cv::TermCriteria termCrit(termCriteriaType, termCriteriaMaxIterations,
        termCriteriaEpsilon);

    int retries = 1;
    int flags = cv::KMEANS_PP_CENTERS;

    bowKmeansTrainer_ = new cv::BOWKMeansTrainer(dictionarySize, termCrit,
        retries, flags);

    bowVocabulary_ = cv::Mat::zeros(dictionarySize, 1, CV_32FC1);

    bowDescriptorExtractor_ = new cv::BOWImgDescriptorExtractor(
        descriptorExtractor_, descriptorMatcher_);
  }

  /**
   * @brief Destructor
   */
  BagOfWordsTrainer::~BagOfWordsTrainer()
  {
  }

  /**
   * @brief
   */
  void BagOfWordsTrainer::addDescriptors(const cv::Mat& descriptorsVec)
  {
    bowKmeansTrainer_->add(descriptorsVec);
  }

  /**
   * @brief
   */
  std::vector<cv::Mat> BagOfWordsTrainer::getDescriptors() const
  {
    return bowKmeansTrainer_->getDescriptors();
  }

  /**
   * @brief
   */
  void BagOfWordsTrainer::createVocabulary()
  {
    bowVocabulary_ = bowKmeansTrainer_->cluster();
    bowDescriptorExtractor_->setVocabulary(bowVocabulary_);
  }

  /**
   * @brief
   */
  void BagOfWordsTrainer::setVocabulary(const cv::Mat& vocabulary)
  {
    bowVocabulary_ = vocabulary;
    bowDescriptorExtractor_->setVocabulary(bowVocabulary_);
  }

  /**
   * @brief
   */
  void BagOfWordsTrainer::createBowRepresentation(const cv::Mat& inImage,
      cv::Mat* descriptors)
  {
    std::vector<cv::KeyPoint> keyPoints;
    featureDetector_->detect(inImage, keyPoints);
    bowDescriptorExtractor_->compute(inImage, keyPoints, *descriptors);
  }

  cv::Mat BagOfWordsTrainer::getVocabulary() const
  {
    return bowVocabulary_;
  }

  void BagOfWordsTrainer::plotDescriptor(const cv::Mat& descriptor)
  {
    // Initialize the window where the histogram will be displayed.
    std::string windowName = "BagOfWords Descriptor";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    // Initialize the window dimensions.
    const int windowHeight = 640;
    const int windowWidth = descriptor.cols;

    int bins = descriptor.cols > descriptor.rows ? descriptor.cols :
      descriptor.rows;

    // Create the canvas where the descriptor/histogram of visual words
    // will be drawn.
    cv::Mat canvas = cv::Mat::zeros(windowHeight, bins, CV_8UC3);

    for (int i = 0; i < bins; ++i)
    {
      cv::line(canvas,
          cv::Point(i, windowHeight),
          cv::Point(i, windowHeight - descriptor.at<float>(i) * windowHeight),
          cv::Scalar(255, 0, 0),
          1,
          CV_AA,
          0);  
    }

    cv::imshow(windowName, canvas);
    cv::waitKey(0);

    cv::destroyAllWindows();
  }
}  // namespace pandora_vision
