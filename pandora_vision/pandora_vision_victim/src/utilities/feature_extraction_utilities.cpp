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

#include "pandora_vision_victim/utilities/feature_extraction_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @brief Constructor
   */
  FeatureExtractionUtilities::FeatureExtractionUtilities()
  {
  }

  /**
   * @brief Destructor
   */
  FeatureExtractionUtilities::~FeatureExtractionUtilities()
  {
  }

  /**
   * @brief
   * @param newMax [double]
   * @param newMin [double]
   * @param image [cv::Mat*]
   * @param minVec [std::vector<double>*]
   * @param maxVec [std::vector<double>*]
   * @return void
   */
  void FeatureExtractionUtilities::findMinMaxParameters(double newMax,
      double newMin, cv::Mat* image, std::vector<double>* minVec,
      std::vector<double>* maxVec)
  {
    double minVal, maxVal;
    if (!minVec->empty())
      minVec->clear();
    if (!maxVec->empty())
      maxVec->clear();

    for (int ii = 0; ii < image->cols; ii++)
    {
      cv::minMaxLoc(image->col(ii), &minVal, &maxVal);
      minVec->push_back(minVal);
      maxVec->push_back(maxVal);
    }
    performMinMaxNormalization(newMax, newMin, image, *minVec, *maxVec);
  }

  /**
   * @brief
   * @param newMax [double]
   * @param newMin [double]
   * @param image [cv::Mat*]
   * @param minVec [std::vector<double>&]
   * @param maxVec [std::vector<double>&]
   * @return void
   */
  void FeatureExtractionUtilities::performMinMaxNormalization(double newMax,
      double newMin, cv::Mat* image, const std::vector<double>& minVec,
      const std::vector<double>& maxVec)
  {
    for (int ii = 0; ii < image->cols; ii++)
    {
      subtract(image->col(ii), minVec.at(ii), image->col(ii));
      if (maxVec.at(ii) != minVec.at(ii))
      {
        divide(image->col(ii), maxVec.at(ii) - minVec.at(ii), image->col(ii));
        multiply(image->col(ii), newMax - newMin, image->col(ii));
      }
      add(image->col(ii), newMin, image->col(ii));
    }
  }

  /**
   * @brief
   * @param image [cv::Mat*]
   * @param meanVec [std::vector<double>*]
   * @param stdDevVec [std::vector<double>*]
   * @return void
   */
  void FeatureExtractionUtilities::findZScoreParameters(cv::Mat* image,
      std::vector<double>* meanVec, std::vector<double>* stdDevVec)
  {
    cv::Scalar mu, sigma;
    if (!meanVec->empty())
      meanVec->clear();
    if (!stdDevVec->empty())
      stdDevVec->clear();

    for (int ii = 0; ii < image->cols; ii++)
    {
      meanStdDev(image->col(ii), mu, sigma);
      meanVec->push_back(mu.val[0]);
      stdDevVec->push_back(sigma.val[0]);
    }
    performZScoreNormalization(image, *meanVec, *stdDevVec);
  }

  void FeatureExtractionUtilities::performZScoreNormalization(cv::Mat* image,
      const std::vector<double>& meanVec,
      const std::vector<double>& stdDevVec)
  {
    for (int ii = 0; ii < image->cols; ii++)
    {
      subtract(image->col(ii), meanVec.at(ii), image->col(ii));
      if (stdDevVec.at(ii) != 0.0)
        divide(image->col(ii), stdDevVec.at(ii), image->col(ii));
    }
  }

  /**
   * @brief This function performs PCA analysis to reduce the feature
   * dimensions.
   * @param featuresMat [cv::Mat*] Feature matrix to be used in the PCA
   * analysis.
   * @return void
   */
  cv::Mat FeatureExtractionUtilities::performPcaAnalysis(
      const cv::Mat& featuresMat, int nEigens)
  {
    cv::Mat projectedFeaturesMat;
    cv::PCA pca(featuresMat, cv::Mat(), CV_PCA_DATA_AS_ROW);
    cv::Mat mean = pca.mean.clone();
    cv::Mat eigenvalues = pca.eigenvalues.clone();
    cv::Mat eigenvectors = pca.eigenvectors.clone();
    std::cout << "EigenValues" << eigenvalues << std::endl;
    std::cout << "EigenVectors " << eigenvectors.size() << std::endl;
    pca.project(featuresMat, projectedFeaturesMat);
    return projectedFeaturesMat;
  }
}  // namespace pandora_vision

