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

#ifndef PANDORA_VISION_VICTIM_UTILITIES_PRINCIPAL_COMPONENT_ANALYSIS_H
#define PANDORA_VISION_VICTIM_UTILITIES_PRINCIPAL_COMPONENT_ANALYSIS_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @class PrincipalComponentAnalysis
   * @brief This class implements a wrapper of the OPENCV Principal Component
   * Analysis.
   */
  class PrincipalComponentAnalysis
  {
    public:
      /**
       * @brief Default Constructor.
       */
      PrincipalComponentAnalysis();

      /**
       * @brief Constructor. Initializes PCA with a specified minimum retained
       * variance.
       */
      PrincipalComponentAnalysis(const cv::Mat& featuresMat,
          double retainedVariance);

      /**
       * @brief Destructor.
       */
      ~PrincipalComponentAnalysis();

      /**
       * @brief This function performs Principal Component Analysis to a
       * supplied dataset.
       * @param featuresMat [const cv::Mat&] The features matrix in which to
       * perform the analysis.
       * @return void
       */
      void performPCA(const cv::Mat& featuresMat);

      /**
       * @brief This function projects a matrix into the Principal Component
       * Subspace.
       * @param featuresMat [const cv::Mat&] The features matrix to be
       * projected.
       * @param projectedFeaturesMat [cv::Mat*] The projected features matrix.
       */
      void project(const cv::Mat& featuresMat, cv::Mat* projectedFeaturesMat);

      /**
       * @brief This function saves the Principal components in a file.
       * @param fileName [const std::string&] The name of the file in which to
       * save the data.
       * @return void
       */
      void save(const std::string& fileName);

      /**
       * @brief This function loads the Principal components from a file.
       * @param fileName [const std::string&] The name of the file from which
       * to load the data.
       * @return void
       */
      void load(const std::string& fileName);
    private:
      cv::PCA pca_;
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_UTILITIES_PRINCIPAL_COMPONENT_ANALYSIS_H
