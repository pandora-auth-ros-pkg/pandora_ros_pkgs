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

#include "pandora_vision_victim/utilities/principal_component_analysis.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  PrincipalComponentAnalysis::PrincipalComponentAnalysis()
  {
    pca_ = cv::PCA();
  }

  PrincipalComponentAnalysis::PrincipalComponentAnalysis(const cv::Mat& featuresMat,
      double retainedVariance)
  {
    pca_ = cv::PCA(featuresMat, cv::Mat(), CV_PCA_DATA_AS_ROW, retainedVariance);
  }

  PrincipalComponentAnalysis::~PrincipalComponentAnalysis()
  {
  }

  void PrincipalComponentAnalysis::performPCA(const cv::Mat& featuresMat)
  {
    pca_.computeVar(featuresMat, cv::Mat(), CV_PCA_DATA_AS_ROW, 0.95);
  }

  void PrincipalComponentAnalysis::project(
      const cv::Mat& featuresMat, cv::Mat* projectedFeaturesMat)
  {
    pca_.project(featuresMat, *projectedFeaturesMat);
  }

  void PrincipalComponentAnalysis::save(const std::string& fileName)
  {
      cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
      fs << "mean" << pca_.mean;
      fs << "eigenvectors" << pca_.eigenvectors;
      fs << "eigenvalues" << pca_.eigenvalues;
      fs.release();
  }

  void PrincipalComponentAnalysis::load(const std::string& fileName)
  {
      cv::FileStorage fs(fileName, cv::FileStorage::READ);
      fs["mean"] >> pca_.mean;
      fs["eigenvectors"] >> pca_.eigenvectors;
      fs["eigenvalues"] >> pca_.eigenvalues;
      fs.release();
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision


