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

#include <opencv2/objdetect/objdetect.hpp>

#include "pandora_vision_victim/feature_extractors/hog_extractor.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Default Constructor
   */
  HOGExtractor::HOGExtractor() : FeatureExtractorFactory()
  {
    cv::Size windowSize = cv::Size(160, 128);
    cv::Size blockSize = cv::Size(16, 16);
    cv::Size blockStride = cv::Size(16, 16);
    cv::Size cellSize = cv::Size(8, 8);
    int numBins = 9;
    double winSigma = cv::HOGDescriptor::L2Hys;
    double thresholdL2Hys = 0.2;
    bool gammaCorrection = true;
    int numLevels = cv::HOGDescriptor::DEFAULT_NLEVELS;
    hogDescriptor_ = new cv::HOGDescriptor(windowSize, blockSize, blockStride,
        cellSize, numBins, 1, -1, winSigma, thresholdL2Hys, gammaCorrection,
        numLevels);
  }

  /**
   * @brief Destructor
   */
  HOGExtractor::~HOGExtractor()
  {
  }

  /**
   *
   */
  void HOGExtractor::extractFeatures(const cv::Mat& inImage,
      std::vector<float>* descriptors)
  {
    cv::Mat grayscaleImage = inImage.clone();
    // TODO(Miltos/Vassilis) Check if HOG works with RGB image.
    if (grayscaleImage.channels() > 1)
      cvtColor(grayscaleImage, grayscaleImage, CV_BGR2GRAY);
    // TODO(Miltos/Vassilis) Check proper size.
    cv::resize(grayscaleImage, grayscaleImage, cv::Size(160, 128));

    std::vector<cv::Point> pointLocations;
    hogDescriptor_->compute(grayscaleImage, *descriptors, cv::Size(0, 0),
        cv::Size(0, 0), pointLocations);
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision

