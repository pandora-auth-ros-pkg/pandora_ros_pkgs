/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
*   Victor Daropoulos
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/
#ifndef PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HARALICKFEATURE_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HARALICKFEATURE_EXTRACTOR_H

#include <vector>

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  class HaralickFeaturesExtractor
  {
    public:
      /**
       * @brief Function for calculating the normalized GLCM matrix.
       * The returned matrix is symmetric with respect to the main diagonal.
       * @param xOffset [int] horizontal offset.
       * @param yOffset [int] vertical offset.
       * @param in [cv::Mat&] A grayscale image with 8 bit values.
       * @return [cv::Mat] The GLCM matrix.
       */
      static cv::Mat calculateGLCM(int xOffset, int yOffset, const cv::Mat& in);

      /**
       * @brief Function for updating the values of the GLCM matrix.
       * @param y1 [int] The y coordinate of the first pixel
       * @param x1 [int] The x coordinate of the first pixel
       * @param y2 [int] The y coordinate of the second pixel
       * @param x2 [int] The x coordinate of the second pixel
       * @param out [cv::Mat*] The updated GLCM matrix
       * @param in [cv::Mat&] The GLCM matrix
       * @return void
       */
      static void updateGLCM(int y1, int x1, int y2, int x2,
          cv::Mat* out, const cv::Mat& in);

      /**
       * @brief Function for normalizing the values of the GLCM matrix.
       * @param in [cv::Mat*] The matrix to be normalized
       * @return [cv::Mat] The normalized matrix
       */
      static void normalizeGLCM(cv::Mat* in);

      /**
       * @brief Function for calculating the Angular Second Moment,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The angular second moment
       */
      static double getAngularSecondMoment(const cv::Mat& in);

      /**
       * @brief Function for calculating entropy,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The entropy
       */
      static double getEntropy(const cv::Mat& in);

      /**
       * @brief Function for calculating contrast,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The contrast
       */
      static double getContrast(const cv::Mat& in);

      /**
       * @brief Function for calculating variance,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The variance
       */
      static double getVariance(const cv::Mat& in);

      /**
       * @brief Function for calculating correlation,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The correlation
       */
      static double getCorrelation(const cv::Mat& in);

      /**
       * @brief Function for calculating homogeneity,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The homogeneity
       */
      static double getHomogeneity(const cv::Mat& in);

      /**
       * @brief Function for calculating sum average,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The sum average
       */
      static double getSumAverage(const cv::Mat& in);

      /**
       * @brief Function for calculating sum variance,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The sum variance
       */
      static double getSumVariance(const cv::Mat& in, const double& sumAverage);

      /**
       * @brief Function for calculating sum entropy,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The sum entropy
       */
      static double getSumEntropy(const cv::Mat& in);

      /**
       * @brief Function for calculating difference variance,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The difference variance
       */
      static double getDifferenceVariance(const cv::Mat& in);

      /**
       * @brief Function for calculating difference entropy,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @return [double] The difference entropy
       */
      static double getDifferenceEntropy(const cv::Mat& in);

      /**
       * @brief Function for calculating Info Measure of Correlation 1 and 2,
       * http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
       * @param in [cv::Mat&] The normalized GLCM matrix
       * @param feat1 [double*] The Info Measure of Correlation 1
       * @param feat2 [double*] The Info Measure of Correlation 2
       * @return void
       */
      static void getInfoMeasuresCorr(const cv::Mat& in, double* feat1, double* feat2);

      /**
       * @brief This is the main function called to extract haralick features
       * @param image [cv::Mat&] The current frame to be processed
       * @param haralickFeatures [std::vector<double>*] The vector containing
       * the Haralick features
       * @return void
       */
      static void findHaralickFeatures(const cv::Mat& image, std::vector<double>* haralickFeatures);
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HARALICKFEATURE_EXTRACTOR_H
