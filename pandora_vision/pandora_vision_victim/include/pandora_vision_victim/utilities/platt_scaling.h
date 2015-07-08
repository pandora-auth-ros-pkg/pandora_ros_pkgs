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
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_UTILITIES_PLATT_SCALING_H
#define PANDORA_VISION_VICTIM_UTILITIES_PLATT_SCALING_H

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @class PlattScaling
   * @brief This class implements Platt's Binary Probabilistic Output, an
   * Improvement from Lin et al.
   */
  class PlattScaling
  {
    public:
      /**
       * @brief Constructor. Initializes Alpha and Beta parameters for Platt
       * Scaling.
       */
      PlattScaling();

      /**
       * @brief Default Destructor
       */
      ~PlattScaling();

      /**
       * @brief This functionn estimates the optimal Alpha and Beta parameters
       * for the sigmoid Platt Scaling Training.
       * @param predictedLabelsMat [const cv::Mat&] The predicted class labels
       * matrix.
       * @param actualLabelsMat [const cv::Mat&] The actual class labels matrix.
       * @return void
       */
      void sigmoidTrain(const cv::Mat& predictionsMat,
          const cv::Mat& actualLabelsMat);

      /**
       * @brief This function estimates the classification probability for a
       * single image, based on the trained sigmoid model.
       * @param predicted [float] The predicted class label for a single image.
       * @return [float] The prediction probability.
       */
      float sigmoidPredict(double predicted);

      /**
       * @brief This function estimates the classification probability for a
       * set of images, based on the trained sigmoid model.
       * @param predicted [float] The predicted class label for a set of images.
       * @return [std::vector<float>] The prediction probabilities.
       */
      std::vector<float> sigmoidPredict(const cv::Mat& predicted);

      /**
       * @brief This function loads the estimated Alpha and Beta parameters of
       * the Platt Scaling model.
       * @param fileName [const std::string&] The name of the file from which to
       * load the parameters.
       * @return void
       */
      void load(const std::string& fileName);

      /**
       * @brief This function saves the estimated Alpha and Beta parameters of
       * the Platt Scaling model.
       * @param fileName [const std::string&] The name of the file in which to
       * save the parameters.
       * @return void
       */
      void save(const std::string& fileName);

    private:
      /// The Platt Scaling Alpha parameter.
      double A_;
      /// The Platt Scaling Beta parameter.
      double B_;
  };
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_UTILITIES_PLATT_SCALING_H

