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
 * Authors: Choutas Vassilis
 *********************************************************************/

#ifndef PANDORA_VISION_HAZMAT_FILTERS_IMAGE_SIGNATURE_H
#define PANDORA_VISION_HAZMAT_FILTERS_IMAGE_SIGNATURE_H

#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"


namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @class ImageSignature
     * @brief Class that implements the image signature saliency map.
     */
    class ImageSignature
    {
      public:
        /**
         * @brief : Calculates the signature of the image.
         * @param image[const cv::Mat&] : The input image.
         * @param imgSign[cv::Mat *]: The output saliency map.
         */
        static void calculateSignature(const cv::Mat& image ,
            cv::Mat* imgSign);

        /**
         * @brief : Creates a mask for the frame based on the saliency map
         * produced by the algorithm.
         * @param frame[const cv::Mat&] : The input image.
         * @param mask[cv::Mat*] : The output matsk.
         */
        static void createSaliencyMapMask(const cv::Mat& frame ,
            cv::Mat* mask);

        /**
         * @brief Calculates the signs of an arbitrary 1-channel matrix.
         * @param image [const cv::Mat &] : The input image
         * @param signs[cv::Mat *] : The output matrix with the signs of the
         * image.
         */
        static void signFunction(const cv::Mat& array, cv::Mat* signs);

        /**
         * @brief : Default Empty Constructor.
         */
        ImageSignature()
        {
        };

      private:
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_FILTERS_IMAGE_SIGNATURE_H
