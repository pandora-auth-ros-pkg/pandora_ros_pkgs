/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_HOLE_FUSION_NODE_UTILS_NOISE_ELIMINATION_H
#define PANDORA_VISION_HOLE_HOLE_FUSION_NODE_UTILS_NOISE_ELIMINATION_H

#include "hole_fusion_node/utils/morphological_operators.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace hole_fusion
{
  /**
    @class NoiseElimination
    @brief Provides methods for elimination of the kinect noise
   **/
  class NoiseElimination
  {
    public:
      /**
        @brief Interpolates the noise produced by the depth sensor.
        The black blobs take the depth value of the closest neighbour obstacles.
        @param[in] inImage [const cv::Mat&] The input image
        @param[out] outImage [cv::Mat*] The output image
        @return void
       **/
      static void brushfireNear(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Iteration for the interpolateNoise_brushNear function
        @param[in,out] image [cv::Mat*] The input image
        @param index [const int&] Where to start the brushfire
        algorithm (index = y * cols + x)
        @return void
       **/
      static void brushfireNearStep(cv::Mat* image, const int& index);

      /**
        @brief Changes the interpolation method according to the image's values
        @param[in] image [const cv::Mat&] The input image
        @return void
       **/
      static void chooseInterpolationMethod(const cv::Mat& image);

      /**
        @brief Interpolates the noise of an image at its borders.
        @param[in,out] inImage [cv::Mat*] The image whose noise will be
        interpolated at the edges.
        @return void
       **/
      static void interpolateImageBorders(cv::Mat* inImage);

      /**
        @brief Replaces the value (0) of pixel im(row, col) with the mean value
        of its non-zero neighbors.
        @param[in] inImage [const cv::Mat &] The input depth image
        @param[in] row [const int&] The row index of the pixel of interest
        @param[in] col [const int&] The column index of the pixel of interest
        @param[in,out] endFlag [bool*] True indicates that there are pixels
        left with zero value
        @return void
       **/
      static float interpolateZeroPixel(const cv::Mat& inImage,
        const int& row, const int& col, bool* endFlag);

      /**
        @brief Interpolates the noise produced by kinect. The noise is the areas
        kinect cannot produce depth measurements (value = 0)
        @param[in] inImage [const cv::Mat&] The input image
        @param[out] outImage [cv::Mat*] The output image
        @return void
       **/
      static void interpolation(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Iteration for the interpolateNoise function
        @param[in,out] inImage [cv::Mat*] The input image
        @return flag [bool]. if flag == true there exist pixels
        that their value needs to be interpolated
       **/
      static bool interpolationIteration(cv::Mat* inImage);

      /**
        @brief Given an input image from the depth sensor, this function
        eliminates noise in it depending on the amount of noise
        @param[in] inImage [cv::Mat&] The input image
        @param[out] outImage [cv::Mat*] The denoised depth image
        @return void
       **/
      static void performNoiseElimination(const cv::Mat& inImage,
        cv::Mat* outImage);

      /**
        @brief Transforms all black noise to white
        @param[in] inImage [const cv::Mat&] The input image
        @param[out] outImage [cv::Mat*] The output image
        @return void
       **/
      static void transformNoiseToWhite(const cv::Mat& inImage,
        cv::Mat* outImage);
  };

}  // namespace hole_fusion
}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_HOLE_FUSION_NODE_UTILS_NOISE_ELIMINATION_H
