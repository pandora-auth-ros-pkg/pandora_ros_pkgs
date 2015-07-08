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
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_DISCRETE_WAVELET_TRANSFORM_H
#define PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_DISCRETE_WAVELET_TRANSFORM_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class DiscreteWaveletTransform
  {
    public:
      typedef boost::shared_ptr<cv::Mat> MatPtr;
      typedef boost::shared_ptr<cv::Mat const> MatConstPtr;

    public:
      /**
       * @brief Constructor used to implement the Ingrid Daubechies
       * Wavelets
       * @param kernelSize [int] The size of the kernel used to
       * perform the transform
       **/
      explicit DiscreteWaveletTransform(int kernelSize);
      /**
       * @brief Constructor used to implement the DWT with a user
       * defined kernel
       * @param columnKernelLow [cost cv::Mat&] The low frequency
       * kernel used to perform the convolution of the DWT column-wise
       * @param columnKernelHigh [const cv::Mat&] The high frequency
       * kernel used to perform the convolution of the DWT column-wise
       **/
      DiscreteWaveletTransform(const cv::Mat& columnKernelLow,
          const cv::Mat& columnKernelHigh);

      /**
       * @brief Virtual Destructor
       **/
      virtual ~DiscreteWaveletTransform();

    public:
      /**
       * @brief Return the final LL result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of LL images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getLowLow(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final LH result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of LH images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getLowHigh(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final HL result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of HL images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getHighLow(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final HH result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of HH images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getHighHigh(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of images that are
       * the result of the transform with order LL, LH, HL, HH and
       * so on according to the level
       **/
      std::vector<MatPtr> dwt2D(const cv::Mat& inImage, int level = 1);

    private:
      /**
       * @brief Perform convolution with a vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param low [bool] Whether the low frequency filter is used
       * @param rows [bool] Whether the type of convolution will
       * be performed row-wise
       * @return [cv::Mat] The result of the convolution
       **/
      cv::Mat optionalConv(const cv::Mat& inImage,
          bool low, bool rows);
      /**
       * @brief Perform convolution with a column vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param low [bool] Whether the low frequency filter is used
       * @return [cv::Mat] The result of the convolution performed
       * column-wise
       **/
      cv::Mat convCols(const cv::Mat& inImage, bool low);
      /**
       * @brief Perform convolution with a row vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param low [bool] Whether the low frequency filter is used
       * @return [cv::Mat] The result of the convolution performed
       * row-wise
       **/
      cv::Mat convRows(const cv::Mat& inImage, bool low);

      /**
       * @brief Creating an image by taking half the values of the
       * image that the convolution with the dwt kernel resulted to
       * @param image [const cv::Mat&] The input image
       * @param rows [bool] Whether the images are downsampled row-wise
       * @param subImage [const MatPtr&] The output image after
       * subsampling
       **/
      void subSample(const cv::Mat& image, bool rows,
          const MatPtr& subImage);

      /**
       * @brief Perform convolution row-wise or column-wise and the oposite
       * subsampling according to boolean input
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param rows [bool] Whether the convolution is performed
       * row-wise. Then downsampling is performed column-wise
       * @param subImageLow [const MatPtr&] The output low frequency image
       * @param subImageHigh [const MatPtr&] The output high frequency image
       **/
      void convAndSubSample(const cv::Mat& inImage, bool rows,
          const MatPtr& subImageLow, const MatPtr& subImageHigh);

    private:
      /// The row kernel used to perform the DWT that represents the
      /// low - pass filter used
      cv::Mat rowKernelLow_;
      /// The column kernel used to perform the DWT that represents
      /// the  low - pass filter used
      cv::Mat columnKernelLow_;
      /// The row kernel used to perform the DWT that represents the
      /// band - pass filter used for high frequencies
      cv::Mat rowKernelHigh_;
      /// The column kernel used to perform the DWT that represents
      /// the band - pass filter used for high frequencies
      cv::Mat columnKernelHigh_;

      friend class DiscreteWaveletTransformTest;
  };
  typedef DiscreteWaveletTransform::MatPtr MatPtr;
  typedef DiscreteWaveletTransform::MatConstPtr MatConstPtr;

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_PANDORA_VISION_UTILITIES_DISCRETE_WAVELET_TRANSFORM_H
