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

#include <vector>

#include "pandora_vision_hazmat/filters/image_signature.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @brief Calculates the signs of an arbitrary 1-channel matrix.
     * @param image [const cv::Mat &] : The input image
     * @param signs [cv::Mat *] : The output matrix with the signs of the
     * image.
     */
    void ImageSignature::signFunction(const cv::Mat& array, cv::Mat* signs)
    {
      if ( array.channels() > 1 )
      {
        ROS_ERROR("Invalid channel number!\n");
        signs->data = NULL;
        return;
      }

      cv::divide(array, abs(array), *signs);
    }

    /**
     * @brief : Calculates the signature of the image.
     * @param image[const cv::Mat&] : The input image.
     * @param imgSign[cv::Mat *]: The output saliency map.
     */
    void ImageSignature::calculateSignature(const cv::Mat& image,
        cv::Mat* imgSign)
    {
      cv::Mat imageDCT;

      // Compute the discrete cosine transform of the input image.
      cv::dct(image, imageDCT);

      // Calculate the signature of the image.
      ImageSignature::signFunction(imageDCT, imgSign);

      return;
    }

    /**
     * @brief : Creates a mask for the frame based on the saliency map
     * produced by the algorithm.
     * @param frame[const cv::Mat&] : The input image.
     * @param mask[cv::Mat*] : The output matsk.
     */
    void  ImageSignature::createSaliencyMapMask(const cv::Mat& frame,
        cv::Mat* mask)
    {
      if (!frame.data)
      {
        ROS_ERROR("Invalid frame!\n ");
        mask->data = NULL;
        return;
      }

      static cv::Mat saliency;

      // Convert the frame to a 3-channel float image and scale
      // it's values accordingly.
      frame.convertTo(saliency , CV_32FC3 , 1/255.f);

      // Resize the frame so as to process it correctly.
      // TO DO : read the size from file.
      cv::resize(saliency, saliency, cv::Size(64, 48));

      static cv::Mat signature;
      static cv::Mat invDCT;

      static std::vector<cv::Mat> channels;

      // Split the input frame into it's separate channels.
      cv::split(saliency , channels);

      // The resulting mask is a 1-channel floating point image , so as to
      // correctly perform forward and inverse Discrete Cosine
      // Transformatios.
      cv::Mat sum = cv::Mat::zeros(saliency.size() , CV_32FC1);

      // Temporary container for the inverse Discrete Cosine Transform
      // for the #i channel of the image.
      static cv::Mat tempMap = cv::Mat::zeros(saliency.size() , CV_32FC1);
      // For every channel of the image :
      for (int i = 0; i < saliency.channels(); i++)
      {
        ImageSignature::calculateSignature(channels[i] , &signature);

        // Perform the inverse DCT on the signature.
        cv::dct(signature, invDCT , cv::DCT_INVERSE);
        tempMap = invDCT.mul(invDCT);
        cv::GaussianBlur(tempMap, tempMap, cv::Size(5, 5), 0);
        // Calculate the total map.
        sum = sum + tempMap;
      }

      // Calculate the mean of the saliency values of the 3 input channels.
      sum = (1/3.f) * sum;

      // Resize the mask so that it can be applied to the frame.
      cv::resize(sum, sum, frame.size());

      // Threshold the mask to decrease noise and keep only the regions
      // of interest.
      cv::threshold(sum, sum, 0.99, 1, cv::THRESH_BINARY);

      // Convert the mask to 1-channel 8 bit format.
      sum.convertTo(sum, CV_8UC1, 255);

      cv::medianBlur(sum, sum, 3);

      *mask = sum;

      return;
    }

}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
