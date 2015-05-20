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
 * Authors:
 *   Miltiadis-Alexios Papadopoulos
 *   Vassilis Choutas <vasilis4ch@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_QRCODE_QRCODE_DETECTOR_H
#define PANDORA_VISION_QRCODE_QRCODE_DETECTOR_H

#include <iostream>
#include <string>
#include <vector>
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include "pandora_vision_qrcode/qrcode_poi.h"

namespace pandora_vision
{
  class QrCodeDetector
  {
    public:
      typedef boost::shared_ptr<QrCodePOI> QrCodePOIPtr;

      /*
       * @brief: The main constructor for the QR detector objects.
       * @param gaussianSharpenBlur[int]: The standard deviation in the x axis
       * for the Gaussian Filter.
       * @param gaussianSharpenWeight[float]: The weight for the summation
       * that is used to sharpen the image.
       * @param debugCode[bool]: A boolean flag that specifies whether the
       * input and result images will be displayed.
      */
      QrCodeDetector(int gaussianSharpenBlur, float gaussianSharpenWeight,
          bool debugCode);

      /*
       * @brief : The default constructor for the QR detector objects.
      */
      QrCodeDetector(){}
      virtual ~QrCodeDetector() {}

      void set_debug(bool flag)
      {
        debugQrcode_ = flag;
      };

      void setSharpenBlur(int gaussianSharpenBlur)
      {
        gaussianSharpenBlur_ = gaussianSharpenBlur;
      }

      void setSharpenWeight(float gaussianSharpenWeight)
      {
        gaussianSharpenWeight_ = gaussianSharpenWeight;
      }

      /**
        @brief Detects qrcodes and stores them in a vector.
        @param frame [cv::Mat] The image in which the QRs are detected
        @return void
       **/
      std::vector<POIPtr> detectQrCode(const cv::Mat& frame);

    private:
      //!< Filter Parameters
      int gaussianSharpenBlur_;
      double gaussianSharpenWeight_;

      //!< Debug image topic
      std::string debugTopic_;

      //!< Debug images publisher flag
      bool debugQrcode_;

      //!< QrCode scanner
      zbar::ImageScanner scanner_;

      //!< List of detected qrcodes
      std::vector<POIPtr> qrCodeList_;
      
      /**
        @brief Creates view for debugging purposes.
        @param image [zbar::Image&] The QR image. 
        @param inputFrame[const cv::Mat&]: The input frame of the system.
        @param constrastImage[const cv::Mat&]: The image with the enhanced 
        contrast used to detect the QR codes.
        @return void
       **/
      void debugShow(const zbar::Image& image, const cv::Mat& inputFrame,
          const cv::Mat& contrastImage);
      
      friend class QrCodeDetectorTest;
  };
}  // namespace pandora_vision

#endif  // PANDORA_VISION_QRCODE_QRCODE_DETECTOR_H
