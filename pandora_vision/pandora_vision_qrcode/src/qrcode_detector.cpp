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

#include "pandora_vision_qrcode/qrcode_detector.h"

namespace pandora_vision
{
  /*
   * @brief: The main constructor for the QR detector objects.
   * @param gaussianSharpenBlur[int]: The standard deviation in the x axis
   * for the Gaussian Filter.
   * @param gaussianSharpenWeight[float]: The weight for the summation
   * that is used to sharpen the image.
   * @param debugCode[bool]: A boolean flag that specifies whether the
   * input and result images will be displayed.
   */
  QrCodeDetector::QrCodeDetector(int gaussianSharpenBlur,
      float gaussianSharpenWeight, bool debugCode)
  {
    // Initiliaze the zbar scanner_.
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner_.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    gaussianSharpenBlur_ = gaussianSharpenBlur;
    gaussianSharpenWeight_ = gaussianSharpenWeight;
    debugCode = debugCode;
  }
  
  /**
    @brief Creates view for debugging purposes.
    @param image [zbar::Image&] The QR image. 
    @param inputFrame[const cv::Mat&]: The input frame of the system.
    @param constrastImage[const cv::Mat&]: The image with the enhanced 
    contrast used to detect the QR codes.
    @return void
   **/
  void QrCodeDetector::debugShow(const zbar::Image& image,
      const cv::Mat& inputFrame, const cv::Mat& contrastImage)
  {
    cv::Mat debugFrame;

    cvtColor(contrastImage, debugFrame, CV_GRAY2BGR);

    int counter = 0;
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end(); ++symbol)
    {
      for (int i = 0; i < symbol->get_location_size(); i++)
      {
        line(debugFrame,
            cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)),
            cv::Point(symbol->get_location_x((i+1)%symbol->get_location_size()),
            symbol->get_location_y((i+1)%symbol->get_location_size())),
            cv::Scalar(0, 255, 0), 2, 8, 0);
      }

      cv::circle(debugFrame,
          qrCodeList_[counter]->getPoint(),
          6,
          cv::Scalar(0, 0, 255),
          CV_FILLED);

      counter++;
    }

    if (debugQrcode_)
    {
      if (!debugFrame.empty())
      {
        cv::imshow("[QrCodeNode] Processed Frame", debugFrame);
        cv::imshow("[QrCodeNode] Input Frame", inputFrame);
        cv::waitKey(10);
      }
    }
  }

  /**
    @brief Detects qrcodes and stores them in a vector.
    @param frame [cv::Mat] The image in which the QRs are detected
    @return void
    */
  std::vector<POIPtr> QrCodeDetector::detectQrCode(const cv::Mat& frame)
  {
    cv::Mat grayFrame;

    if (frame.channels() == 3)
      cv::cvtColor(frame, grayFrame, CV_BGR2GRAY);
    else
      frame.copyTo(grayFrame);
    cv::Mat blured;
    normalize(grayFrame, grayFrame, 255, 0, cv::NORM_MINMAX);
    cv::GaussianBlur(grayFrame, blured, cv::Size(0, 0), gaussianSharpenBlur_);
    cv::addWeighted(grayFrame, 1 + gaussianSharpenWeight_, blured,
        -gaussianSharpenWeight_, 0, grayFrame);
    int width = grayFrame.cols;
    int height = grayFrame.rows;
    uchar *raw = grayFrame.data;

    zbar::Image image(width, height, "Y800", raw, width * height);

    scanner_.scan(image);
    qrCodeList_.clear();

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end(); ++symbol)
    {
      QrCodePOIPtr qrCodePOIPtr(new QrCodePOI);
      qrCodePOIPtr->setContent(symbol->get_data());

      cv::Point detected_center;
      for (int i = 0; i < symbol->get_location_size(); i++)
      {
        detected_center.x += symbol->get_location_x(i);
        detected_center.y += symbol->get_location_y(i);
      }

      detected_center.x /= symbol->get_location_size();
      detected_center.y /= symbol->get_location_size();

      qrCodePOIPtr->setPoint(detected_center);
      qrCodeList_.push_back(qrCodePOIPtr);
    }

    if (debugQrcode_)
    {
      debugShow(image, frame, grayFrame);
    }
    image.set_data(NULL, 0);

    return qrCodeList_;
  }

}  // namespace pandora_vision
