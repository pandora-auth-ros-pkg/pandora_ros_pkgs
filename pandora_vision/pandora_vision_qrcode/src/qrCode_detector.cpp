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
 * Author: Miltiadis-Alexios Papadopoulos
 *********************************************************************/

#include "pandora_vision_qrcode/qrCode_detector.h"

#define DEBUG_MODE false

namespace pandora_vision {

  QrCodeDetector::QrCodeDetector() : debug_publish(false)
  {
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
  }

  /**
    @brief Creates view for debugging purposes.
    @param image [zbar::Image&] The image
    @return void
   */
  void QrCodeDetector::debug_show(const zbar::Image& image)
  {
    cvtColor(gray_frame, debug_frame, CV_GRAY2BGR);

    int counter = 0;
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end(); ++symbol)
    {
      for(int i = 0; i < symbol->get_location_size(); i++)
      {
        line(debug_frame,
            cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)),
            cv::Point(symbol->get_location_x((i+1)%symbol->get_location_size()),
            symbol->get_location_y((i+1)%symbol->get_location_size())),
            cv::Scalar(0, 255, 0), 2, 8, 0);
      }

      cv::circle(debug_frame,
          qrcode_list[counter].qrcode_center,
          6,
          cv::Scalar(0, 0, 255),
          CV_FILLED);

      counter++;
    }

    #if DEBUG_MODE
    if(!debug_frame.empty())
    {
      cv::imshow("[QrCodeNode] processed frame", debug_frame);
      cv::imshow("[QrCodeNode] input frame", input_frame);
      cv::waitKey(1);
    }
    #endif
  }



  /**
    @brief Detects qrcodes and stores them in a vector.
    @param frame [cv::Mat] The image in which the QRs are detected
    @return void
   */
  void QrCodeDetector::detect_qrcode(cv::Mat frame)
  {
    frame.copyTo(input_frame);
    cvtColor(input_frame, gray_frame, CV_BGR2GRAY);

    cv::Mat blured;
    normalize(gray_frame, gray_frame, 255, 0, cv::NORM_MINMAX);
    cv::GaussianBlur(gray_frame, blured, cv::Size(0, 0), gaussiansharpenblur);
    cv::addWeighted(gray_frame, 1 + gaussiansharpenweight, blured,
        -gaussiansharpenweight, 0, gray_frame);

    int width = gray_frame.cols;
    int height = gray_frame.rows;
    uchar *raw = gray_frame.data;

    zbar::Image image(width, height, "Y800", raw, width * height);

    scanner.scan(image);
    qrcode_list.clear();

    for (zbar::Image::SymbolIterator symbol = image.symbol_begin();
        symbol != image.symbol_end(); ++symbol)
    {
      QrCode detected_code;
      detected_code.qrcode_desc = symbol->get_data();

      cv::Point detected_center;
      for(int i = 0; i < symbol->get_location_size(); i++)
      {
        detected_center.x += symbol->get_location_x(i);
        detected_center.y += symbol->get_location_y(i);
      }

      detected_center.x /= symbol->get_location_size();
      detected_center.y /= symbol->get_location_size();

      detected_code.qrcode_center = detected_center;
      qrcode_list.push_back(detected_code);
    }

    if(debug_publish)
    {
      debug_show(image);
    }

    image.set_data(NULL, 0);
  }
}// namespace pandora_vision
