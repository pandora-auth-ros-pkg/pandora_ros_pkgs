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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/feature_extractors/dft_coeffs.h"


namespace pandora_vision
{
  DFTCoeffsExtractor::DFTCoeffsExtractor(cv::Mat* img)
    : BaseFeatureExtractor(img)
  {
    
  }
  
  std::vector<double> DFTCoeffsExtractor::extract(void)
  {
    std::vector<double>temp(6);
    cv::Mat padded;
    
    //!< Expand input image to optimal size
    int rows = cv::getOptimalDFTSize( _img->rows );
    int cols = cv::getOptimalDFTSize( _img->cols );
    
    //!< On the border add zero values
    copyMakeBorder(*_img, padded, 0, rows - _img->rows, 0, cols - _img->cols,
    cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded),
                        cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    
    //!< Add to the expanded another plane with zeros
    merge(planes, 2, complexI);
    
    //!< This way the result may fit in the source matrix
    dft(complexI, complexI);
    
    //!< Normalize the dft coeffs
    for (int ii = 0; ii < complexI.rows; ii++)
      for(int jj = 0; jj < complexI.cols; jj++)
          complexI.at<float>(ii, jj)=complexI.at<float>(ii, jj) /
                                      (complexI.cols * complexI.rows);
              

    //!< Compute the magnitude
    // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    split(complexI, planes);
    
    //!< planes[0] = magnitude
    magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magI = planes[0];
    temp[0] = static_cast<double>(magI.at<float>(0, 0));
    temp[1] = static_cast<double>(magI.at<float>(0, 1));
    temp[2] = static_cast<double>(magI.at<float>(1, 0));
    temp[3] = static_cast<double>(magI.at<float>(0, 2));
    temp[4] = static_cast<double>(magI.at<float>(1, 1));
    temp[5] = static_cast<double>(magI.at<float>(2, 0));
    return temp;
  }
}


