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

#include <vector>

#include "pandora_vision_victim/channels_statistics_feature_extractors/color_angles_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief Constructor
   */
  ColorAnglesExtractor::ColorAnglesExtractor(cv::Mat* img)
    : ChannelsStatisticsFeatureExtractor(img)
  {
  }

  /**
   * @brief Destructor
   */
  ColorAnglesExtractor::~ColorAnglesExtractor()
  {
  }

  /**
   * @brief This function extracts the color angles features.
   * @return [std::vector<double>] The color angles vector.
   */
  std::vector<double> ColorAnglesExtractor::extract(void)
  {
    std::vector<double> colorAnglesAndStd;

    /// Separate the image in 3 matrices, one for each of the R, G, B channels
    std::vector<cv::Mat> rgbPlanes;
    split(*img_, rgbPlanes);
    /// Compute the average pixel value of each r,g,b color component
    cv::Scalar bMean = mean(rgbPlanes[0]);
    cv::Scalar gMean = mean(rgbPlanes[1]);
    cv::Scalar rMean = mean(rgbPlanes[2]);

    /// Obtain zero-mean colour vectors r0, g0 and b0 by subtracting the
    /// corresponding average pixel value of each original color vector
    cv::Mat r0 = cv::Mat::zeros(img_->rows, img_->cols, CV_64FC1);
    cv::Mat b0 = cv::Mat::zeros(img_->rows, img_->cols, CV_64FC1);
    cv::Mat g0 = cv::Mat::zeros(img_->rows, img_->cols, CV_64FC1);

    for (int ii = 0; ii < img_->rows; ii++)
    {
      for (int jj = 0; jj < img_->cols; jj++)
      {
        b0.at<double>(ii, jj) = rgbPlanes[0].at<uchar>(ii, jj) - bMean.val[0];
        g0.at<double>(ii, jj) = rgbPlanes[1].at<uchar>(ii, jj) - gMean.val[0];
        r0.at<double>(ii, jj) = rgbPlanes[2].at<uchar>(ii, jj) - rMean.val[0];
      }
    }
    /*/
    for (int ii = 0; ii < img_->rows; ii++)
      for (int jj = 0; jj < img_->cols; jj++)
        ROS_INFO_STREAM(b0.at<double>(ii, jj));
    // */

    /// Compute the dot product of the RGB color components
    double rgDot = r0.dot(g0);
    double gbDot = g0.dot(b0);
    double rbDot = r0.dot(b0);
    double rSum = 0, bSum = 0, gSum = 0;
    /*/
    ROS_INFO_STREAM("rgDot = " << rgDot);
    ROS_INFO_STREAM("gbDot = " << gbDot);
    ROS_INFO_STREAM("rbDot = " << rbDot);
    // */

    /// Compute the lengh of the color angle vector
    for (int ii = 0; ii < r0.rows; ii++)
    {
      for (int jj = 0; jj < r0.cols; jj++)
      {
         rSum+= pow(r0.at<double>(ii, jj), 2);
         gSum+= pow(g0.at<double>(ii, jj), 2);
         bSum+= pow(b0.at<double>(ii, jj), 2);
      }
    }
    /*/
    ROS_INFO_STREAM("rSum = " << rSum);
    ROS_INFO_STREAM("gSum = " << gSum);
    ROS_INFO_STREAM("bSum = " << bSum);
    // */

    double rLength = sqrt(rSum);
    double gLength = sqrt(gSum);
    double bLength = sqrt(bSum);
    /*/
    ROS_INFO_STREAM("rLength= " << rLength);
    ROS_INFO_STREAM("gLength= " << gLength);
    ROS_INFO_STREAM("bLength= " << bLength);
    // */

    rgDot /= (rLength * gLength);
    gbDot /= (gLength * bLength);
    rbDot /= (rLength * bLength);
    /*/
    ROS_INFO_STREAM("rgDot = " << rgDot);
    ROS_INFO_STREAM("gbDot = " << gbDot);
    ROS_INFO_STREAM("rbDot = " << rbDot);
    // */

    /// Compute the color angles
    double rgAngle = acos(rgDot);
    double gbAngle = acos(gbDot);
    double rbAngle = acos(rbDot);
    /*/
    ROS_INFO_STREAM("rgAngle = " << rgAngle);
    ROS_INFO_STREAM("gbAngle = " << gbAngle);
    ROS_INFO_STREAM("rbAngle = " << rbAngle);
    // */

    cv::Mat gray;
    /// Normalised intensity standard deviation
    /// Transform the src image to grayscale
    cvtColor(*img_, gray, CV_BGR2GRAY);

    /// Compute the mean intensity value
    cv::Scalar meanI = mean(gray);

    /// Find the maximum intensity value
    double maxVal, stdDev, sum = 0;
    cv::minMaxLoc(gray, NULL, &maxVal);

    for (int ii = 0; ii < gray.rows; ii++)
      for (int jj = 0; jj < gray.cols; jj++)
      {
        sum += pow((gray.at<uchar>(ii, jj) - meanI.val[0]), 2);
      }

    stdDev = 2.0 / (maxVal * gray.cols * gray.rows) * sqrt(sum);

    /// Construct the final feature vector
    colorAnglesAndStd.push_back(rgAngle);
    colorAnglesAndStd.push_back(gbAngle);
    colorAnglesAndStd.push_back(rbAngle);
    colorAnglesAndStd.push_back(stdDev);

    return colorAnglesAndStd;
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision

