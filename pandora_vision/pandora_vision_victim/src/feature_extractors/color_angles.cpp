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
* Author: Marios Protopapas
*********************************************************************/

#include "pandora_vision_victim/feature_extractors/color_angles.h"


namespace pandora_vision
{
  /**
  @brief Constructor
  **/
  ColorAnglesExtractor::ColorAnglesExtractor(cv::Mat* img)
    : BaseFeatureExtractor(img)
  {

  }
  /**
  @brief this function extracts the color angles features
  @return color angles vector
  **/
  std::vector<double> ColorAnglesExtractor::extract(void)
  {
    std::vector<double> colorAnglesAndStd;

    //!< Separate the image in 3 places (R,G,B) one for each channel
    std::vector<cv::Mat> rgb_planes;
    split( *_img, rgb_planes );
    //!< Compute the average pixel value of each r,g,b color component
    cv::Scalar bmean = mean(rgb_planes[0]);
    cv::Scalar gmean = mean(rgb_planes[1]);
    cv::Scalar rmean = mean(rgb_planes[2]);

    //!< Obtain zero-mean colour vectors r0, g0 and b0 by subtracting the
    //!< Corresponding average pixel value of each original colour vector
    cv::Mat r0 = cv::Mat::zeros(_img->rows, _img->cols, CV_64FC1);
    cv::Mat b0 = cv::Mat::zeros(_img->rows, _img->cols, CV_64FC1);
    cv::Mat g0 = cv::Mat::zeros(_img->rows, _img->cols, CV_64FC1);

    for (int ii = 0; ii < _img->rows; ii++)
      for (int jj = 0; jj < _img->cols; jj++)
        b0.at<double>(ii, jj) = rgb_planes[0].at<uchar>(ii, jj) - bmean.val[0];
    for (int ii = 0; ii < _img->rows; ii++)
      for (int jj = 0; jj < _img->cols; jj++)
        g0.at<double>(ii, jj) = rgb_planes[1].at<uchar>(ii, jj) - gmean.val[0];
    for (int ii = 0; ii < _img->rows; ii++)
      for (int jj = 0; jj < _img->cols; jj++)
        r0.at<double>(ii, jj) = rgb_planes[2].at<uchar>(ii, jj) - rmean.val[0];

/*   for (int ii=0; ii< _img->rows; ii++)*/
     //for(int jj=0; jj<_img->cols; jj++)
      /*ROS_INFO_STREAM(b0.at<double>(ii,jj));*/

    //!< Compute the dot product of rgb color components
    double rgdot = r0.dot(g0);
    double gbdot = g0.dot(b0);
    double rbdot = r0.dot(b0);
    double rsum = 0, bsum = 0, gsum = 0;

/*    ROS_INFO_STREAM("rgdot= " << rgdot);*/
    //ROS_INFO_STREAM("gbdot= " << gbdot);
    //ROS_INFO_STREAM("rbdot= " << rbdot);

    //!< Compute the lengh of the color angle vector
    for (int ii = 0; ii < r0.rows; ii++)
      for(int jj = 0; jj < r0.cols; jj++)
        {
           rsum+= pow(r0.at<double>(ii, jj), 2);
           gsum+= pow(g0.at<double>(ii, jj), 2);
           bsum+= pow(b0.at<double>(ii, jj), 2);
        }
/*    ROS_INFO_STREAM("rsum= " << rsum);*/
    //ROS_INFO_STREAM("gsum= " << gsum);
    /*ROS_INFO_STREAM("bsum= " << bsum);*/

    double rlength = sqrt(rsum);
    double glength = sqrt(gsum);
    double blength = sqrt(bsum);

/*    ROS_INFO_STREAM("rlength= " << rlength);*/
    //ROS_INFO_STREAM("glength= " << glength);
    /*ROS_INFO_STREAM("blength= " << blength);*/
    rgdot/= (rlength*glength);
    gbdot/= (glength*blength);
    rbdot/= (rlength*blength);


/*    ROS_INFO_STREAM("rgdot= " << rgdot);*/
    //ROS_INFO_STREAM("gbdot= " << gbdot);
    /*ROS_INFO_STREAM("rbdot= " << rbdot);*/



    //!< Compute the color angles
    double rgAngle = acos(rgdot);
    double gbAngle = acos(gbdot);
    double rbAngle = acos(rbdot);
/*    ROS_INFO_STREAM("rgAngle= " << rgAngle);*/
    //ROS_INFO_STREAM("gbAngle= " << gbAngle);
    /*ROS_INFO_STREAM("rbAngle= " << rbAngle);*/
    cv::Mat gray;
    //!< Normalised intensity standard deviation
    //!< Transform the src image to grayscale
    cvtColor( *_img, gray, CV_BGR2GRAY );

    //!< Compute the mean intensity value
    cv::Scalar meanI = mean(gray);


    //!< Find the maximum intensity value
    double maxVal, std, sum = 0;
    cv::minMaxLoc( gray, NULL, &maxVal );

    for(int ii = 0; ii < gray.rows; ii++)
      for(int jj = 0; jj < gray.cols; jj++)
      {
        sum+= pow((gray.at<uchar>(ii, jj) - meanI.val[0]), 2);
      }

    std = 2.0 / (maxVal * gray.cols * gray.rows) * sqrt(sum);

    //!< Construct the final feature vector
    colorAnglesAndStd.push_back(rgAngle);
    colorAnglesAndStd.push_back(gbAngle);
    colorAnglesAndStd.push_back(rbAngle);
    colorAnglesAndStd.push_back(std);

    return colorAnglesAndStd;
  }
}// namespace pandora_vision


