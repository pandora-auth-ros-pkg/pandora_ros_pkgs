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

#include "rgb_node/texture_filter.h"

namespace pandora_vision
{
  /**
    @brief Class constructor
   **/
  TextureDetector::TextureDetector()
  {
    getGeneralParams();

    pathToWalls = packagePath + "/walls/";

    //! Calculate histogramm according to a given set of images
    HistogramCalculation::getHistogram(1, &histogramm);

    ROS_INFO("[rgb_node]: Textrure detector instance created");
  }



  /**
    @brief Destructor
   **/
  TextureDetector::~TextureDetector()
  {
    ROS_INFO("[rgb_node]: Textrure detector instance deleted");
  }



  /**
    @brief Get parameters referring to view and frame characteristics
    from launch file
    @return void
   **/
  void TextureDetector::getGeneralParams()
  {
    #ifdef DEBUG_TIME
    Timer::start("getGeneralParams", "TextureDetector");
    #endif

    packagePath = ros::package::getPath("pandora_vision_hole_detector");

    //!< Get the Height parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_height"))
    {
      _nh.getParam("/" + cameraName + "/image_height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      frameHeight = 480;
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }

    //!< Get the Width parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_width"))
    {
      _nh.getParam("/" + cameraName + "/image_width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else
    {
      frameWidth = 640;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }

    #ifdef DEBUG_TIME
    Timer::tick("getGeneralParams");
    #endif
  }



  /**
    @brief Function for calculating applying backprojection in input image
    @param frame [cv::Mat] current frame to be processed
    @param backprojectedframe [cv::Mat*] image after backprojection is
    applied
    @return void
   **/
  void TextureDetector::applyBackprojection(cv::Mat* holeFrame,
    cv::Mat* backprojectedFrame)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyBackprojection", "applyTexture");
    #endif

    cv::Mat hsv;
    cvtColor(*holeFrame, hsv, CV_BGR2HSV);

    /// hue varies from 0 to 180
    float hranges[] = { 0, 180 };
    /// saturation varies from 0 to 80
    float sranges[] = { 0, 120 };

    const float* ranges[] = { hranges, sranges};
    int channels[] = {0, 1};
    cv::calcBackProject( &hsv, 1, channels , histogramm,
      *backprojectedFrame, ranges, 1, true );
    int const max_BINARY_value = 255;
    cv::Mat temp;
    backprojectedFrame->copyTo(temp);
    /// apply thresholds in backprojected image
    cv::threshold(temp, temp, 45, max_BINARY_value, 0);
    Morphology::dilation(&temp, 3, false);
    temp.copyTo(*backprojectedFrame);

    #ifdef DEBUG_TIME
    Timer::tick("applyBackprojection");
    #endif
  }



  /**
    @brief Function that applies backprogected image in current frame
    in order to find out which part of it belong to the given texture
    @param holeFrame [cv::Mat] the currrent frame to be processed
    @param backprojectedFrame [cv::Mat*] current frame after backprojection,
    this parameter is returned
    @return void
   **/
  void TextureDetector::applyTexture(cv::Mat* holeFrame,
    cv::Mat* backprojectedFrame)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyTexture", "findHoles");
    #endif

    cv::Mat backprojection = cv::Mat::zeros(480, 640, CV_8UC1);

    applyBackprojection(holeFrame, backprojectedFrame);

    cvtColor(*holeFrame, *holeFrame, CV_BGR2GRAY);

    bitwise_and(*holeFrame, *backprojectedFrame, backprojection);

    #ifdef DEBUG_TIME
    Timer::tick("applyTexture");
    #endif
  }



  /**
    @brief Function for debbuging reasons,shows histogramm and
    current frame after backprojection is applied
    @param holeFrame [cv::Mat] the currrent frame to be processed
    @param backprojection [cv::Mat] calculated backprojection
    @param backprojectedFrame [cv::Mat*] current frame after backprojection,
    this parameter is returned
    @return void
   **/
  void TextureDetector::debug_show(cv::Mat holeFrame,
    cv::Mat backprojection, cv::Mat backprojectedFrame)
  {
    ros::Time timeBegin = ros::Time::now();
    while( ros::Time::now()-timeBegin < ros::Duration(1))
    {
      cv::imshow(" Current frame", holeFrame);
      cv::imshow(" Backprojection", backprojection);
      cv::imshow(" Frame after backprojection ", backprojectedFrame);
      cv::waitKey(10);
    }
  }

} // namespace pandora_vision
