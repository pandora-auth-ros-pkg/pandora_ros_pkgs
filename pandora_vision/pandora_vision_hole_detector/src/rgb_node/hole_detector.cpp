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
#include "rgb_node/hole_detector.h"

namespace pandora_vision
{
  /**
    @brief Class constructor
    */
  HoleDetector::HoleDetector()
  {
    ROS_INFO("[rgb_node]: HoleDetector instance created");
  }



  /**
    @brief Class destructor
    */
  HoleDetector::~HoleDetector()
  {
    ROS_INFO("[rgb_node]: HoleDetector instance destroyed");
  }



  /**
    @brief Function that locates the position of potentional holes
    in current frame.
    @param holeFrame [cv::Mat] current frame to be processed
    @return void
    */
  HolesConveyor HoleDetector::findHoles(cv::Mat holeFrame)
  {
    #ifdef DEBUG_TIME
    Timer::start("findHoles", "inputRgbImageCallback");
    #endif

    //! Find pixels in current frame where there is the same texture
    //! according to the given histogramm and calculate
    std::vector<cv::KeyPoint> detectedkeyPoints;
    cv::Mat temp, backprojectedFrame;

    #ifdef SHOW_DEBUG_IMAGE
    std::string msg;
    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;
    #endif

    #ifdef SHOW_DEBUG_IMAGE
    cv::Mat before_blur;
    holeFrame.copyTo(before_blur);
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : Initial RGB image";
    msgs.push_back(msg);
    imgs.push_back(before_blur);
    #endif

    backprojectedFrame = cv::Mat::zeros(holeFrame.size().height,
      holeFrame.size().width, CV_8UC1);

    holeFrame.copyTo(temp);

    cv::Mat after_blur;
    holeFrame.copyTo(after_blur);
    cv::blur(holeFrame, after_blur, cv::Size(7,7));

    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : After blur";
    msgs.push_back(msg);
    imgs.push_back(after_blur);
    #endif

    //! backprojection of current frame
    _textureDetector.applyTexture(&holeFrame, &backprojectedFrame);

    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : After texture";
    msgs.push_back(msg);
    imgs.push_back(backprojectedFrame);
    #endif

    //! Apply in current frame Canny edge detection algorithm
    EdgeDetection::applySobel(backprojectedFrame, &temp);

    EdgeDetection::denoiseEdges(&temp);


    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += " : After denoising";
    msgs.push_back(msg);
    imgs.push_back(temp);
    #endif

    BlobDetection::detectBlobs(temp, &detectedkeyPoints);


    //!< The final vectors of keypoints, rectangles and blobs' outlines.
    struct HolesConveyor conveyor;

    HoleFilters::validateBlobs(
      &detectedkeyPoints,
      &temp,
      Parameters::bounding_box_detection_method,
      &conveyor);


    #ifdef SHOW_DEBUG_IMAGE
    msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
    msg += STR(" : Blobs");
    msgs.push_back(msg);
    imgs.push_back(
      Visualization::showHoles(
        msg,
        before_blur,
        -1,
        conveyor.keyPoints,
        conveyor.rectangles,
        std::vector<std::string>(),
        conveyor.outlines)
      );
    #endif

    #ifdef SHOW_DEBUG_IMAGE
    //Visualization::multipleShow("RGB node",imgs,msgs,800,1);
    #endif

    #ifdef DEBUG_TIME
    Timer::tick("findHoles");
    #endif

    return conveyor;
  }

} // namespace pandora_vision