/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/
#ifndef PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H
#define PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H

#include "ros/ros.h"
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdlib.h>

#include "pandora_vision_landoltc/landoltc_detector.h"
//!< default frame height
#define DEFAULT_HEIGHT 480

//!< default frame width
#define DEFAULT_WIDTH 640

namespace pandora_vision
{
class LandoltCDetection
{
private:
  //!<Subscriber of RGB Image
  ros::Subscriber _inputImageSubscriber;
  //!<Node Handler
  ros::NodeHandle _nh;
  //!< Current frame to be processed
  cv::Mat landoltCFrame;

  LandoltCDetector _landoltcDetector;
  std::string packagePath;

  std::string patternPath;

  int frameHeight;
  int frameWidth;

  std::string cameraName;
  std::string cameraFrameId;

  //!< The topic subscribed to for the front camera
  std::string imageTopic;

  /**
  @brief Callback for the RGB Image
  @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
  @return void
  **/
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
  @brief main function called for publishing messages in
    data fusion. In this function an object of class landoltcDetector
    is created to identify landoltCs in current frame.
  @param void
  @return void
  **/
  void landoltcCallback();

public:

  /**
  @brief Default Constructor
  @param ref [cv::Mat&] Reference Image
  @return void
  **/
  LandoltCDetection();

  /**
  @brief Default Destructor
  @return void
  **/
  ~LandoltCDetection();

  /**
  @brief Get parameters referring to view and frame characteristics
  @return void
  **/
  void getGeneralParams();


};
} // namespace pandora_vision
#endif  // PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H
