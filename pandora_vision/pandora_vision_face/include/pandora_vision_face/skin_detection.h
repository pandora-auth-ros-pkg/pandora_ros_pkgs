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
* Author:  Skartados Evangelos
*********************************************************************/

#ifndef PANDORA_VISION_FACE_SKIN_DETECTION_H
#define PANDORA_VISION_FACE_SKIN_DETECTION_H
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "state_manager/state_client.h"
#include "vision_communications/victimIdentificationDirectionMsg.h"
#include "pandora_vision_face/skin_detector.h"
#include "pandora_vision_face/time_calculator.h"

//!< Horizontal field of view in degrees
#define HFOV 61.14
//!< Vertical field of view in degrees
#define VFOV 48
//!< Default frame height
#define DEFAULT_HEIGHT 480
//!< Default frame width
#define DEFAULT_WIDTH 640

namespace pandora_vision
{
class SkinDetection : public StateClient
{
private:

  //nodeHandle
  ros::NodeHandle _nh;
  SkinDetector* _skinDetector;
  float ratioX;
  float ratioY;

  float hfov; //horizontal Field Of View (rad)
  float vfov;
  int frameWidth; //frame width
  int frameHeight; //frame height

  cv::Mat skinFrame; // frame processed by SkinDetector
  cv::Mat extraFrame; // frame processed by SkinDetector

  ros::Time skinFrameTimestamp; // SkinDetector frame timestamp
  ros::Timer skinTimer; // Timer for frame callback

  string imageTopic;

  //time durations for every callback Timer in spin() function
  double skinTime;

  ros::Publisher _victimDirectionPublisher;

  //the subscriber that listens to the frame topic advertised by the usbCamNode
  image_transport::Subscriber _frameSubscriber;

  //debug publishers for SkinDetector
  image_transport::Publisher _skinSourcePublisher;
  image_transport::Publisher _skinResultPublisher;

  // variables for changing in dummy msg mode for debugging
  bool skinDummy;
  // variables for changing in debug mode. Publish images for debugging
  bool debugSkin;

  //variable used for State Managing
  bool skinNowON;

public:

  //!< constructor
  SkinDetection();

  //!< destructor
  virtual ~SkinDetection();

  /**
    @brief Get parameters referring to the view and
    frame characteristics
    @return void
  */
  void getGeneralParams();

  /**
    @brief Get parameters referring to the skin detection algorithm
    @return void
  */
  void getSkinParams();

  /**
    @brief Get parameters referring to the timer
    @return void
  */
  void getTimerParams();

  /**
    @brief Get paths referring to the skin detection algorithm
    @return void
  */
  void getSkinPaths();

  /**
    @brief This method uses a FaceDetector instance to detect all
    present faces in a given frame
    @return void
  */
  void skinCallback(const ros::TimerEvent&);

  /**
   @brief Function called when new ROS message appears, for camera
   @param msg [const sensor_msgs::ImageConstPtr&] The message
   @return void
  */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
    @brief Node's state manager
    @param newState [int] The robot's new state
    @return void
  */
  void startTransition(int newState);

  /**
    @brief After completion of state transition
    @return void
  */
  void completeTransition(void);

  int curState; //Current state of robot
  int prevState; //Previous state of robot
};
}// namespace pandora_vision
#endif  // PANDORA_VISION_FACE_SKIN_DETECTION_H


