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
* Author: Aprilis George
* 		  Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_FACE_FACE_DETECTION_H 
#define PANDORA_VISION_FACE_FACE_DETECTION_H 

#include <iostream>
#include <stdlib.h>
#include <string>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "vision_communications/FaceDirectionMsg.h"
#include "pandora_vision_face/face_detector.h"
#include "pandora_vision_face/time_calculator.h"
#include "state_manager/state_client.h"

//!< Horizontal field of view in degrees
#define HFOV 61.14

//!< vertical field of view in degrees
#define VFOV 48

//!< default frame height
#define DEFAULT_HEIGHT 480

//!< default frame width
#define DEFAULT_WIDTH 640

namespace pandora_vision
{
class FaceDetection : public StateClient
{
private:

  //!< The NodeHandle
  ros::NodeHandle _nh;

  FaceDetector* _faceDetector;

  float ratioX;
  float ratioY;

  //!< Horizontal field of view in rad
  double hfov;

  //!< Vertical Field Of View (rad)
  double vfov;

  int frameWidth;
  int frameHeight;

  std::string cameraName;

  //!< Frame processed by FaceDetector
  cv::Mat faceFrame;

  //!<FaceDetector frame timestamp
  ros::Time faceFrameTimestamp;

  //!< Timer used for FaceCallaback
  ros::Timer faceTimer;

  //!< The topic subscribed to for the camera
  std::string imageTopic;
  std::string cameraFrameId;

  //!< time durations for every callback Timer
  double faceTime;

  //!< Publishers for FaceDetector result messages
  ros::Publisher _victimDirectionPublisher;

  //!< The subscriber that listens to the frame
  //!< topic advertised by the central node
  ros::Subscriber _frameSubscriber;

  //!< variables for changing in dummy msg mode for debugging
  bool faceDummy;

  //!< Variable used for State Managing
  bool faceNowON;


  //!< parameters for the FaceDetector:
  std::string cascade_path;
  std::string model_path;
  std::string model_url;
  int bufferSize;
  bool skinEnabled;

  //!< Paths for Skin Detector
  std::string skinHist;
  std::string wallHist;
  std::string wall2Hist;
  std::string packagePath;

  //!< Current state of robot
  int curState;
  //!< Previous state of robot
  int prevState;

  /**
   * @brief Get parameters referring to view and frame characteristics from
   * launch file
   * @return void
  */
  void getGeneralParams();

  /**
   * @brief Get parameters referring to facedetection algorithm
   * @return void
  */
  void getFaceParams();

  void getTimerParams();

  /**
   * @brief This method uses a FaceDetector instance to detect all
   * present faces in a given frame
   * @param timer [ros:TimerEvemt] the timer used to call
   * faceCallback
   * @return void
  */
  void faceCallback(const ros::TimerEvent&);

  /**
   * Function called when new ROS message appears, for front camera
   * @param msg [const sensor_msgs::Image&] The message
   * @return void
  */
  void imageCallback(const sensor_msgs::Image& msg);

public:

  //!< The Constructor
  FaceDetection();

  //!< The Destructor
  ~FaceDetection();

  void createFaceMessage(vision_communications::FaceDirectionMsg *faceMessage);
  void createDummyFaceMessage(float *center_x, float *center_y, 
    vision_communications::FaceDirectionMsg *faceMessage);

  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
  */
  void startTransition(int newState);

  /**
   * @brief After completion of state transition
   * @return void
  */
  void completeTransition(void);

};
}// namespace pandora_vision
#endif  // PANDORA_VISION_FACE_FACE_DETECTION_H


