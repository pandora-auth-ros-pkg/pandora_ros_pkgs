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

#ifndef RGB_NODE_HOLE_DETECTION_H
#define RGB_NODE_HOLE_DETECTION_H

#include <iostream>
#include <stdlib.h>

#include "ros/ros.h"
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "vision_communications/CandidateHolesVectorMsg.h"
#include "state_manager/state_client.h"
#include "utils/parameters.h"
#include "utils/message_conversions.h"
#include "rgb_node/hole_detector.h"

namespace pandora_vision
{
  class HoleDetection
  {
    private:

      //!< The NodeHandle
      ros::NodeHandle _nh;

      std::string packagePath;

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
      cv::Mat _holeFrame;

      //!<FaceDetector frame timestamp
      ros::Time _holeFrameTimestamp;

      //!< The topic subscribed to for the camera
      std::string imageTopic;
      std::string cameraFrameId;

      //!< The ROS subscriber for acquisition of the RGB image through the
      //depth sensor
      ros::Subscriber _frameSubscriber;

      //!< The ROS publisher ofcandidate holes
      ros::Publisher rgbCandidateHolesPublisher_;

      //!< Current state of robot
      int curState;
      //!< Previous state of robot
      int prevState;

      //!< Variable used for State Managing
      bool holeNowON;

      //!< Class HoleDetector instance that finds and locates tha position
      //!< potentional holes in current frame
      HoleDetector _holeDetector;

      //!< The dynamic reconfigure (RGB) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_detector::rgb_cfgConfig>
        server;

      //!< The dynamic reconfigure (RGB) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_detector::rgb_cfgConfig>::
        CallbackType f;

      /**
        @brief Get parameters referring to view and frame characteristics from
        launch file
        @return void
       **/
      void getGeneralParams();

      /**
        Function called when new ROS message appears, for front camera
        @param msg [const sensor_msgs::ImageConstPtr&] The message
        @return void
       **/
      void inputRgbImageCallback(const sensor_msgs::Image& inImage);

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole_detector::rgb_cfgConfig&]
        @param[in] level [const uint32_t] The level (?)
        @return void
       **/
      void parametersCallback(
        const pandora_vision_hole_detector::rgb_cfgConfig& config,
        const uint32_t& level);


    public:

      //!< The constructor
      HoleDetection();

      //!< The destructor
      virtual ~HoleDetection();
  };

} //namespace pandora_vision

#endif  // RGB_NODE_HOLE_DETECTION_H
