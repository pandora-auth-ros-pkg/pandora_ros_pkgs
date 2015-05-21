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
* Authors: Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_VICTIM_PARAMETERS_H
#define PANDORA_VISION_VICTIM_VICTIM_PARAMETERS_H

#include <iostream>
#include <cstdlib>
#include <limits>
#include <map>
#include <vector>

#include "ros/ros.h"
#include <ros/package.h>

#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <urdf_parser/urdf_parser.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

#include "state_manager/state_client.h"
#include <pandora_vision_victim/victim_dyn_reconfConfig.h>

namespace pandora_vision
{

  enum VictimSource
  {
    RGB_VJ,
    DEPTH_VJ,
    RGB_SVM,
    DEPTH_RGB_SVM
  };

  enum DetectionMode
  {
    GOT_RGB = 1,
    GOT_HOLES_AND_DEPTH = 4,
    GOT_HOLES = 2,
    GOT_DEPTH = 3
  };

  struct EnhancedMat
  {
    cv::Mat img;
    cv::Rect bounding_box;
    cv::Point2f keypoint;
  };
  
  struct DetectionImages
  {
    // EnhancedMat rgb;
    // EnhancedMat depth;
    std::vector<EnhancedMat> rgbMasks;
    std::vector<EnhancedMat> depthMasks;
  };

  struct BoundingBox
  {
    cv::Rect bounding_box;
    cv::Point2f keypoint;
  };

  class VictimParameters
  {
    public:
      //!< Default contructor
      VictimParameters();

      //!< The dynamic reconfigure (motion's) parameters' server
      dynamic_reconfigure::Server
        <pandora_vision_victim::victim_dyn_reconfConfig>server;
      //!< The dynamic reconfigure (depth) parameters' callback
      dynamic_reconfigure::Server
        <pandora_vision_victim::victim_dyn_reconfConfig>::CallbackType f;

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
        @param[in] level [const uint32_t] The level
        @return void
      **/
      void parametersCallback(
        const pandora_vision_victim::victim_dyn_reconfConfig& config,
        const uint32_t& level);
        
      void configVictim(const ros::NodeHandle& nh);
    
    public:  
      //!< Weight parameters for the victim subsystems
      double rgb_vj_weight;
      double depth_vj_weight;
      double rgb_svm_weight;
      double depth_svm_weight;

      //!< Parameters for debug purposes
      bool debug_img;
      bool debug_img_publisher;

      //!< parameters referring to the view and frame characteristics
      std::string packagePath;
      std::string victimDebugImg;
      std::string interpolatedDepthImg;
      int modelImageHeight;
      int modelImageWidth;
      int positivesCounter;

      //!< parameters referring to the face detection algorithm
      std::string cascade_path;
      std::string model_url;
      std::string model_path;
      std::string rgb_classifier_path;
      std::string depth_classifier_path;

      /// parameters for svms
      double rgb_svm_C;
      double rgb_svm_gamma;
      double rgb_svm_prob_scaling;
      double rgb_svm_prob_translation;
      double depth_svm_C;
      double depth_svm_gamma;
      double depth_svm_prob_scaling;
      double depth_svm_prob_translation;
  };

}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_PARAMETERS_H
