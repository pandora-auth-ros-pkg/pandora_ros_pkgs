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

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
  //----------------------------Parameters----------------------------//
  //!< Dynamic reconfigure parameters
  double rgb_vj_weight = 0.2;
  double depth_vj_weight = 0;
  double rgb_svm_weight = 0.9;
  double depth_svm_weight = 0;
  
  bool debug_img = false;
  bool debug_img_publisher = false;
  
  double rgb_svm_prob_scaling = 0.5;
  double rgb_svm_prob_translation = 7.0;
  
  //----------------------------Methods----------------------------//
  
  VictimParameters::VictimParameters()
  {
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&VictimParameters::parametersCallback,
        this, _1, _2));
  }
  
  /**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
  @param[in] level [const uint32_t] The level 
  @return void
  **/
  void VictimParameters::parametersCallback(
    const pandora_vision_victim::victim_dyn_reconfConfig& config,
    const uint32_t& level)
  {
    rgb_vj_weight = config.rgb_vj_weight;
    depth_vj_weight = config.depth_vj_weight;
    rgb_svm_weight = config.rgb_svm_weight;
    depth_svm_weight = config.depth_svm_weight;
    debug_img = config.debug_img;
    debug_img_publisher = config.debug_img_publisher;
    rgb_svm_prob_scaling = config.rgb_svm_prob_scaling;
    rgb_svm_prob_translation = 
      config.rgb_svm_prob_translation;
  }
  
  void VictimParameters::configVictim(const ros::NodeHandle& nh)
  {
    packagePath =  
      ros::package::getPath("pandora_vision_victim");

    if (!nh.getParam("victim_interpolated_depth_img_topic", interpolatedDepthImg))
    {
      interpolatedDepthImg = "";
      ROS_FATAL("[victim_node] : interpolatedDepthImg name param not found");
      ROS_BREAK();
    }
    
    if (!nh.getParam("victim_debug_img_topic", victimDebugImg))
    {
      victimDebugImg = "";
      ROS_FATAL("[victim_node] : victimDebugImg name param not found");
      ROS_BREAK();
    }

    nh.param("model_image_height", modelImageHeight, 0);
    nh.param("model_image_width", modelImageWidth, 0);
    nh.param("rgb_svm_C", rgb_svm_C, 312.5);
    nh.param("rgb_svm_gamma", rgb_svm_gamma, 0.50625);
    nh.param("depth_svm_C", depth_svm_C, 312.5);
    nh.param("depth_svm_gamma", depth_svm_gamma, 0.50625);
    nh.param("depth_svm_prob_scaling", depth_svm_prob_scaling, 0.5);
    nh.param("depth_svm_prob_translation", depth_svm_prob_translation, 7.);
    
    if (!nh.getParam("cascade_path", cascade_path))
    {
      cascade_path = "/data/haarcascade_frontalface_alt_tree.xml";
      ROS_FATAL("[victim_node] : cascade_path name param not found");
      ROS_BREAK();
    }
    cascade_path = packagePath + cascade_path;
    
    if (!nh.getParam("model_path", model_path))
    {
      model_path = "/data/model.xml";
      ROS_FATAL("[victim_node] : model_path name param not found");
      ROS_BREAK();
    }
    model_path = packagePath + model_path;
    
    if (!nh.getParam("rgb_classifier_path", rgb_classifier_path))
    {
      rgb_classifier_path = "data/rgb_svm_classifier.xml";
      ROS_FATAL("[victim_node] : rgb_classifier_path name param not found");
      ROS_BREAK();
    }
    rgb_classifier_path = packagePath + rgb_classifier_path;
    
    if (!nh.getParam("depth_classifier_path", depth_classifier_path))
    {
      depth_classifier_path = "/data/depth_svm_classifier.xml";
      ROS_FATAL("[victim_node] : depth_classifier_path name param not found");
      ROS_BREAK();
    }
    depth_classifier_path = packagePath + depth_classifier_path;
    
    if (!nh.getParam("model_url", model_url))
    {
      model_url = "https://pandora.ee.auth.gr/vision/model.xml";
      ROS_FATAL("[victim_node] : model_url name param not found");
      ROS_BREAK();
    }
  }
}  // namespace pandora_vision
