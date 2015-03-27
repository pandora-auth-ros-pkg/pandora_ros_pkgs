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
  double VictimParameters::rgb_vj_weight = 0.2;
  double VictimParameters::depth_vj_weight = 0;
  double VictimParameters::rgb_svm_weight = 0.9;
  double VictimParameters::depth_svm_weight = 0;
  
  bool VictimParameters::debug_img = false;
  bool VictimParameters::debug_img_publisher = false;
  
  //!< Static parameters
  std::string VictimParameters::packagePath = "";
  std::string VictimParameters::victimAlertTopic = "";
  std::string VictimParameters::victimDebugImg = "";
  std::string VictimParameters::interpolatedDepthImg = "";
  std::string VictimParameters::enhancedHolesTopic = "";
  std::string VictimParameters::cameraName = "";
  int VictimParameters::frameHeight = 0;
  int VictimParameters::frameWidth = 0;
  int VictimParameters::modelImageHeight = 0;
  int VictimParameters::modelImageWidth = 0;
  double VictimParameters::vfov = 0.0;
  double VictimParameters::hfov = 0.0;
  
  std::string VictimParameters::cascade_path = "";
  std::string VictimParameters::model_url = "";
  std::string VictimParameters::model_path = "";
  std::string VictimParameters::rgb_classifier_path = "";
  std::string VictimParameters::depth_classifier_path = "";
  
  double VictimParameters::rgb_svm_C = 312.5;
  double VictimParameters::rgb_svm_gamma = 0.50625;
  double VictimParameters::rgb_svm_prob_scaling = 0.5;
  double VictimParameters::rgb_svm_prob_translation = 7.0;
  double VictimParameters::depth_svm_C = 312.5;
  double VictimParameters::depth_svm_gamma = 0.50625;
  double VictimParameters::depth_svm_prob_scaling = 0.5;
  double VictimParameters::depth_svm_prob_translation = 7.0;
  
  //----------------------------Methods----------------------------//
  
  VictimParameters::VictimParameters(const std::string& ns):
    _nh(ns)
  {
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&VictimParameters::parametersCallback,
        this, _1, _2));
        
    getGeneralParams();
    getVictimDetectorParameters();
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
    VictimParameters::rgb_vj_weight = config.rgb_vj_weight;
    VictimParameters::depth_vj_weight = config.depth_vj_weight;
    VictimParameters::rgb_svm_weight = config.rgb_svm_weight;
    VictimParameters::depth_svm_weight = config.depth_svm_weight;
    VictimParameters::debug_img = config.debug_img;
    VictimParameters::debug_img_publisher = config.debug_img_publisher;
    VictimParameters::rgb_svm_prob_scaling = config.rgb_svm_prob_scaling;
    VictimParameters::rgb_svm_prob_translation = 
      config.rgb_svm_prob_translation;
  }
  
  /**
  @brief Get parameters referring to the view and
  frame characteristics
  @return void
  **/
  void VictimParameters::getGeneralParams(void)
  {

    VictimParameters::packagePath =  
      ros::package::getPath("pandora_vision_victim");
    
    std::string str_param;
    int int_param;
    double double_param;
    
    if (_nh.getParam("published_topic_names/victim_debug_img", str_param))
    {
      VictimParameters::victimDebugImg = str_param;
    }
    else
    {
      ROS_FATAL("[victim_node] : victimDebugImg name param not found");
      ROS_BREAK();
    }
    
    if (_nh.getParam("published_topic_names/victim_interpolated_depth_img", 
      str_param))
    {
      VictimParameters::interpolatedDepthImg = str_param;
    }
    else
    {
      ROS_FATAL("[victim_node] : interpolatedDepthImg name param not found");
      ROS_BREAK();
    }
    
    if (_nh.getParam("published_topic_names/victim_alert", str_param))
    {
      VictimParameters::victimAlertTopic = str_param;
    }
    else
    {
      ROS_FATAL("[victim_node] : Victim alert topic name param not found");
      ROS_BREAK();
    }

    //! Declare subsciber's topic name
    if (_nh.getParam("subscribed_topic_names/enhanded_hole_alert", str_param))
    {
      ROS_INFO_STREAM("PARAM"<< str_param);
      VictimParameters::enhancedHolesTopic = str_param;
    }  
    else
    {
      ROS_FATAL("[victim_node] : Victim subscribed topic name param not found");
      ROS_BREAK();
    }  
    
        
    //!< Get the camera to be used by motion node;
    if (_nh.getParam("camera_name", str_param)) 
    {
      VictimParameters::cameraName = str_param;
      ROS_DEBUG_STREAM("camera_name : " << str_param);
    }
    else 
    {
      ROS_FATAL("[victim_node]: Camera name not found");
      ROS_BREAK(); 
    }

    //! Get the Height parameter if available;
    if (_nh.getParam("image_height", int_param)) 
    {
      VictimParameters::frameHeight = int_param;
      ROS_DEBUG_STREAM("height : " << int_param);
    }
    else 
    {
      ROS_FATAL
        ("[victim_node] : Parameter frameHeight not found. Using Default");
      ROS_BREAK();
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("image_width", int_param)) 
    {
      VictimParameters::frameWidth = int_param;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else 
    {
      ROS_FATAL
        ("[victim_node] : Parameter frameWidth not found. Using Default");
      ROS_BREAK();
    }
    
    //! Get the Height parameter if available;
    if (_nh.getParam("model_image_height", int_param)) 
    {
      VictimParameters::modelImageHeight = int_param;
      ROS_DEBUG_STREAM("model image height : " << int_param);
    }
    else 
    {
      ROS_FATAL
        ("[victim_node] : Parameter modelImageHeight not found. Using Default");
      ROS_BREAK();
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("model_image_width", int_param)) 
    {
      VictimParameters::modelImageWidth = int_param;
      ROS_DEBUG_STREAM("model image width : " << int_param);
    }
    else 
    {
      ROS_FATAL
        ("[victim_node] : Parameter modelImageWidth not found. Using Default");
      ROS_BREAK();
    }
  
    //!< Get the HFOV parameter if available;
    if (_nh.getParam("hfov", double_param))
    {
      VictimParameters::hfov = double_param;
      ROS_DEBUG_STREAM("HFOV : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: Horizontal field of view not found");
     ROS_BREAK();
    }
    
    //!< Get the VFOV parameter if available;
    if (_nh.getParam("vfov", double_param))
    { 
      VictimParameters::vfov = double_param;
      ROS_DEBUG_STREAM("VFOV : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: Vertical field of view not found");
     ROS_BREAK();
    }  
    
    if (_nh.getParam("rgb_svm_C", double_param))
    { 
      VictimParameters::rgb_svm_C = double_param;
      ROS_DEBUG_STREAM("rgb_svm_C : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: rgb_svm_C not found");
     ROS_BREAK();
    } 
     
    if (_nh.getParam("rgb_svm_gamma", double_param))
    { 
      VictimParameters::rgb_svm_gamma = double_param;
      ROS_DEBUG_STREAM("rgb_svm_gamma : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: rgb_svm_gamma not found");
     ROS_BREAK();
    }  
    
    if (_nh.getParam("depth_svm_C", double_param))
    { 
      VictimParameters::depth_svm_C = double_param;
      ROS_DEBUG_STREAM("depth_svm_C : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: depth_svm_C not found");
     ROS_BREAK();
    } 
     
    if (_nh.getParam("depth_svm_gamma", double_param))
    { 
      VictimParameters::depth_svm_gamma = double_param;
      ROS_DEBUG_STREAM("depth_svm_gamma : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: depth_svm_gamma not found");
     ROS_BREAK();
    }  
    
    if (_nh.getParam("depth_svm_prob_scaling", double_param))
    { 
      VictimParameters::depth_svm_prob_scaling = double_param;
      ROS_DEBUG_STREAM("depth_svm_prob_scaling : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: depth_svm_prob_scaling not found");
     ROS_BREAK();
    }  
    if (_nh.getParam("depth_svm_prob_translation", double_param))
    { 
      VictimParameters::depth_svm_prob_translation = double_param;
      ROS_DEBUG_STREAM("depth_svm_prob_translation : " << double_param);
    }
    else 
    {
     ROS_FATAL("[victim_node]: depth_svm_prob_translation not found");
     ROS_BREAK();
    }  
  }
  

  /**
  @brief Get parameters referring to the face detection algorithm
  @return void
  **/
  void VictimParameters::getVictimDetectorParameters(void)
  {
    
    std::string str_param;
    bool bool_param;
    int int_param;
    
    //!< Get the path of haar_cascade xml file if available;
    if ( _nh.getParam("cascade_path", str_param))
    {
      VictimParameters::cascade_path = VictimParameters::packagePath + 
        str_param;
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << str_param);
    }
    else
    {
      VictimParameters::cascade_path = VictimParameters::packagePath + 
        "/data/haarcascade_frontalface_alt_tree.xml";
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << 
        VictimParameters::cascade_path);
    }

    //!< Get the model.xml url;
    if (_nh.getParam("model_url", str_param))
    {
      VictimParameters::model_url = str_param;
      ROS_INFO_STREAM("[victim_node]: modelURL : " << str_param);
    }
    else{
      VictimParameters::model_url = 
        "https://pandora.ee.auth.gr/vision/model.xml";
      ROS_INFO_STREAM("[victim_node]: modelURL : " << 
        VictimParameters::model_url);
    }

    //!< Get the path of model_path xml file to be loaded
    if (_nh.getParam("model_path",  str_param))
    {
      VictimParameters::model_path = 
        VictimParameters::packagePath + str_param;
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  str_param);
    }
    else
    {
      VictimParameters::model_path = VictimParameters::packagePath + 
        "/data/model.xml";
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  
        VictimParameters::model_path);
    }
    
    //!< Get the path of rgb classifier
    if (_nh.getParam("rgb_classifier_path",  str_param))
    {
      VictimParameters::rgb_classifier_path = VictimParameters::packagePath + 
        str_param;
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
        <<  str_param);
    }
    else
    {
      VictimParameters::rgb_classifier_path = 
        VictimParameters::packagePath + "/data/rgb_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
        <<  VictimParameters::rgb_classifier_path);
    }
    
    //!< Get the path of depth classifier
    if (_nh.getParam("depth_classifier_path",  str_param))
    {
      VictimParameters::depth_classifier_path = VictimParameters::packagePath + 
        str_param;
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
        <<  str_param);
    }
    else
    {
      VictimParameters::depth_classifier_path = VictimParameters::packagePath +
        "/data/depth_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
        <<  VictimParameters::depth_classifier_path);
    }

  }
}// namespace pandora_vision
