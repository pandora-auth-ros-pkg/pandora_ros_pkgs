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
* Authors:  Tsakalis Vasilis, Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_hazmat/hazmat_detection.h"

namespace pandora_vision
{
  /**
  @brief Default constructor
  @return void
  **/
  HazmatDetection::HazmatDetection(const std::string& ns) :_nh(ns)
  {
    
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&HazmatDetection::parametersCallback,
        this, _1, _2));
         
    // Get General Parameters, such as frame width & height , camera id
    getGeneralParams();

    //initialize hazmat detector
    hazmatDetector_ = new HazmatEpsilonDetector(packagePath_);
    
     hazmatDetector_->setHazmatParameters(
      HazmatParameters::colorVariance,
      static_cast<float>(HazmatParameters::votingThreshold),
      static_cast<float>(HazmatParameters::minAreaThreshold),
      static_cast<float>(HazmatParameters::maxAreaThreshold),
      HazmatParameters::sideLength,
      HazmatParameters::featureThreshold,
      static_cast<float>(HazmatParameters::MOThreshold)
    );
            
    //Convert field of view from degrees to rads
    ratioX_ = hfov_ / frameWidth_;
    ratioY_ = vfov_ / frameHeight_;

    hazmatFrame_ = cv::Mat( frameWidth_, frameHeight_, CV_8U );
      
    //subscribe to input image's topic
    sub_ = _nh.subscribe
      (imageTopic_, 1, &HazmatDetection::imageCallback, this);

    //initialize states - robot starts in STATE_OFF 
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    //initialize state Managing Variables
    hazmatNowOn_ = false;
      
    clientInitialize();
      
    ROS_INFO("[hazmat_node] : Created Hazmat Detection instance");
  }

  /**
  @brief Default destructor
  @return void
  **/
  HazmatDetection::~HazmatDetection(void)
  {
    ROS_INFO("[hazmat_node] : Destroying Hazmat Detection instance");
    delete hazmatDetector_;
  }

  /**
  @brief Reads the general parameters from the launch file
  @return void
  **/
  void HazmatDetection::getGeneralParams(void)
  {
    packagePath_ = ros::package::getPath("pandora_vision_hazmat");
    
    //! Publishers
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    if (_nh.getParam("published_topic_names/hazmat_alert", param))
    {
    hazmatPublisher_ = _nh.advertise
      <vision_communications::HazmatAlertsVectorMsg>(param, 10);
    }
    else
    {
      ROS_FATAL("Hazmat alert topic name param not found");
      ROS_BREAK();
    }
      
    
    //!< Get the camera to be used by qr node;
    if (_nh.getParam("camera_name", cameraName)) 
    {
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }
    else 
    {
      ROS_FATAL("Camera name not found");
      ROS_BREAK(); 
    }

    //! Get the Height parameter if available;
    if (_nh.getParam("/" + cameraName + "/image_height", frameHeight_)) 
    {
      ROS_DEBUG_STREAM("height : " << frameHeight_);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
      frameHeight_ = DEFAULT_HEIGHT;
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("/" + cameraName + "/image_width", frameWidth_)) 
    {
      ROS_DEBUG_STREAM("width : " << frameWidth_);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter frameWidth not found. Using Default");
      frameWidth_ = DEFAULT_WIDTH;
    }
    
    //! Get the images's topic;
    if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic_)) 
    {
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic_);
    }
    else 
    {
      ROS_FATAL("Camera name not found");
      ROS_BREAK();
    }

    //! Get the images's frame_id;
    if (_nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId)) 
    {
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }
    else 
    {
     ROS_FATAL("Camera name not found");
     ROS_BREAK();
    }
  
  }

  /**
  @brief Callback for a new image
  @param msg [const sensor_msgs::Image&] The new image
  @return void
  **/
  void HazmatDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat temp = in_msg->image.clone();
    hazmatFrame_ = new IplImage(temp);
    hazmatFrameTimestamp_ = msg.header.stamp;
    
    if ( hazmatFrame_.empty() )
    {               
      ROS_ERROR("[hazmatNode] : No more Frames");
      return;
    }
    hazmatCallback();
  }

  /**
  @brief Method called only when a new image message is present
  @return void
  **/
  void HazmatDetection::hazmatCallback()
  {
    cv::Mat allblack = cv::Mat( frameWidth_, frameHeight_, CV_8U );
    if(!hazmatNowOn_)
    {
      return;
    }
    
    if(std::equal (hazmatFrame_.begin<uchar>(), hazmatFrame_.end<uchar>(), 
      allblack.begin<uchar>() ))
    {
      return;
    }
    // Create Msg for hazmat
    vision_communications::HazmatAlertsVectorMsg hazmatVectorMsg;
    vision_communications::HazmatAlertMsg hazmatMsg;
     
    std::vector<HazmatEpsilon> a = 
        hazmatDetector_->DetectHazmatEpsilon(hazmatFrame_);

    //if hazmat found
    if (a.size() > 0)
    {
      hazmatVectorMsg.header.frame_id = cameraFrameId;
      hazmatVectorMsg.header.stamp = hazmatFrameTimestamp_;
      for (unsigned int i = 0; i < a.size() ; i++)
      {
        hazmatMsg.yaw = ratioX_ * ( a[i].x - frameWidth_ / 2 );
        hazmatMsg.pitch = - ratioY_ * ( a[i].y - frameHeight_ / 2 );
        hazmatMsg.patternType = a[i].pattern_num;
        hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
              
        ROS_INFO("[hazmatNode] : Hazmat found!");
      }
      if(hazmatVectorMsg.hazmatAlerts.size() > 0)
      {
        hazmatPublisher_.publish(hazmatVectorMsg);
      }
    }
    a.erase(a.begin(), a.end());
  }

  /**
  @brief Implemented from state manager. Called when a new transition \
  happens
  @param newState [int] The new state of the system
  @return void
  **/
  void HazmatDetection::startTransition(int newState){
    
    curState = newState;
      
    //check if each algorithm should be running now
    hazmatNowOn_ = 
      ( curState == 
        state_manager_communications::robotModeMsg::MODE_EXPLORATION ) ||
      ( curState == 
        state_manager_communications::robotModeMsg::MODE_IDENTIFICATION ) ||
      ( curState == 
        state_manager_communications::robotModeMsg::MODE_ARM_APPROACH ) ||
      ( curState == 
        state_manager_communications::robotModeMsg::\
          MODE_TELEOPERATED_LOCOMOTION )
      ||( curState == 
        state_manager_communications::robotModeMsg::MODE_DF_HOLD );
        
    if (curState == state_manager_communications::robotModeMsg::\
      MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState = curState;

    transitionComplete(curState);
  }

  /**
  @brief Called when the transition completes
  @return void
  **/
  void HazmatDetection::completeTransition(void)
  {
    ROS_INFO("[hazmatNode] : Transition Complete");
  }
  
  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
  **/
  void HazmatDetection::parametersCallback(
    const pandora_vision_hazmat::hazmat_cfgConfig& config,
    const uint32_t& level)
  {
    HazmatParameters::colorVariance = config.colorVariance;
    HazmatParameters::sideLength = config.sideLength;
    HazmatParameters::maxAreaThreshold = config.maxAreaThreshold;
    HazmatParameters::minAreaThreshold = config.minAreaThreshold;
    HazmatParameters::votingThreshold = config.votingThreshold;
    HazmatParameters::MOThreshold = config.MOThreshold;
    HazmatParameters::featureThreshold = config.featureThreshold;
    HazmatParameters::numberOfHazmats = config.numberOfHazmats;;
  }
  
} // namespace pandora_vision




