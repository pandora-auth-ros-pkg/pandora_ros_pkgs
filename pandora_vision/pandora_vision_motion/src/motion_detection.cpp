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
* Author:  Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_motion/motion_detection.h"
namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  MotionDetection::MotionDetection(const std::string& ns) : _nh(ns)
  {
  
    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();
   
    motionFrame = cv::Mat(cv::Size(frameWidth, frameHeight), CV_8UC3);
    
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&MotionDetection::parametersCallback,
        this, _1, _2));
        
    //!< Subscribe to input image's topic
    _frameSubscriber = _nh.subscribe(imageTopic , 1,
        &MotionDetection::imageCallback, this );
    
    //!< Initialize states - robot starts in STATE_OFF 
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();
    
    motionNowON = false;  
    
    ROS_INFO("[Motion_node] : Created Motion Detection instance");
  }

  /**
    @brief Destructor
  */
  MotionDetection::~MotionDetection()
  {
    ROS_INFO("[Motion_node] : Destroying Motion Detection instance");
  }

  /**
   @brief Get parameters referring to view and frame characteristics from
   launch file
   @return void
  */
  void MotionDetection::getGeneralParams()
  { 
    //! Publishers
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    if (_nh.getParam("published_topic_names/motion_alert", param))
    {
    _motionPublisher = 
      _nh.advertise<vision_communications::MotionMsg>(param, 10);
    }
    else
    {
      ROS_FATAL("Motion alert topic name param not found");
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
    if (_nh.getParam("/" + cameraName + "/image_height", frameHeight)) 
    {
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
      frameHeight = DEFAULT_HEIGHT;
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("/" + cameraName + "/image_width", frameWidth)) 
    {
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter frameWidth not found. Using Default");
      frameWidth = DEFAULT_WIDTH;
    }
    
    //! Get the images's topic;
    if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic)) 
    {
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
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
   @brief Function called when new ROS message appears, for front camera
   @param msg [const sensor_msgs::Image&] The message
   @return void
  */
  void MotionDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    motionFrame = in_msg->image.clone();
    motionFrameTimestamp = msg.header.stamp;

    if ( motionFrame.empty() )
    {               
      ROS_ERROR("[motion_node] : No more Frames or something went wrong with bag file");
      return;
    }
    
    motionCallback();
  }


  /**
   @brief This method uses a MotionDetector instance to detect motion
   in current frame.
   @return void
  */
  void MotionDetection::motionCallback()
  {
    if(!motionNowON)
    {
      return;
    }
    //!< Create message of Motion Detector
    vision_communications::MotionMsg motionMessage;
    switch (_motionDetector.detectMotion(motionFrame))
    {
      case 0:
        motionMessage.probability = 0;
        break;
      case 1:
        motionMessage.probability = 0.51;
        break;
      case 2:
        motionMessage.probability = 1;
        break;
      default:
        motionMessage.probability = -1;
        ROS_INFO("Unable to get frame for motion detection");
        break;
    }
    if(motionMessage.probability > 0.1)
    {
      motionMessage.header.frame_id = cameraFrameId;
      motionMessage.header.stamp = ros::Time::now();
      ROS_INFO_STREAM( "[Motion_node] :Motion found with probability: "<< motionMessage.probability);
      _motionPublisher.publish(motionMessage);
    }
  }


  /**
   @brief Node's state manager
   @param newState [int] The robot's new state
   @return void
  */
  void MotionDetection::startTransition(int newState)
  {
    curState = newState;
    //!< Check if motion algorithm should be running now
    motionNowON = ( curState == 
      state_manager_communications::robotModeMsg::MODE_DF_HOLD );

    //!< Shutdown if the robot is switched off
    if (curState == 
      state_manager_communications::robotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState = curState;

    transitionComplete(curState); 
  }

  /**
   @brief After completion of state transition
   @return void
   */
  void MotionDetection::completeTransition(void)
  {
    ROS_INFO("[motion_node] : Transition Complete");
  }
  
  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
    @param[in] level [const uint32_t] The level 
    @return void
  **/
  void MotionDetection::parametersCallback(
    const pandora_vision_motion::motion_cfgConfig& config,
    const uint32_t& level)
  {
    //!< Background segmentation parameters
    MotionParameters::history = config.history;
    MotionParameters::varThreshold = config.varThreshold;
    MotionParameters::bShadowDetection = config.bShadowDetection;
    MotionParameters::nmixtures = config.nmixtures;
    
    //!< Threshold parameters
    MotionParameters::diff_threshold = config.diff_threshold;
    MotionParameters::motion_high_thres= config.motion_high_thres;
    MotionParameters::motion_low_thres= config.motion_low_thres;
    
  }
}// namespace pandora_vision

