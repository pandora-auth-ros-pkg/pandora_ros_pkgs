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
  HazmatDetection::HazmatDetection(void) :nh_()
  {

    // Get General Parameters, such as frame width & height , camera id
    getGeneralParams();

    //initialize hazmat detector
    hazmatDetector_ = new HazmatEpsilonDetector(packagePath_);	
  
    //Get HazmatDetector Parameters
    getHazmatParams();
    
    //Convert field of view from degrees to rads

    ratioX_ = hfov_ / frameWidth_;
    ratioY_ = vfov_ / frameHeight_;

    hazmatFrame_ = cv::Mat( frameWidth_, frameHeight_, CV_8U );
      
    // Declare publisher and advertise topic where algorithm results are posted
    hazmatPublisher_ = nh_.advertise
      <vision_communications::HazmatAlertsVectorMsg>("hazmat_alert", 10);

    //subscribe to input image's topic
    sub_ = image_transport::ImageTransport(nh_).subscribe
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
  
    if (nh_.hasParam("hazmatDummy"))
    {
      nh_.getParam("hazmatDummy_", hazmatDummy_);
      ROS_DEBUG("hazmatDummy_ : %d", hazmatDummy_);
    }
    else 
    {
      ROS_DEBUG("[webNode] : Parameter hazmatDummy_ not found. Using Default");
      hazmatDummy_ = false;
    }
  
    //get the store location of the image
    if (nh_.hasParam("saveImagePath_"))
    {
      nh_.getParam("saveImagePath_", saveImagePath_);
      ROS_DEBUG_STREAM("path : " << saveImagePath_);
    }
    else
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter saveImagePath_ not found. Using Default");
      saveImagePath_ = "/home/pandora/Desktop/eyeCharts/";
    }
  
  //!< Get the camera to be used by qr node;
  if (nh_.hasParam("camera_name")) {
    nh_.getParam("camera_name", cameraName);
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  }
  else {
    ROS_DEBUG("[hazmat_node] : Parameter frameHeight not found. Using Default");
    cameraName = "camera";
  }
  //!< Get the Height parameter if available;
  if (nh_.hasParam("/" + cameraName + "/image_height"))
  {
  nh_.getParam("/" + cameraName + "/image_height", frameHeight_);
    ROS_DEBUG_STREAM("height : " << frameHeight_);
  }
  else
  {
    ROS_DEBUG("[hazmat_node] : \
      Parameter frameHeight not found. Using Default");
  }

  //!< Get the Width parameter if available;
  if (nh_.hasParam("/" + cameraName + "/image_width"))
  {
    nh_.getParam("/" + cameraName + "/image_width", frameWidth_);
    ROS_DEBUG_STREAM("width : " << frameWidth_);
  }
  else
  {
    ROS_DEBUG("[hazmat_node] : Parameter frameWidth not found. Using Default");
    frameWidth_ = DEFAULT_WIDTH;
  }

  //!< Get the listener's topic;
  if (nh_.hasParam("/" + cameraName + "/topic_name"))
  {
    nh_.getParam("/" + cameraName + "/topic_name", imageTopic_);
  }
  else 
  {
    ROS_DEBUG("[hazmat_node] : Parameter imageTopic not found. Using Default");
    imageTopic_ = "/camera_head/image_raw";
  }

  //!< Get the images's frame_id;
  if (nh_.hasParam("/" + cameraName + "/camera_frame_id")) {
    nh_.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
    ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
  }
  else 
  {
    ROS_DEBUG("[hazmat_node] : Parameter camera_frame_id not found. Using Default");
    cameraFrameId = "/camera";
  }
  
  //!< Get the HFOV parameter if available;
  if (nh_.hasParam("/" + cameraName + "/hfov")) {
    nh_.getParam("/" + cameraName + "/hfov", hfov_);
    ROS_DEBUG_STREAM("HFOV : " << hfov_);
  }
  else {
    ROS_DEBUG("[hazmat_node] : Parameter frameWidth not found. Using Default");
    hfov_ = HFOV;
  }
  
  //!< Get the VFOV parameter if available;
  if (nh_.hasParam("/" + cameraName + "/vfov")) {
    nh_.getParam("/" + cameraName + "/vfov", vfov_);
    ROS_DEBUG_STREAM("VFOV : " << vfov_);
  }
  else {
    ROS_DEBUG("[hazmat_node] : Parameter frameWidth not found. Using Default");
    vfov_ = VFOV;
  }
  
  }

  /**
  @brief Reads the hazmat - specific parameters from the launch file
  @return void
  **/
  void HazmatDetection::getHazmatParams(void)
  {
    // Get the test parameter if available;
    int colorVariance;
    if (nh_.hasParam("colorVariance")) 
    {
      nh_.getParam("colorVariance", colorVariance);
    }
    else 
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter colorVariance not found. Using Default");
      colorVariance = 10;
    }

    double votingThreshold;
    if (nh_.hasParam("votingThreshold")) 
    {
      nh_.getParam("votingThreshold", votingThreshold);
    }
    else 
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter votingThreshold not found. Using Default");
      votingThreshold = 39900;
    }

    //get the minimum area threshold of the hazmat in the image
    double minAreaThreshold;
    if (nh_.hasParam("minAreaThreshold")) 
    {
      nh_.getParam("minAreaThreshold", minAreaThreshold);
    }
    else 
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter minAreaThreshold not found. Using Default");
      minAreaThreshold = 1000;
    }

    //get the maximum area threshold of the hazmat in the image
    double maxAreaThreshold;
    if (nh_.hasParam("maxAreaThreshold")) 
    {
      nh_.getParam("maxAreaThreshold", maxAreaThreshold);
    }
    else 
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter maxAreaThreshold not found. Using Default");
      maxAreaThreshold = 100000;
    }

    //get the sidelenght parameter of the rectangle in which to test for colour
    int sideLength;
    if (nh_.hasParam("sideLength")) 
    {
      nh_.getParam("sideLength", sideLength);
    }
    else 
    {
      ROS_DEBUG("[hazmatNode] : Parameter sideLength not found. Using Default");
      //sideLength = 200
      sideLength = 100;
    }

    //get the minimum number of features threshold
    int featureThreshold;
    if (nh_.hasParam("featureThreshold")) 
    {
      nh_.getParam("featureThreshold", featureThreshold);
    }
    else 
    {
      ROS_DEBUG
        ("[hazmatNode] : Parameter featureThreshold not found. Using Default");
      featureThreshold = 20;
    }

    //how many hazmats i have to search for
    int hazmatNumber_;
    if (nh_.hasParam("hazmatNumber_")) 
    {
      nh_.getParam("hazmatNumber_", hazmatNumber_);
    }
    else 
    {
      ROS_DEBUG(
        "[hazmatNode] : Parameter hazmatNumber_ not found. Using Default");
      hazmatNumber_ = 9;
    }

    //get the MO threshold
    double MOThreshold;
    if (nh_.hasParam("MOThreshold")) 
    {
      nh_.getParam("MOThreshold", MOThreshold);
    }
    else 
    {
      ROS_DEBUG(
        "[hazmatNode] : Parameter MOThreshold not found. Using Default");
      MOThreshold = 120000;
    }

    hazmatDetector_->setHazmatParameters(
      colorVariance,
      static_cast<float>(votingThreshold),
      static_cast<float>(minAreaThreshold),
      static_cast<float>(maxAreaThreshold),
      sideLength,
      featureThreshold,
      static_cast<float>(MOThreshold)
    );

  }

  /**
  @brief Callback for a new image
  @param msg [const sensor_msgs::ImageConstPtr&] The new image
  @return void
  **/
  void HazmatDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //update image contents
    //sensor_msgs::CvBridge bridge;
    //hazmatFrame = bridge.imgMsgToCv(msg, "bgr8");
    //hazmatFrameTimestamp_ = msg->header.stamp;
    
    cv_bridge::CvImagePtr in_msg;
    
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    cv::Mat temp = in_msg->image.clone();
    hazmatFrame_ = new IplImage(temp);
    hazmatFrameTimestamp_ = msg->header.stamp;
    
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

    if (hazmatDummy_)
    {
      /*
       * Dummy Hazmat Message
       */
      hazmatVectorMsg.header.frame_id = "Hazmat";
      hazmatVectorMsg.header.stamp = ros::Time::now();
      for (int i = 0 ; i < 3 ; i++)
      {
        hazmatMsg.yaw = 0;
        hazmatMsg.pitch = 0;
        hazmatMsg.patternType = 0;

        hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
      }
      if(hazmatVectorMsg.hazmatAlerts.size() > 0)
      {
        hazmatPublisher_.publish(hazmatVectorMsg);
      }

      //dummy delay
      usleep(1000 * 2000);
    }
    else
    {
      //
      // Hazmat Message
      //

      // run hazmat detector
    
      std::vector<HazmatEpsilon> a = 
        hazmatDetector_->DetectHazmatEpsilon(hazmatFrame_);

      //if hazmat found
      if (a.size() > 0)
      {
        hazmatVectorMsg.header.frame_id = cameraFrameId;
        hazmatVectorMsg.header.stamp = hazmatFrameTimestamp_;
        for (unsigned int i = 0; i < a.size() ; i++)
        {
          //hazmat message information
          hazmatMsg.yaw = ratioX_ * ( a[i].x - frameWidth_ / 2 );
          hazmatMsg.pitch = - ratioY_ * ( a[i].y - frameHeight_ / 2 );
          hazmatMsg.patternType = a[i].pattern_num;
          //add the message to vector
          hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
                
          ROS_INFO("[hazmatNode] : Hazmat found!");
          //check if eye chart
          if (a[i].pattern_num >hazmatNumber_)
          {
            std::stringstream ss;
            //save Image to the desired location
            ss << saveImagePath_ << hazmatFrameTimestamp_ << ".jpg";
            imwrite(ss.str().c_str(), hazmatFrame_);
          }
        }
        if(hazmatVectorMsg.hazmatAlerts.size() > 0)
        {
          hazmatPublisher_.publish(hazmatVectorMsg);
        }

      }
      a.erase(a.begin(), a.end());
    }
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
} // namespace pandora_vision




