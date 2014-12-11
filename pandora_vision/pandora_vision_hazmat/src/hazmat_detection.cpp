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
    //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";
    _camera_indicator = -1;
    
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&HazmatDetection::parametersCallback,
        this, _1, _2));
         
    /// Get General Parameters
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
            
    //!< Convert field of view from degrees to rads
    for(int ii= 0; ii < _hfov.size(); ii++){
      _hfov.at(ii) = _hfov.at(ii) * CV_PI / 180;
      _vfov.at(ii) = _vfov.at(ii) * CV_PI / 180;
    }
      
    for(int ii = 0; ii < _imageTopics.size(); ii++ ){
      //!< subscribe to input image's topic
      frameSubscriber = _nh.subscribe(
        _imageTopics.at(ii), 1, &HazmatDetection::imageCallback, this);
      _frameSubscribers.push_back(frameSubscriber);
    }
    //initialize states - robot starts in STATE_OFF 
    curState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    prevState = state_manager_msgs::RobotModeMsg::MODE_OFF;

    //initialize state Managing Variables
    hazmatNowOn_ = false;
    
    _lastTimeProcessed = ros::Time::now(); 
      
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
      <pandora_vision_msgs::HazmatAlertsVectorMsg>(param, 10);
    }
    else
    {
      ROS_FATAL("[Hazmat_node]: Hazmat alert topic name param not found");
      ROS_BREAK();
    }
    
    XmlRpc::XmlRpcValue cameras_list;
    _nh.getParam("camera_sensors", cameras_list);
    ROS_ASSERT(cameras_list.getType() == XmlRpc::XmlRpcValue::TypeArray); 

    std::string key;
    for (int ii = 0; ii < cameras_list.size(); ii++)
    {
      ROS_ASSERT(
        cameras_list[ii].getType() == XmlRpc::XmlRpcValue::TypeStruct);
      
      key = "name";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeString);
      cameraName = static_cast<std::string>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[Hazmat_node]: camera_name : " << cameraName);
      
      //!< Get the listener's topic for camera
      if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
        ROS_INFO_STREAM("[Hazmat_node]: imageTopic for camera : " << imageTopic);
      else
      {
       ROS_FATAL("[Hazmat_node]: Image topic name not found");
       ROS_BREAK(); 
      }
      _imageTopics.push_back("/"+imageTopic);
      
      key = "image_height";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      frameHeight = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[Hazmat_node]: image_height : " << frameHeight);
        
      _frameHeight.push_back(frameHeight);
        
      key = "image_width";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      frameWidth = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[Hazmat_node]: image_width : " << frameWidth);
        
      _frameWidth.push_back(frameWidth);  
      
      key = "hfov";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      hfov = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[Hazmat_node]: hfov : " << hfov);
        
      _hfov.push_back(hfov);  
      
      key = "vfov";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      vfov = static_cast<double>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[Hazmat_node]: vfov : " << vfov);
        
      _vfov.push_back(vfov);  
    
    }
     //!< Get the debugQrCode parameter if available;
    if (_nh.getParam("timerThreshold", _timerThreshold))
      ROS_DEBUG_STREAM("timerThreshold : " << _timerThreshold);
    else
    {
      _timerThreshold = 0.1;
      ROS_DEBUG_STREAM("timerThreshold : " << _timerThreshold);
    }
    
  }
  
  /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool HazmatDetection::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";
    bool res = _nh.hasParam(model_param_name);

    std::string robot_description = "";

    if(!res || !_nh.getParam(model_param_name, robot_description))
    {
      ROS_ERROR("[QrCode_node]:Robot description couldn't be retrieved from the parameter server.");
      return false;
    }
  
    boost::shared_ptr<urdf::ModelInterface> model(
      urdf::parseURDF(robot_description));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> currentLink = model->getLink(_frame_id);
    if(currentLink){
      boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();
      // Set the parent frame_id to the parent of the frame_id
      _parent_frame_id = parentLink->name;
      return true;
    }
    else
      _parent_frame_id = _frame_id;
      
    return false;
  }
  
  /**
  @brief Callback for a new image
  @param msg [const sensor_msgs::Image&] The new image
  @return void
  **/
  void HazmatDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    if( ros::Time::now() - _lastTimeProcessed < ros::Duration (_timerThreshold))
      return;
      
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat temp = in_msg->image.clone();
    hazmatFrame_ = new IplImage(temp);
    hazmatFrameTimestamp_ = msg.header.stamp;
    _frame_id = msg.header.frame_id;
    
     if(_frame_id.c_str()[0] == '/'){
      _frame_id = _frame_id.substr(1);
      _camera_indicator = 1;
    }  
      
    if ( hazmatFrame_.empty() )
    {               
      ROS_ERROR("[hazmatNode] : No more Frames");
      return;
    }
    
     std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
      
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
      bool _indicator = getParentFrameId();
      
      _frame_ids_map.insert( it , std::pair<std::string, std::string>(
         _frame_id, _parent_frame_id));
      
       for (it = _frame_ids_map.begin(); it != _frame_ids_map.end(); ++it)
          ROS_DEBUG_STREAM("" << it->first << " => " << it->second );
    } 
    
    _lastTimeProcessed = ros::Time::now(); 
     
    hazmatDetect();
  }

  /**
  @brief Method called only when a new image message is present
  @return void
  **/
  void HazmatDetection::hazmatDetect()
  {
    
    if( _camera_indicator == -1){
      frameWidth = _frameWidth.at(1);
      frameHeight = _frameHeight.at(1);
      hfov = _hfov.at(1);
      vfov = _vfov.at(1);
    }
    else{
      frameWidth = _frameWidth.at(0);
      frameHeight = _frameHeight.at(0);
      hfov = _hfov.at(0);
      vfov = _vfov.at(0);
    }
    
    cv::Mat allblack = cv::Mat( frameWidth, frameHeight, CV_8U );
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
    pandora_vision_msgs::HazmatAlertsVectorMsg hazmatVectorMsg;
    pandora_vision_msgs::HazmatAlertMsg hazmatMsg;
     
    std::vector<HazmatEpsilon> a = 
        hazmatDetector_->detectHazmat(hazmatFrame_);
    
    //if hazmat found
    if (a.size() > 0)
    {
      hazmatVectorMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
      hazmatVectorMsg.header.stamp = hazmatFrameTimestamp_;
      for (unsigned int i = 0; i < a.size() ; i++)
      {
        // Hazmat's center coordinates relative to the center of the frame
        float x = a[i].x
          - static_cast<float>(frameWidth) / 2;
        float y = static_cast<float>(frameHeight) / 2
          - a[i].y;

        // Hazmat center's yaw and pitch
        hazmatMsg.yaw = atan(2 * x / frameWidth * tan(hfov / 2));
        hazmatMsg.pitch = atan(2 * y / frameHeight * tan(vfov / 2));
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
        state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE ) ||
      ( curState == 
        state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION ) ||
      ( curState == 
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD ) ||
      ( curState == 
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST );
        
    if (curState == state_manager_msgs::RobotModeMsg::\
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




