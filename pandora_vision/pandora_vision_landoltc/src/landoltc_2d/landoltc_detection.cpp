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
* Author: Victor Daropoulos
*********************************************************************/

#include "pandora_vision_landoltc/landoltc_2d/landoltc_detection.h"

namespace pandora_vision
{

/**
@brief Default Constructor
@param void
@return void
**/

LandoltCDetection::LandoltCDetection(const std::string& ns) : _nh(ns), landoltcNowON(false)
{
  //!< Set initial value of parent frame id to null
  _parent_frame_id = "";
  _frame_id = "";
    
  getGeneralParams();

  //!< Convert field of view from degrees to rads
  hfov = hfov * CV_PI / 180;
  vfov = vfov * CV_PI / 180;
  
  //!< Initiliaze and preprocess reference image
  _landoltcDetector.initializeReferenceImage(patternPath);

  _inputImageSubscriber = _nh.subscribe(imageTopic, 1,
                                        &LandoltCDetection::imageCallback, this);
  
  //!< The dynamic reconfigure parameter's callback
  server.setCallback(boost::bind(&LandoltCDetection::parametersCallback, this, _1, _2));
  
  //!< initialize states - robot starts in STATE_OFF
  curState = state_manager_communications::robotModeMsg::MODE_OFF;
  prevState = state_manager_communications::robotModeMsg::MODE_OFF;

  clientInitialize();
  
  _lastTimeProcessed = ros::Time::now();    
      
  ROS_INFO("[landoltc_node] : Created LandoltC Detection instance");

}

/**
@brief Default destructor
@return void
**/

LandoltCDetection::~LandoltCDetection(void)
{
  ROS_INFO("[landoltc_node] : Destroying LandoltC Detection instance");
}

/**
@brief Get parameters referring to view and frame characteristics
@return void
**/
void LandoltCDetection::getGeneralParams()
{
  packagePath = ros::package::getPath("pandora_vision_landoltc");
  
  //! Declare publisher and advertise topic
  //! where algorithm results are posted if it works alone
  if (_nh.getParam("published_topic_names/landoltc_alert", param))
  {
    _landoltcPublisher = 
      _nh.advertise<vision_communications::LandoltcAlertsVectorMsg>(param, 10);
  }
  else
  {
    ROS_FATAL("[landoltc_node]: Landoltc alert topic name not found");
    ROS_BREAK();
  }
  
  //!< Get the path to the pattern used for detection
  if (_nh.hasParam("patternPath"))
  {
    _nh.getParam("patternPath", patternPath);
    ROS_DEBUG_STREAM("patternPath: " << patternPath);
  }
  else
  {
    ROS_DEBUG("[landoltc_node] : Parameter patternPath not found. Using Default");
    std:: string temp = "/bold.jpg";
    patternPath.assign(packagePath);
    patternPath.append(temp);
  }

  //!< Get the camera to be used by landoltc node;
  if (_nh.getParam("camera_name", cameraName))
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  else
  {
    ROS_FATAL("[landoltc_node] : Camera name not found");
    ROS_BREAK();
  }

  //!< Get the Height parameter if available;
  if (_nh.getParam("image_height", frameHeight))
    ROS_DEBUG_STREAM("height : " << frameHeight);
  else
  {
    ROS_FATAL("[landoltc_node] : Parameter frameHeight not found. Using Default");
    ROS_BREAK();
  }

  //!< Get the Width parameter if available;
  if ( _nh.getParam("image_width", frameWidth))
    ROS_DEBUG_STREAM("width : " << frameWidth);
  else
  {
    ROS_FATAL("[landoltc_node] : Parameter frameWidth not found. Using Default");
    ROS_BREAK();
  }

  //!< Get the HFOV parameter if available;
  if (_nh.getParam("hfov", hfov)) 
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  else 
  {
    ROS_FATAL("[landoltc_node] : Horizontal field of view not found. Using Default");
    ROS_BREAK();
  }
  
  //!< Get the VFOV parameter if available;
  if (_nh.getParam("vfov", vfov)) 
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  else 
  {
    ROS_FATAL("[landoltc_node] : Vertical field of view not found. Using Default");
    ROS_BREAK();;
  }
    
  //!< Get the listener's topic;
  if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  else
  {
    ROS_FATAL("[landoltc_node] : Parameter imageTopic not found");
    ROS_BREAK();
  }

}

/**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool LandoltCDetection::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";
    bool res = _nh.hasParam(model_param_name);

    std::string robot_description = "";

    if(!res || !_nh.getParam(model_param_name, robot_description))
    {
      ROS_ERROR("[Motion_node]:Robot description couldn't be retrieved from the parameter server.");
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
@brief Callback for the RGB Image
@param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
@return void
**/

void LandoltCDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  
  if( ros::Time::now() - _lastTimeProcessed < ros::Duration (LandoltcParameters::timerThreshold))
      return;
      
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  landoltCFrame = in_msg -> image.clone();
  landoltcFrameTimestamp = msg->header.stamp;  
  _frame_id = msg->header.frame_id;
    
  if(_frame_id.c_str()[0] == '/')
    _frame_id = _frame_id.substr(1);
      
  if ( landoltCFrame.empty() )
  {
    ROS_ERROR("[landoltc_node] : No more Frames");
    return;
  }
  
  std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
    
  if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
    bool _indicator = getParentFrameId();
    
    _frame_ids_map.insert( it , std::pair<std::string, std::string>(
       _frame_id, _parent_frame_id));
  } 
  
  _lastTimeProcessed = ros::Time::now(); 
   
  landoltcCallback();

}

/**
 @brief main function called for publishing messages in
  data fusion. In this function an object of class landoltcDetector
  is created to identify landoltCs in current frame.
 @param void
 @return void
 **/

void LandoltCDetection::landoltcCallback()
{
  if(!landoltcNowON)
  {
    return;
  }
    
  _landoltcDetector.begin(&landoltCFrame);
  
  std::vector<LandoltC> _landoltc = _landoltcDetector.getDetectedLandolt();
  
  //!< Create message of Landoltc Detector
  vision_communications::LandoltcAlertsVectorMsg landoltcVectorMsg;
  vision_communications::LandoltcAlertMsg landoltccodeMsg;

  landoltcVectorMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
  landoltcVectorMsg.header.stamp = landoltcFrameTimestamp;
  
  for(int i = 0; i < _landoltc.size(); i++){
     
    // Landoltc center's coordinates relative to the center of the frame
    float x = _landoltc.at(i).center.x
      - static_cast<float>(frameWidth) / 2;
    float y = static_cast<float>(frameHeight) / 2
      - _landoltc.at(i).center.y;

    // Landoltc center's yaw and pitch
    landoltccodeMsg.yaw = atan(2 * x / frameWidth * tan(hfov / 2));
    landoltccodeMsg.pitch = atan(2 * y / frameHeight * tan(vfov / 2));

    landoltccodeMsg.posterior = _landoltc.at(i).probability;
    
    if(_landoltc.at(i).angles.size() == 0) continue;
    
    for(int j = 0; j < _landoltc.at(i).angles.size(); j++)
      landoltccodeMsg.angles.push_back( _landoltc.at(i).angles.at(j));
    
    landoltcVectorMsg.landoltcAlerts.push_back(landoltccodeMsg);
    
    ROS_INFO_STREAM("[landoltc_node] : Landoltc found");
    landoltccodeMsg.angles.clear();
  }
  if(_landoltc.size() > 0 && landoltcVectorMsg.landoltcAlerts.size() > 0){
    _landoltcPublisher.publish(landoltcVectorMsg);
    landoltcVectorMsg.landoltcAlerts.clear();
  }  
  _landoltcDetector.clear();
}

/**
  @brief Node's state manager
  @param newState [int] The robot's new state
  @return void
*/
void LandoltCDetection::startTransition(int newState)
{
  curState = newState;

  //!< check if datamatrix algorithm should be running now
  landoltcNowON =
    (curState ==
     state_manager_communications::robotModeMsg::MODE_EXPLORATION_RESCUE)
    || (curState ==
        state_manager_communications::robotModeMsg::MODE_IDENTIFICATION)
    || (curState ==
        state_manager_communications::robotModeMsg::MODE_SENSOR_HOLD)
    || (curState ==
        state_manager_communications::robotModeMsg::MODE_SENSOR_TEST);

  //!< shutdown if the robot is switched off
  if (curState ==
      state_manager_communications::robotModeMsg::MODE_TERMINATING)
  {
    ros::shutdown();
    return;
  }

  prevState = curState;
  
  //!< this needs to be called everytime a node finishes transition
  transitionComplete(curState);

}

/**
 @brief After completion of state transition
 @return void
 */
void LandoltCDetection::completeTransition()
{
  ROS_INFO("[Landoltc_node] : Transition Complete");
}

/**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_landoltc::landoltc_cfgConfig&]
  @param[in] level [const uint32_t] The level 
  @return void
**/
void LandoltCDetection::parametersCallback(
  const pandora_vision_landoltc::landoltc_cfgConfig& config,
  const uint32_t& level)
  {
    //!< Threshold parameters
    LandoltcParameters::gradientThreshold = config.gradientThreshold;
    LandoltcParameters::centerThreshold = config.centerThreshold;
    LandoltcParameters::huMomentsPrec = config.huMomentsPrec;
    LandoltcParameters::adaptiveThresholdSubtractSize = config.adaptiveThresholdSubtractSize;
    LandoltcParameters::visualization = config.visualization;    
    LandoltcParameters::timerThreshold = config.timerThreshold;    
  }
} // namespace pandora_vision
