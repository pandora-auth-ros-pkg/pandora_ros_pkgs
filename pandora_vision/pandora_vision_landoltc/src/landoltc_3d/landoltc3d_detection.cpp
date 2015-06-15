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

#include "pandora_vision_landoltc/landoltc_3d/landoltc3d_detection.h"


namespace pandora_vision
{

/**
@brief Default Constructor
@param void
@return void
**/

LandoltC3dDetection::LandoltC3dDetection(const std::string& ns): _nh(ns), landoltc3dNowON(false)
{
  getGeneralParams();

  //!< Convert field of view from degrees to rads
  hfov = hfov * CV_PI / 180;
  vfov = vfov * CV_PI / 180;

  //!< Initiliaze and preprocess reference image
  _landoltc3dDetector.initializeReferenceImage(patternPath);

  if(PredatorOn)
  {
    _landoltc3dPredator = _nh.subscribe(predator_topic_name, 1,
    &LandoltC3dDetection::predatorCallback, this);

  }
  else
  {
    _inputImageSubscriber = _nh.subscribe(imageTopic, 1,
    &LandoltC3dDetection::imageCallback, this);
  }


  //!<Setting Predator value ON or OFF for the 3DLandoltC Detector
  //!<This is used for the fusion function later, in order to assign
  //!<probabilities according to the predator state

  _landoltc3dDetector.setPredatorOn(PredatorOn);

  //!< The dynamic reconfigure parameter's callback
  server.setCallback(boost::bind(&LandoltC3dDetection::parametersCallback, this, _1, _2));

  //!< initialize states - robot starts in STATE_OFF
  curState = state_manager_msgs::RobotModeMsg::MODE_OFF;
  prevState = state_manager_msgs::RobotModeMsg::MODE_OFF;

  clientInitialize();

  _lastTimeProcessed = ros::Time::now();

  ROS_INFO("[landoltc3d_node] : Created LandoltC3d Detection instance");

}

/**
@brief Default destructor
@return void
**/

LandoltC3dDetection::~LandoltC3dDetection(void)
{
  ROS_INFO("[landoltc3d_node] : Destroying LandoltC3d Detection instance");
}

/**
@brief Get parameters referring to view and frame characteristics
@return void
**/
void LandoltC3dDetection::getGeneralParams()
{
  packagePath = ros::package::getPath("pandora_vision_landoltc");

  //! Declare publisher and advertise topic
  //! where algorithm results are posted if it works alone
  if (_nh.getParam("published_topic_names/landoltc_alert", param))
  {
    _landoltc3dPublisher =
      _nh.advertise<pandora_vision_msgs::LandoltcAlertVector>(param, 10);
  }
  else
  {
    ROS_FATAL("[landoltc3d_node] : Landoltc alert topic name not found");
    ROS_BREAK();
  }

  //! Declare subscriber
  //! where algorithm results are posted if it works with predator
  if (_nh.getParam("subscribed_topic_names/predator_topic_name", predator_topic_name))
    ROS_DEBUG("[landoltc3d_node] : Loaded topic name to use with predator");
  else
  {
    ROS_FATAL("[landoltc3d_node] : Landoltc image topic name not found");
    ROS_BREAK();
  }

  //!< Get the path to the pattern used for detection
  if (_nh.getParam("patternPath", patternPath))
    ROS_DEBUG_STREAM("patternPath: " << patternPath);
  else
  {
    ROS_DEBUG("[landoltc3d_node] : Parameter patternPath not found. Using Default");
    std:: string temp = "/bold.jpg";
    patternPath.assign(packagePath);
    patternPath.append(temp);
  }

  //!< Get the PredatorOn value
  if(_nh.getParam("operation_state", PredatorOn))
    ROS_DEBUG("[landoltc3d_node] : Loading predator_on parameter");
  else
  {
    ROS_DEBUG("[landoltc3d_node] : Parameter PredatorOn not found. Using Default");
    PredatorOn = false;
  }

  //!< Get the camera to be used by landoltc3d node;
  if (_nh.getParam("camera_name", cameraName))
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Camera name not found");
    ROS_BREAK();
  }

  //!< Get the Height parameter if available;
  if (_nh.getParam("image_height", frameHeight))
    ROS_DEBUG_STREAM("height : " << frameHeight);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Parameter frameHeight not found");
    ROS_BREAK();
  }

  //!< Get the Width parameter if available;
  if (_nh.getParam("image_width", frameWidth))
    ROS_DEBUG_STREAM("width : " << frameWidth);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Parameter frameWidth not found.");
    ROS_BREAK();
  }

  //!< Get the HFOV parameter if available;
  if (_nh.getParam("hfov", hfov))
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Horiznontal field of view not found.");
    ROS_BREAK();
  }

  //!< Get the VFOV parameter if available;
  if (_nh.getParam("vfov", vfov))
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Vertical field of view not found.");
    ROS_BREAK();
  }

  //!< Get the listener's topic;
  if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  else
  {
    ROS_FATAL("[landoltc3d_node] : Image topic name not found.");
    ROS_BREAK();
  }

}

/**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
**/
bool LandoltC3dDetection::getParentFrameId()
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

void LandoltC3dDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if( ros::Time::now() - _lastTimeProcessed < ros::Duration (Landoltc3DParameters::timerThreshold))
      return;

  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  landoltCFrame = in_msg -> image.clone();
  landoltc3dFrameTimestamp = msg->header.stamp;
  _frame_id = msg->header.frame_id;

  if(_frame_id.c_str()[0] == '/')
    _frame_id = _frame_id.substr(1);

  if ( landoltCFrame.empty() )
  {
    ROS_ERROR("[landoltc3d_node] : No more Frames");
    return;
  }

  std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();

  if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
    bool _indicator = getParentFrameId();

    _frame_ids_map.insert( it , std::pair<std::string, std::string>(
       _frame_id, _parent_frame_id));
  }

  //ROS_INFO("Getting Frame From Camera");
  _lastTimeProcessed = ros::Time::now();

  landoltc3dCallback();

}

void LandoltC3dDetection::predatorCallback(
    const pandora_vision_msgs::Predator& msg)
{
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg.image, sensor_msgs::image_encodings::BGR8);
  landoltCFrame = in_msg->image.clone();
  landoltc3dFrameTimestamp = msg.header.stamp;
  _frame_id  = msg.header.frame_id;

  std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();

  if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
    bool _indicator = getParentFrameId();

    _frame_ids_map.insert( it , std::pair<std::string, std::string>(
       _frame_id, _parent_frame_id));
  }

  cv::Rect bounding_box = cv::Rect(msg.regionOfInterest.center.x, 
      msg.regionOfInterest.center.y, msg.regionOfInterest.width,
      msg.regionOfInterest.height);
  float posterior = msg.posterior;
  //ROS_INFO("Getting Frame From Predator");
  _landoltc3dDetector.setPredatorValues(bounding_box, posterior);

  landoltc3dCallback();
}


/**
 @brief main function called for publishing messages in
  data fusion. In this function an object of class landoltc3dDetector
  is created to identify landoltCs in current frame.
 @param void
 @return void
 **/

void LandoltC3dDetection::landoltc3dCallback()
{
  if(!landoltc3dNowON)
  {
    return;
  }

  _landoltc3dDetector.begin(&landoltCFrame);

  std::vector<LandoltC3D> _landoltc3d = _landoltc3dDetector.getDetectedLandolt();

  //!< Create message of Landoltc Detector
  pandora_vision_msgs::LandoltcAlertVector landoltc3dVectorMsg;
  pandora_vision_msgs::LandoltcAlert landoltc3dcodeMsg;

  landoltc3dVectorMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
  landoltc3dVectorMsg.header.stamp = landoltc3dFrameTimestamp;

  for(int i = 0; i < _landoltc3d.size(); i++){

    // Landoltc center's coordinates relative to the center of the frame
    float x = _landoltc3d.at(i).center.x
      - static_cast<float>(frameWidth) / 2;
    float y = static_cast<float>(frameHeight) / 2
      - _landoltc3d.at(i).center.y;

    // Landoltc center's yaw and pitch
    landoltc3dcodeMsg.info.yaw = atan(2 * x / frameWidth * tan(hfov / 2));
    landoltc3dcodeMsg.info.pitch = atan(2 * y / frameHeight * tan(vfov / 2));
    landoltc3dcodeMsg.info.probability = _landoltc3d.at(i).probability;

    landoltc3dcodeMsg.posterior = _landoltc3d.at(i).probability;

    if(_landoltc3d.at(i).angles.size() == 0) continue;

    for(int j = 0; j < _landoltc3d.at(i).angles.size(); j++)
    {
      landoltc3dcodeMsg.angles.push_back( _landoltc3d.at(i).angles.at(j));
    }

    landoltc3dVectorMsg.alerts.push_back(landoltc3dcodeMsg);


    ROS_INFO_STREAM("[landoltc3d_node] : Landoltc3D found");
    landoltc3dcodeMsg.angles.clear();

  }
  if(_landoltc3d.size() > 0 && landoltc3dVectorMsg.alerts.size() > 0){
    _landoltc3dPublisher.publish(landoltc3dVectorMsg);
    landoltc3dVectorMsg.alerts.clear();
  }
  _landoltc3dDetector.clear();
}

/**
  @brief Node's state manager
  @param newState [int] The robot's new state
  @return void
*/
void LandoltC3dDetection::startTransition(int newState)
{
  curState = newState;

  //!< check if datamatrix algorithm should be running now
  landoltc3dNowON =
    (curState ==
     state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE)
    || (curState ==
        state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION)
    || (curState ==
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD)
    || (curState ==
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST);

  //!< shutdown if the robot is switched off
  if (curState ==
      state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
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
void LandoltC3dDetection::completeTransition()
{
  ROS_INFO("[Landoltc3d_node] : Transition Complete");
}

/**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_landoltc::landoltc3d_cfgConfig&]
  @param[in] level [const uint32_t] The level
  @return void
**/
void LandoltC3dDetection::parametersCallback(const pandora_vision_landoltc::landoltc3d_cfgConfig& config,
const uint32_t& level)
{
  //!< Threshold parameters
  Landoltc3DParameters::gradientThreshold = config.gradientThreshold;
  Landoltc3DParameters::centerThreshold = config.centerThreshold;
  Landoltc3DParameters::huMomentsPrec = config.huMomentsPrec;
  Landoltc3DParameters::adaptiveThresholdSubtractSize = config.adaptiveThresholdSubtractSize;
  Landoltc3DParameters::bradleyPerc = config.bradleyPerc;
  Landoltc3DParameters::visualization = config.visualization;
  Landoltc3DParameters::timerThreshold = config.timerThreshold;
}

} // namespace pandora_vision
