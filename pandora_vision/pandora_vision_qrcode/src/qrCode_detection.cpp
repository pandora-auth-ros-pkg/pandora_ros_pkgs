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
 * Author: Miltiadis-Alexios Papadopoulos
 *********************************************************************/

#include "pandora_vision_qrcode/qrCode_detection.h"

namespace pandora_vision
{
  /**
    @brief Constructor
   **/
  QrCodeDetection::QrCodeDetection(const std::string& ns) : _nh(ns), qrcodeNowON(false)
  {
    //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";
    _camera_indicator = -1;
    //!< Get Motion Detector Parameters
    getQrCodeParams();

    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();

    //!< Convert field of view from degrees to rads
    for(int ii= 0; ii < _hfov.size(); ii++){
      _hfov.at(ii) = _hfov.at(ii) * CV_PI / 180;
      _vfov.at(ii) = _vfov.at(ii) * CV_PI / 180;
    }
    for(int ii = 0; ii < _imageTopics.size(); ii++ ){
      //!< subscribe to input image's topic
      frameSubscriber = _nh.subscribe(
        _imageTopics.at(ii), 1, &QrCodeDetection::imageCallback, this);
      _frameSubscribers.push_back(frameSubscriber);
    }
    //!< initialize states - robot starts in STATE_OFF
    curState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    prevState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    
    _lastTimeProcessed = ros::Time::now(); 
    
    clientInitialize();

    ROS_INFO("[QrCode_node] : Created QrCode Detection instance");
  }


  /**
    @brief Destructor
   */
  QrCodeDetection::~QrCodeDetection()
  {
    ROS_INFO("[QrCode_node] : Destroying QrCode Detection instance");
  }


  /**
   * @brief Get parameters referring to view and frame characteristics from
   * launch file
   * @return void
   */
  void QrCodeDetection::getGeneralParams()
  {
    //! Publishers
    
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    if (_nh.getParam("published_topic_names/qr_alert", param))
    {
      _qrcodePublisher = 
        _nh.advertise<pandora_vision_msgs::QRAlertsVectorMsg>(param, 10, true);
    }
    else
    {
      ROS_FATAL("[QrCode_node]: Qr alert topic name param not found");
      ROS_BREAK();
    }
  
    //!< Get the debugQrCode parameter if available;
    if (_nh.getParam("debugQrCode", debugQrCode))
      ROS_DEBUG_STREAM("debugQrCode : " << debugQrCode);
    else
    {
      debugQrCode = false;
      ROS_DEBUG_STREAM("debugQrCode : " << debugQrCode);
    }
    
    if(debugQrCode)
    {    
      //! Advertise topics for debugging if we are in debug mode
      if (_nh.getParam("published_topic_names/debug_qrcode", param))
      {
        _qrcodeDebugPublisher =
          image_transport::ImageTransport(_nh).advertise(param, 1);
      }
      else
        ROS_WARN("Cannot find qrcode debug show topic");
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
      ROS_INFO_STREAM("[QrCode_node]: camera_name : " << cameraName);
      
      //!< Get the listener's topic for camera
      if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
        ROS_INFO_STREAM("[QrCode_node]: imageTopic for camera : " << imageTopic);
      else
      {
       ROS_FATAL("[QrCode_node]: Image topic name not found");
       ROS_BREAK(); 
      }
      _imageTopics.push_back("/"+imageTopic);
      
      key = "image_height";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      frameHeight = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[QrCode_node]: image_height : " << frameHeight);
        
      _frameHeight.push_back(frameHeight);
        
      key = "image_width";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      frameWidth = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[QrCode_node]: image_width : " << frameWidth);
        
      _frameWidth.push_back(frameWidth);  
      
      key = "hfov";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeInt);
      hfov = static_cast<int>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[QrCode_node]: hfov : " << hfov);
        
      _hfov.push_back(hfov);  
      
      key = "vfov";
      ROS_ASSERT(cameras_list[ii][key].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      vfov = static_cast<double>(cameras_list[ii][key]);
      ROS_INFO_STREAM("[QrCode_node]: vfov : " << vfov);
        
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
   * @brief Get parameters referring to Qrcode detection algorithm
   * @return void
   */
  void QrCodeDetection::getQrCodeParams()
  {
    //!< Get the buffer size parameter if available;
    if (_nh.hasParam("qrcodeSharpenBlur"))
    {
      _nh.getParam("qrcodeSharpenBlur", _qrcodeDetector.gaussiansharpenblur);
      ROS_DEBUG_STREAM("qrcodeSharpenBlur : "
          << _qrcodeDetector.gaussiansharpenblur);
    }
    else
    {
      _qrcodeDetector.gaussiansharpenblur = 5;
    }

    //!< Get the difference threshold parameter if available;
    if (_nh.hasParam("qrcodeSharpenWeight"))
    {
      _nh.getParam("qrcodeSharpenWeight",
          _qrcodeDetector.gaussiansharpenweight);
      ROS_DEBUG_STREAM("qrcodeSharpenWeight : "
          << _qrcodeDetector.gaussiansharpenweight);
    }
    else
    {
      _qrcodeDetector.gaussiansharpenweight = 0.8;
    }
  }
  
  /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool QrCodeDetection::getParentFrameId()
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
   * @brief Function called when new ROS message appears, for front camera
   * @param msg [const sensor_msgs::Imager&] The message
   * @return void
   */
  void QrCodeDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    if( ros::Time::now() - _lastTimeProcessed < ros::Duration (_timerThreshold))
      return;
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    qrcodeFrame = in_msg->image.clone();
    qrcodeFrameTimestamp = msg.header.stamp;
    _frame_id = msg.header.frame_id;
    
    if(_frame_id.c_str()[0] == '/'){
      _frame_id = _frame_id.substr(1);
      _camera_indicator = 1;
    }
    if (qrcodeFrame.empty())
    {
      ROS_ERROR("[QrCode_node] : No more Frames");
      ros::shutdown();
      return;
    }
    
    if(!qrcodeNowON)
      return;
    
    std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
      
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
      bool _indicator = getParentFrameId();
      
      _frame_ids_map.insert( it , std::pair<std::string, std::string>(
         _frame_id, _parent_frame_id));
      
       for (it = _frame_ids_map.begin(); it != _frame_ids_map.end(); ++it)
          ROS_DEBUG_STREAM("" << it->first << " => " << it->second );
    } 
  
    _lastTimeProcessed = ros::Time::now(); 
    qrDetect();
  }


  /**
   * @brief This method uses a QrCodeDetector instance to detect all present
   * qrcodes in a given frame
   * @return void
   */
  void QrCodeDetection::qrDetect()
  {
    //!< Create message of QrCode Detector
    pandora_vision_msgs::QRAlertsVectorMsg qrcodeVectorMsg;
    pandora_vision_msgs::QRAlertMsg qrcodeMsg;
 
    //!< Qrcode message 
    //!< do detection and examine result cases
    qrcodeVectorMsg.header.frame_id =  _frame_ids_map.find(_frame_id)->second;
    qrcodeVectorMsg.header.stamp = qrcodeFrameTimestamp;

    _qrcodeDetector.detect_qrcode(qrcodeFrame);
    std::vector<QrCode> list_qrcodes = _qrcodeDetector.get_detected_qr();
     
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
      
    for(int i = 0; i < static_cast<int>(list_qrcodes.size()); i++)
    {
      qrcodeMsg.QRcontent = list_qrcodes[i].qrcode_desc;

      // Qr's center coordinates relative to the center of the frame
      float x = list_qrcodes[i].qrcode_center.x
        - static_cast<float>(frameWidth) / 2;
      float y = static_cast<float>(frameHeight) / 2
        - list_qrcodes[i].qrcode_center.y;

      // Qr center's yaw and pitch
      qrcodeMsg.yaw = atan(2 * x / frameWidth * tan(hfov / 2));
      qrcodeMsg.pitch = atan(2 * y / frameHeight * tan(vfov / 2));

      qrcodeVectorMsg.qrAlerts.push_back(qrcodeMsg);

      ROS_INFO("[QrCode_node]: QR found.");
    }

    if (debugQrCode)
    {
      publish_debug_images();
    }

    if(qrcodeVectorMsg.qrAlerts.size() > 0)
    {
      _qrcodePublisher.publish(qrcodeVectorMsg);
    }
  }



  /**
   * @brief Publishing debug images
   * @return void
   */
  void QrCodeDetection::publish_debug_images()
  {
    cv_bridge::CvImage qrcodeDebug;
    qrcodeDebug.encoding = sensor_msgs::image_encodings::MONO8;
    qrcodeDebug.image = _qrcodeDetector.get_debug_frame().clone();
    _qrcodeDebugPublisher.publish(qrcodeDebug.toImageMsg());
  }



  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
   */
  void QrCodeDetection::startTransition(int newState){

    curState = newState;

    //!< check if QR algorithm should be running now
    qrcodeNowON = (curState !=
          state_manager_msgs::RobotModeMsg::MODE_OFF);

    //!< shutdown if the robot is switched off
    if (curState ==
        state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState = curState;

    //!< this needs to be called everyTime a node finishes transition
    transitionComplete(curState);
  }



  /**
   * @brief After completion of state transition
   * @return void
   */
  void QrCodeDetection::completeTransition()
  {
    ROS_INFO("[QrCodeNode] : Transition Complete");
  }
}// namespace pandora_vision

