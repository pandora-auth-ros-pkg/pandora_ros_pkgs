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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/victim_detection.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  VictimDetection::VictimDetection(const std::string& ns) : _nh(ns), victimNowON(false)
  {
     //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";
    
    /// Get general parameters for image processing
    getGeneralParams();
    
    /// Get parameters referring to faceDetector instance
    getVictimDetectorParameters();

    /// Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;
    
    //!< Subscribe to input image's topic
    //!< image_transport::ImageTransport it(_nh);
    //~ _frameSubscriber = _nh.subscribe(
                       //~ "/kinect/rgb/image_color", 1, &VictimDetection::dummyimageCallback, this);
                       
    /// Subscribe to input image's topic
    /// image_transport::ImageTransport it(_nh);
    _frameSubscriber = _nh.subscribe(
              _enhancedHolesTopic, 1, &VictimDetection::imageCallback, this);
    
     /// Initialize victim detector
    _victimDetector = new VictimDetector(cascade_path, model_path, bufferSize,
      rgb_classifier_path, depth_classifier_path);
    
    /// Initialize states - robot starts in STATE_OFF
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();
    _rgbImage = cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);
    
    ROS_INFO("[victim_node] : Created Victim Detection instance");
  }

  /**
    @brief Destructor
  */
  VictimDetection::~VictimDetection()
  {
    ROS_DEBUG("[victim_node] : Destroying Victim Detection instance");
   
  }

  /**
   @brief Get parameters referring to the view and
   *frame characteristics
   @return void
  **/
  void VictimDetection::getGeneralParams()
  {
    packagePath = ros::package::getPath("pandora_vision_victim");
    
    //! Publishers
      
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    if (_nh.getParam("published_topic_names/victim_alert", param)){
      _victimDirectionPublisher = 
        _nh.advertise<pandora_common_msgs::GeneralAlertMsg>(param, 10, true);
    }
    else{
      ROS_FATAL("[victim_node] : Victim alert topic name param not found");
      ROS_BREAK();
    }
     
    //! Subscribers
      
    //! Declare subsciber's topic name
    if (_nh.getParam("subscribed_topic_names/enhanded_hole_alert", param))
    {
      ROS_INFO_STREAM("PARAM"<< param);
      _enhancedHolesTopic = param;
    }  
    else{
      ROS_FATAL("[victim_node] : Victim subscribed topic name param not found");
      ROS_BREAK();
    }  
    
        
    //!< Get the camera to be used by motion node;
    if (_nh.getParam("camera_name", cameraName)) 
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    else 
    {
      ROS_FATAL("[Motion_node]: Camera name not found");
      ROS_BREAK(); 
    }

    //! Get the Height parameter if available;
    if (_nh.getParam("image_height", frameHeight)) 
      ROS_DEBUG_STREAM("height : " << frameHeight);
    else 
    {
      ROS_FATAL("[motion_node] : Parameter frameHeight not found. Using Default");
      ROS_BREAK();
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("image_width", frameWidth)) 
      ROS_DEBUG_STREAM("width : " << frameWidth);
    else 
    {
      ROS_FATAL("[motion_node] : Parameter frameWidth not found. Using Default");
      ROS_BREAK();
    }
  
    //!< Get the HFOV parameter if available;
    if (_nh.getParam("hfov", hfov)) 
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    else 
    {
     ROS_FATAL("[motion_node]: Horizontal field of view not found");
     ROS_BREAK();
    }
    
    //!< Get the VFOV parameter if available;
    if (_nh.getParam("vfov", vfov)) 
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    else 
    {
     ROS_FATAL("[motion_node]: Vertical field of view not found");
     ROS_BREAK();
    }  
    
  }

  /**
    @brief Get parameters referring to the face detection algorithm
    @return void
  **/
  void VictimDetection::getVictimDetectorParameters()
  {
    //!< Get the path of haar_cascade xml file if available;
    if ( _nh.getParam("cascade_path", cascade_path)){
      cascade_path = packagePath + cascade_path;
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << cascade_path);
    }
    else{
      model_path = packagePath + "/data/haarcascade_frontalface_alt_tree.xml";
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << cascade_path);
    }

    //!< Get the model.xml url;
    if (_nh.getParam("model_url", model_url))
      ROS_INFO_STREAM("[victim_node]: modelURL : " << model_url);
    else{
      model_url = "https://pandora.ee.auth.gr/vision/model.xml";
      ROS_INFO_STREAM("[victim_node]: modelURL : " << model_url);
    }

    //!< Get the path of model_path xml file to be loaded
    if (_nh.getParam("model_path",  model_path)){
      model_path = packagePath + model_path;
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  model_path);
    }
    else{
      model_path = packagePath + "/data/model.xml";
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  model_path);
    }
    
    //!< Get the path of rgb classifier
    if (_nh.getParam("rgb_classifier_path",  rgb_classifier_path)){
      rgb_classifier_path = packagePath + rgb_classifier_path;
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
            <<  rgb_classifier_path);
    }
    else{
      rgb_classifier_path = packagePath + "/data/rgb_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
            <<  rgb_classifier_path);
    }
    
    //!< Get the path of depth classifier
    if (_nh.getParam("depth_classifier_path",  depth_classifier_path)){
      depth_classifier_path = packagePath + depth_classifier_path;
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
            <<  depth_classifier_path);
    }
    else{
      depth_classifier_path = packagePath + "/data/depth_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
            <<  depth_classifier_path);
    }
    
    /// Parameter that changes respectivly if we have depth information
    if ( _nh.getParam("isDepthEnabled", isDepthEnabled))
      ROS_DEBUG_STREAM("[victim_node] : isDepthEnabled : " << isDepthEnabled);
    else
      isDepthEnabled = false;
      
    /// Parameter that changes respectivly if we have information
    ///about the position of the hole
    if ( _nh.getParam("isHole", isHole))
      ROS_DEBUG_STREAM("[victim_node] : isHole : " << isHole);
    else
      isDepthEnabled = false;
      
    if ( _nh.getParam("bufferSize", bufferSize))
      ROS_DEBUG_STREAM("[victim_node] : bufferSize : " << bufferSize);
    else
      bufferSize = 5;
  }
  
  /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool VictimDetection::getParentFrameId()
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
   * @brief Function called when new ROS message appears, for camera
   * @param msg [const sensor_msgs::Image&] The message
   * @return void
  */
  void VictimDetection::dummyimageCallback(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _rgbImage = in_msg->image.clone();
    victimFrameTimestamp = msg.header.stamp;
    
    if (_rgbImage.empty() )
    {
      ROS_ERROR("[face_node] : No more Frames ");
      return;
    }
    isDepthEnabled = false;
    isHole = true;
    checkState();
  }

  /**
   * @brief Function called when new message appears from hole_detector_node
   * @param msg [vision_communications::EnhancedHolesVectorMsg&] The message
   * @return void
   */
  void VictimDetection::imageCallback(
      const vision_communications::EnhancedHolesVectorMsg& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg.rgbImage, sensor_msgs::image_encodings::TYPE_8UC3);
    _rgbImage = in_msg->image.clone();
    
    if(_frame_id.c_str()[0] == '/')
      _frame_id = _frame_id.substr(1);
      
       
    if (_rgbImage.empty()){
      ROS_FATAL("[victim_node] : No more frames ");
      ROS_BREAK();
    }
    
    in_msg = 
      cv_bridge::toCvCopy(msg.depthImage, sensor_msgs::image_encodings::TYPE_8UC1);
    _depthImage = in_msg->image.clone();
    
    //~ isDepthEnabled = msg.isDepth;
    isDepthEnabled = false;
    
    _frame_id = msg.header.frame_id; 
    _enhancedHoles = msg;
    if (_enhancedHoles.enhancedHoles.size() > 0)
      isHole = true;
    
    
    
    //~ for(unsigned int k = 0 ; k < msg.enhancedHoles.size() ; k++)
    //~ {
      //~ for(int i = 0; i < 4; i++ ){
        //~ cv::line(_rgbImage, 
          //~ cv::Point(msg.enhancedHoles[k].verticesX[i],
            //~ msg.enhancedHoles[k].verticesY[i]),
          //~ cv::Point(msg.enhancedHoles[k].verticesX[(i+1)%4],
            //~ msg.enhancedHoles[k].verticesY[(i+1)%4]), 
          //~ CV_RGB(255, 0, 0), 1, 8);
      //~ }
    //~ }
    //~ 
    //~ cv::imshow("blabla",_rgbImage);
    //~ cv::waitKey(30);
    
    victimFrameTimestamp = in_msg->header.stamp;
    cameraFrameId= in_msg->header.frame_id;
    
    checkState();
    
    std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
      
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
      bool _indicator = getParentFrameId();
      
      _frame_ids_map.insert( it , std::pair<std::string, std::string>(
         _frame_id, _parent_frame_id));
    } 
    
  }
  
  /**
   * @brief This method check in which state we are, according to
   * the information sent from hole_detector_node
   * @return void
  */
  void VictimDetection::checkState()
  {
    //~ _rgbdImages.clear();
    DetectionImages imgs; 
    _stateIndicator = 2 * isDepthEnabled + isHole + 1;
    
    imgs.rgb = _rgbImage;
    switch(_stateIndicator)
    {
      case 1:
        _victimDetector->detectionMode = GOT_NOTHING;
        break;
      case 2:
        _victimDetector->detectionMode = GOT_MASK;
        break;
      case 3:
        _victimDetector->detectionMode = GOT_DEPTH;
        imgs.depth = _depthImage;
        break;
      case 4:
        _victimDetector->detectionMode = GOT_ALL;
        imgs.depth = _depthImage;
        break;
    }
    for(unsigned int i = 0 ; i < _enhancedHoles.enhancedHoles.size();
      i++)
    {
      
      int minx = 10000, maxx = -1, miny = 10000, maxy = -1;
      for(unsigned int j = 0 ; j < 4 ; j++)
      {
        int xx = _enhancedHoles.enhancedHoles[i].verticesX[j];
        int yy = _enhancedHoles.enhancedHoles[i].verticesY[j];
        minx = xx < minx ? xx : minx;
        maxx = xx > maxx ? xx : maxx;
        miny = yy < miny ? yy : miny;
        maxy = yy > maxy ? yy : maxy;
      }
      cv::Rect rect(minx, miny, maxx - minx, maxy - miny);
      cv::Mat temp = _rgbImage(rect);
      cv::resize(temp, temp, cv::Size(640, 480));
      imgs.rgbMasks.push_back(temp);
      if(isDepthEnabled)
      {
        temp = _depthImage(rect);
        imgs.depthMasks.push_back(temp);
      }
    }
    victimDetect(imgs);    
  }
  
  /**
   * @brief This method uses a FaceDetector instance to detect all
   * present faces in a given frame
   * @return void
  */
  void VictimDetection::victimDetect(DetectionImages imgs)
  {
    if(!victimNowON)
      return;
    int facesNum = 0;
    facesNum = _victimDetector->victimFusion(imgs);
    
    //!< Create message of Victim Detector
    pandora_common_msgs::GeneralAlertMsg victimMessage;
    
    if(facesNum > 0){
    int* facesTable = _victimDetector->_faceDetector->getFacePositionTable();
    
    for(int i = 0;  i < facesNum; i++){
      // Victim's center coordinates relative to the center of the frame
      float x = facesTable[i * 4]
          - static_cast<float>(frameWidth) / 2;
      float y = static_cast<float>(frameHeight) / 2
          - facesTable[i * 4 + 1];
                                      
      victimMessage.header.frame_id = _frame_ids_map.find(_frame_id)->second;
      victimMessage.header.stamp = ros::Time::now();
      victimMessage.yaw = atan(2 * x / frameWidth * tan(hfov / 2));;
      victimMessage.pitch = atan(2 * y / frameHeight * tan(vfov / 2));;
      victimMessage.probability = _victimDetector->_faceDetector->getProbability();
      ROS_INFO_STREAM( "[victim_node] :Victim ");
      _victimDirectionPublisher.publish(victimMessage);
     }
     delete facesTable; 
    } 
  }


  /**
    * @brief Node's state manager
    * @param newState [int] The robot's new state
    * @return void
   */
  void VictimDetection::startTransition(int newState)
  {

    curState = newState;

    //!< check if face detection algorithm should be running now
    victimNowON = 
    (curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
    (curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH) ||
    (curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD);

    //!< shutdown if the robot is switched off
    if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState = curState;

    //!< this needs to be called everytime a node finishes transition
    transitionComplete(curState);
  }

  /**
   * @brief After completion of state transition
   * @return void
   */
  void VictimDetection::completeTransition(void)
  {
    ROS_INFO("[victim_node] : Transition Complete");
  }
}// namespace pandora_vision
