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
  VictimDetection::VictimDetection(const std::string& ns) : 
    _nh(ns), 
    params(ns),
    imageTransport_(_nh)
  {
     //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";

    /// Convert field of view from degrees to rads
    VictimParameters::hfov = VictimParameters::hfov * CV_PI / 180;
    VictimParameters::vfov = VictimParameters::vfov * CV_PI / 180;
                       
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    _victimDirectionPublisher = 
      _nh.advertise<pandora_common_msgs::GeneralAlertMsg>(
        VictimParameters::victimAlertTopic, 10, true);
                       
    /// Subscribe to input image's topic
    /// image_transport::ImageTransport it(_nh);
    _frameSubscriber = _nh.subscribe(
      VictimParameters::enhancedHolesTopic, 
        1, &VictimDetection::imageCallback, this);
      
    /// Initialize the face detector and the svm classifiers
    _rgbViolaJonesDetector = VictimVJDetector(
      VictimParameters::cascade_path, 
      VictimParameters::model_path);
      
    _rgbSystemValidator.initialize(
      VictimParameters::rgb_classifier_path);
      
    _depthSystemValidator.initialize(
      VictimParameters::depth_classifier_path);
    
    /// Initialize states - robot starts in STATE_OFF
    curState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    prevState = state_manager_msgs::RobotModeMsg::MODE_OFF;
    
    _debugVictimsPublisher = imageTransport_.advertise
      (VictimParameters::victimDebugImg, 1, true);
    _interpolatedDepthPublisher = imageTransport_.advertise
      (VictimParameters::interpolatedDepthImg, 1, true);

    clientInitialize();
    
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
      ROS_ERROR("[Motion_node]:Robot description couldn't be \
        retrieved from the parameter server.");
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
   * @brief Function called when new message appears from hole_detector_node
   * @param msg [pandora_vision_msgs::EnhancedHolesVectorMsg&] The message
   * @return void
   */
  void VictimDetection::imageCallback(
      const pandora_vision_msgs::EnhancedHolesVectorMsg& msg)
  {
    
    if(
      (curState != 
        state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION) &&
      (curState != 
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD) && 
      (curState != 
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST)
    )
    {
      return;
    }
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg.rgbImage, 
      sensor_msgs::image_encodings::TYPE_8UC3);
    
    cv::Mat rgbImage = in_msg->image.clone();
    
    if(_frame_id.c_str()[0] == '/')
      _frame_id = _frame_id.substr(1);
      
       
    if (rgbImage.empty()){
      ROS_FATAL("[victim_node] : No more frames ");
      ROS_BREAK();
    }
    
    cv_bridge::CvImagePtr in_msg_d = cv_bridge::toCvCopy(msg.depthImage, 
      sensor_msgs::image_encodings::TYPE_8UC1);
    
    cv::Mat depthImage = in_msg_d->image.clone();

    _frame_id = msg.header.frame_id; 
    victimFrameTimestamp = msg.header.stamp;

    victimFrameTimestamp = in_msg->header.stamp;
    cameraFrameId= in_msg->header.frame_id;
    
    //! The actual victim detection
    detectVictims(
      msg.isDepth, 
      msg.enhancedHoles.size() > 0,
      rgbImage,
      depthImage,
      msg
    );
    
    //! Interpolated depth image publishing
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = msg.header;
      msgPtr->encoding = sensor_msgs::image_encodings::MONO8;
      depthImage.copyTo(msgPtr->image);
      
      // Publish the image message
      _interpolatedDepthPublisher.publish(*msgPtr->toImageMsg());
    }
    
    //! Resolve frame ids (must explain more)
    std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) 
    {
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
  void VictimDetection::detectVictims(
    bool depthEnabled, 
    bool holesEnabled,
    const cv::Mat& rgbImage,
    const cv::Mat& depthImage,
    const pandora_vision_msgs::EnhancedHolesVectorMsg& msg
  )
  {
    
    if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
    {
      rgbImage.copyTo(debugImage);
      rgb_vj_keypoints.clear();
      rgb_svm_keypoints.clear();
      depth_vj_keypoints.clear();
      depth_svm_keypoints.clear();
      rgb_vj_bounding_boxes.clear();
      rgb_svm_bounding_boxes.clear();
      depth_vj_bounding_boxes.clear();
      depth_svm_bounding_boxes.clear();
      holes_bounding_boxes.clear();
      rgb_vj_p.clear();
      rgb_svm_p.clear();
      depth_vj_p.clear();
      depth_svm_p.clear();
    }
    
    DetectionImages imgs; 
    int stateIndicator = 2 * depthEnabled + holesEnabled + 1;
    
    {
      EnhancedMat emat;
      rgbImage.copyTo(emat.img);
      imgs.rgb = emat;
      imgs.rgb.bounding_box = cv::Rect(0, 0, 0, 0);
      imgs.rgb.keypoint = cv::Point2f(0, 0);
    }
      
    DetectionMode detectionMode;
    switch(stateIndicator)
    {
      case 1:
        detectionMode = GOT_RGB;
        break;
      case 2:
        detectionMode = GOT_HOLES;
        break;
      case 3:
        detectionMode = GOT_DEPTH;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
      case 4:
        detectionMode = GOT_HOLES_AND_DEPTH;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
    }
    for(unsigned int i = 0 ; i < msg.enhancedHoles.size();
      i++)
    {
      
      int minx = 10000, maxx = -1, miny = 10000, maxy = -1;
      for(unsigned int j = 0 ; j < 4 ; j++)
      {
        int xx = msg.enhancedHoles[i].verticesX[j];
        int yy = msg.enhancedHoles[i].verticesY[j];
        minx = xx < minx ? xx : minx;
        maxx = xx > maxx ? xx : maxx;
        miny = yy < miny ? yy : miny;
        maxy = yy > maxy ? yy : maxy;
      }
      cv::Rect rect(minx, miny, maxx - minx, maxy - miny);
      holes_bounding_boxes.push_back(rect);
      
      EnhancedMat emat;
      emat.img = rgbImage(rect);
      cv::resize(emat.img, emat.img, 
        cv::Size(VictimParameters::frameWidth, VictimParameters::frameHeight));
      emat.bounding_box = rect;
      emat.keypoint = cv::Point2f(
        msg.enhancedHoles[i].keypointX,
        msg.enhancedHoles[i].keypointY
      );
      imgs.rgbMasks.push_back(emat);
      
      if(GOT_HOLES_AND_DEPTH || GOT_DEPTH)
      {
        emat.img = depthImage(rect);
        imgs.depthMasks.push_back(emat);
      }
    }

    std::vector<DetectedVictim> final_victims = 
      victimFusion(imgs, detectionMode);

    //!< Message alert creation
    for(int i = 0;  i < final_victims.size() ; i++)
    {
      if( final_victims[i].probability > 0.0001)
      {
       
        float x = final_victims[i].keypoint.x
          - static_cast<float>(VictimParameters::frameWidth) / 2;
        float y = static_cast<float>(VictimParameters::frameHeight) / 2
          - final_victims[i].keypoint.y;
            
        //!< Create message of Victim Detector
        pandora_common_msgs::GeneralAlertMsg victimMessage;
                                        
        victimMessage.header.frame_id = _frame_ids_map.find(_frame_id)->second;
        
        victimMessage.header.stamp = victimFrameTimestamp;
        
        victimMessage.yaw = 
          atan(2 * x / VictimParameters::frameWidth 
            * tan(VictimParameters::hfov / 2));
        
        victimMessage.pitch = 
          atan(2 * y / VictimParameters::frameHeight 
            * tan(VictimParameters::vfov / 2));
            
        victimMessage.probability = final_victims[i].probability;
        
        _victimDirectionPublisher.publish(victimMessage);
      }
      //!< Debug purposes
      if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
      {
        cv::KeyPoint kp(final_victims[i].keypoint, 10);
        cv::Rect re = final_victims[i].boundingBox;
        switch(final_victims[i].source)
        {
          case RGB_VJ:
            rgb_vj_keypoints.push_back(kp);
            rgb_vj_bounding_boxes.push_back(re);
            rgb_vj_p.push_back(final_victims[i].probability);
            break;
          case RGB_SVM:
            rgb_svm_keypoints.push_back(kp);
            rgb_svm_bounding_boxes.push_back(re);
            rgb_svm_p.push_back(final_victims[i].probability);
            break;
          case DEPTH_VJ:
            depth_vj_keypoints.push_back(kp);
            depth_vj_bounding_boxes.push_back(re);
            depth_vj_p.push_back(final_victims[i].probability);
            break;
          case DEPTH_RGB_SVM:
            depth_svm_keypoints.push_back(kp);
            depth_svm_bounding_boxes.push_back(re);
            depth_svm_p.push_back(final_victims[i].probability);
            break;
        }

      }
    } 
    
    //! Debug image
    if(VictimParameters::debug_img || VictimParameters::debug_img_publisher)
    {
      cv::drawKeypoints(debugImage, rgb_vj_keypoints, debugImage, 
        CV_RGB(0, 255, 0),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < rgb_vj_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, rgb_vj_bounding_boxes[i], 
          CV_RGB(0, 255, 0));
        {
          std::ostringstream convert;
          convert << rgb_vj_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            rgb_vj_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
        }
      }
        
      cv::drawKeypoints(debugImage, depth_vj_keypoints, debugImage, 
        CV_RGB(255, 100, 0),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < depth_vj_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, depth_vj_bounding_boxes[i], 
          CV_RGB(255, 100, 0));
        {
          std::ostringstream convert;
          convert << depth_vj_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            depth_vj_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
        }
      }
        
      cv::drawKeypoints(debugImage, rgb_svm_keypoints, debugImage, 
        CV_RGB(0, 100, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < rgb_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, rgb_svm_bounding_boxes[i], 
          CV_RGB(0, 100, 255));
        {
          std::ostringstream convert;
          convert << rgb_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            rgb_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
        }
      }
      
      cv::drawKeypoints(debugImage, depth_svm_keypoints, debugImage, 
        CV_RGB(0, 255, 255),
        cv::DrawMatchesFlags::DEFAULT);
      for(unsigned int i = 0 ; i < depth_svm_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, depth_svm_bounding_boxes[i], 
          CV_RGB(0, 255, 255));
        {
          std::ostringstream convert;
          convert << depth_svm_p[i];
          cv::putText(debugImage, convert.str().c_str(),
            depth_svm_keypoints[i].pt,
            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
        }
      }
      for(unsigned int i = 0 ; i < holes_bounding_boxes.size() ; i++)
      {
        cv::rectangle(debugImage, holes_bounding_boxes[i], 
          CV_RGB(0, 0, 0));
      }
      
      {
        std::ostringstream convert;
        convert << "RGB_VJ : "<< rgb_vj_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 20),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 0), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "DEPTH_VJ : "<< depth_vj_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 40),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(255, 100, 0), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "RGB_SVM : "<< rgb_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 60),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 100, 255), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "DEPTH_SVM : "<< depth_svm_keypoints.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 80),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 255, 255), 1, CV_AA);
      }
      {
        std::ostringstream convert;
        convert << "Holes got : "<< msg.enhancedHoles.size();
        cv::putText(debugImage, convert.str().c_str(),
          cvPoint(10, 100),
          cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, CV_RGB(0, 0, 0), 1, CV_AA);
      }
    }
    if(VictimParameters::debug_img_publisher)
    {
      // Convert the image into a message
      cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

      msgPtr->header = msg.header;
      msgPtr->encoding = sensor_msgs::image_encodings::BGR8;
      msgPtr->image = debugImage;
      // Publish the image message
      _debugVictimsPublisher.publish(*msgPtr->toImageMsg());
    }
    if(VictimParameters::debug_img)
    {
      cv::imshow("Victim detector", debugImage);
      cv::waitKey(30);
    }
  }
  
    /**
   *@brief Function that enables suitable subsystems, according
   * to the current State 
   * @param [std::vector<cv::Mat>] vector of images to be processed. Size of
   * vector can be either 2 or 1, if we have both rgbd information or not
   * @return void
  */ 
  std::vector<DetectedVictim> VictimDetection::victimFusion(
    DetectionImages imgs,
    DetectionMode detectionMode)
  {
    std::vector<DetectedVictim> final_probabilities;
    
    std::vector<DetectedVictim> rgb_vj_probabilities;
    std::vector<DetectedVictim> depth_vj_probabilities;
    std::vector<DetectedVictim> rgb_svm_probabilities;
    std::vector<DetectedVictim> depth_svm_probabilities;
    
    DetectedVictim temp;
    
    ///Enable Viola Jones for rgb image 
    rgb_vj_probabilities = _rgbViolaJonesDetector.findFaces(imgs.rgb.img);
    
    if(detectionMode == GOT_HOLES_AND_DEPTH || detectionMode == GOT_DEPTH)
    {
      depth_vj_probabilities = _rgbViolaJonesDetector.findFaces(imgs.depth.img);
    }
    if(detectionMode == GOT_HOLES || detectionMode == GOT_HOLES_AND_DEPTH)
    {
      for(int i = 0 ; i < imgs.rgbMasks.size(); i++)
      {
        temp.probability = _rgbSystemValidator.calculateSvmRgbProbability(
          imgs.rgbMasks.at(i).img);
        temp.keypoint = imgs.rgbMasks[i].keypoint;
        temp.source = RGB_SVM;
        temp.boundingBox = imgs.depthMasks[i].bounding_box;
        rgb_svm_probabilities.push_back(temp);
      }  
    }
    if(detectionMode == GOT_HOLES_AND_DEPTH)
    {
      for(int i = 0 ; i < imgs.depthMasks.size(); i++)
      {
        temp.probability = _depthSystemValidator.calculateSvmDepthProbability(
          imgs.depthMasks.at(i).img);
        temp.keypoint = imgs.depthMasks[i].keypoint;
        temp.source = DEPTH_RGB_SVM;
        temp.boundingBox = imgs.depthMasks[i].bounding_box;
        depth_svm_probabilities.push_back(temp);
      }
    }

    // SVM mask merging
    // Combine rgb & depth probabilities
    if(detectionMode == GOT_HOLES_AND_DEPTH) 
    {
      for(unsigned int i = 0 ; i < depth_svm_probabilities.size() ; i++)
      {
        //! Weighted mean
        temp.probability = 
          (VictimParameters::depth_svm_weight * 
            depth_svm_probabilities[i].probability + 
          VictimParameters::rgb_svm_weight * 
            rgb_svm_probabilities[i].probability) / 
          (VictimParameters::depth_svm_weight + 
            VictimParameters::rgb_svm_weight);
        
        temp.keypoint = depth_svm_probabilities[i].keypoint;
        temp.source = DEPTH_RGB_SVM;
        temp.boundingBox = depth_svm_probabilities[i].boundingBox;
        final_probabilities.push_back(temp);
      }
    }
    // Only rgb svm probabilities
    if(detectionMode == GOT_HOLES)
    {
      for(unsigned int i = 0 ; i < rgb_svm_probabilities.size() ; i++)
      {
        temp.probability = rgb_svm_probabilities[i].probability * 
          VictimParameters::rgb_svm_weight;
        temp.keypoint = rgb_svm_probabilities[i].keypoint;
        temp.source = RGB_SVM;
        temp.boundingBox = rgb_svm_probabilities[i].boundingBox;
        final_probabilities.push_back(temp);
      }
    }
    
    // VJ mask merging (?)
    for(unsigned int i = 0 ; i < rgb_vj_probabilities.size() ; i++)
    {
      temp.probability = rgb_vj_probabilities[i].probability * 
        VictimParameters::rgb_vj_weight;
      temp.keypoint = rgb_vj_probabilities[i].keypoint;
      temp.source = RGB_VJ;
      temp.boundingBox = rgb_vj_probabilities[i].boundingBox;
      final_probabilities.push_back(temp);
    }
    for(unsigned int i = 0 ; i < depth_vj_probabilities.size() ; i++)
    {
      temp.probability = depth_vj_probabilities[i].probability * 
        VictimParameters::depth_vj_weight;
      temp.keypoint = depth_vj_probabilities[i].keypoint;
      temp.source = DEPTH_VJ;
      temp.boundingBox = depth_vj_probabilities[i].boundingBox;
      final_probabilities.push_back(temp);
    }
    
    return final_probabilities;
  }


  /**
    * @brief Node's state manager
    * @param newState [int] The robot's new state
    * @return void
   */
  void VictimDetection::startTransition(int newState)
  {
    curState = newState;

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
   * @brief After completion of state transition
   * @return void
   */
  void VictimDetection::completeTransition(void)
  {
    ROS_INFO("[victim_node] : Transition Complete");
  }
}// namespace pandora_vision
