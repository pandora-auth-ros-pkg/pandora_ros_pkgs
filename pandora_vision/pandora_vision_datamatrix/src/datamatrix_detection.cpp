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

#include "pandora_vision_datamatrix/datamatrix_detection.h"

namespace pandora_vision
{
  /**
   *@brief Constructor
  **/
  DatamatrixDetection::DatamatrixDetection() : _nh(), datamatrixNowON(false)
  {
    
    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();
    
    //!< Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;
    
    //!< subscribe to input image's topic
    _frameSubscriber = _nh.subscribe(
        imageTopic, 1, &DatamatrixDetection::imageCallback, this);
     
    //!< Declare publisher and advertise topic
    //!< where algorithm results are posted
    _datamatrixCodePublisher =
      _nh.advertise<vision_communications::DataMatrixAlertsVectorMsg>("datamatrix_alert", 10, true);
      
    //!< initialize states - robot starts in STATE_OFF
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();

    ROS_INFO("[Datamatrix_node] : Created Datamatrix Detection instance");
    
  }
  
  
  
  /**
    @brief Destructor
   */
  DatamatrixDetection::~DatamatrixDetection()
  {
    ROS_INFO("[Datamatrix_node] : Destroying datamatrix Detection instance");
  }
  
  
  
  /**
   * @brief Get parameters referring to view and frame characteristics 
   * from launch file
   * @return void
   */
  void DatamatrixDetection::getGeneralParams()
  {
    
    packagePath = ros::package::getPath("pandora_vision_datamatrix");
    
    //!< Get the camera to be used by hole node;
    if (_nh.hasParam("camera_name"))
    {
      _nh.getParam("camera_name", cameraName);
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter frameHeight not found. Using Default");
      cameraName = "camera";
    }

    //!< Get the Height parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_height"))
    {
      _nh.getParam("/" + cameraName + "/image_height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter frameHeight not found. Using Default");
      frameHeight = DEFAULT_HEIGHT;
    }

    //!< Get the Width parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_width"))
    {
      _nh.getParam("/" + cameraName + "/image_width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
      frameWidth = DEFAULT_WIDTH;
    }

    //!< Get the images's topic;
    if (_nh.hasParam("/" + cameraName + "/topic_name"))
    {
      _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
      ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter imageTopic not found. Using Default");
      imageTopic = "/camera_head/image_raw";
    }

    //!< Get the images's frame_id;
    if (_nh.hasParam("/" + cameraName + "/camera_frame_id"))
    {
      _nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
      ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter camera_frame_id not found. Using Default");
      cameraFrameId = "/camera";
    }

    //!< Get the HFOV parameter if available;
    if (_nh.hasParam("/" + cameraName + "/hfov"))
    {
      _nh.getParam("/" + cameraName + "/hfov", hfov);
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
      hfov = HFOV;
    }

    //!< Get the VFOV parameter if available;
    if (_nh.hasParam("/" + cameraName + "/vfov"))
    {
      _nh.getParam("/" + cameraName + "/vfov", vfov);
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    }
    else
    {
      ROS_DEBUG("[face_node] : Parameter frameWidth not found. Using Default");
      vfov = VFOV;
    }
  }
  
  /**
   * @brief Function called when new ROS message appears from camera
   * @param msg [const sensor_msgs::Image&] The message
   * @return void
  */
  void DatamatrixDetection::imageCallback(
      const sensor_msgs::Image& msg)
  {
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    datamatrixFrame = in_msg->image.clone();
    datamatrixFrameTimestamp = msg.header.stamp;

    if (!datamatrixFrame.data)
    {
      ROS_ERROR("[Datamatrix_node] : \
          No more Frames!");
      return;
    }

    datamatrixCallback();
  }
  
  
  
  /**
   * @brief This method uses a DatamatrixDetector instance to detect 
   * all present datamatrixes in a given frame
   * @return void
  */
  void DatamatrixDetection::datamatrixCallback()
  {
    if(!datamatrixNowON)
    {
      return;
    }
    //!< Create message of DatamatrixCode Detector
    vision_communications::DataMatrixAlertsVectorMsg datamatrixcodeVectorMsg;
    vision_communications::DataMatrixAlertMsg datamatrixcodeMsg;
    datamatrixcodeVectorMsg.header.frame_id = cameraFrameId;
    datamatrixcodeVectorMsg.header.stamp = ros::Time::now();

    _datamatrixDetector.detect_datamatrix(datamatrixFrame);
    std::vector<DataMatrixQode> list_datamatrixes = _datamatrixDetector.get_detected_datamatrix();
    
    for(int i = 0; i < static_cast<int>(list_datamatrixes.size()); i++)
    {
      datamatrixcodeMsg.datamatrixContent = list_datamatrixes[i].message;
      datamatrixcodeMsg.yaw = ratioX *
        (list_datamatrixes[i].datamatrix_center.x - 
          static_cast<double>(frameWidth) / 2);
      datamatrixcodeMsg.pitch = -ratioY *
        (list_datamatrixes[i].datamatrix_center.y - 
          static_cast<double>(frameWidth) / 2);
      datamatrixcodeVectorMsg.dataMatrixAlerts.push_back(datamatrixcodeMsg);

      ROS_INFO("[Datamatrix_node]:Datamatrix found.");
    }

    if(datamatrixcodeVectorMsg.dataMatrixAlerts.size() > 0)
    {
      _datamatrixCodePublisher.publish(datamatrixcodeVectorMsg);
    }
  }
  
  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
  */
  void DatamatrixDetection::startTransition(int newState)
  {
    curState = newState;

    //!< check if datamatrix algorithm should be running now
    datamatrixNowON =
      (curState ==
       state_manager_communications::robotModeMsg::MODE_EXPLORATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_IDENTIFICATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_ARM_APPROACH)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_DF_HOLD);

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
   * @brief After completion of state transition
   * @return void
   */
  void DatamatrixDetection::completeTransition()
  {
    ROS_INFO("[Datamatrix_node] : Transition Complete");
  }
  
}// namespace pandora_vision
