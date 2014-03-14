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
* Author:  George Aprilis
*********************************************************************/

#include "pandora_vision_motion/motion_detection.h"
namespace pandora_vision
{

  /**
    @brief Constructor
  **/
  MotionDetection::MotionDetection() :	_nh()
  {
    //!< Initialize motion detector
    _motionDetector		=	new MotionDetector();

    //!< Get Motion Detector Parameters
    getMotionParams();

    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();

    motionFrame = cv::Mat(cv::Size(frameWidth,frameHeight), CV_8UC3);

    //!< Declare publisher and advertise topic where algorithm results are posted
    _motionPublisher = _nh.advertise<vision_communications::MotionMsg>("motion", 10);

    //!< Advertise topics for debugging if we are in debug mode
    if (debugMotion)
    {
      _motionDiffPublisher = image_transport::ImageTransport(_nh).advertise("debug_motionDiff", 1);
      _motionFrmPublisher = image_transport::ImageTransport(_nh).advertise("debug_motionFrm", 1);
    }

    //!< Subscribe to input image's topic
    _frameSubscriber = image_transport::ImageTransport(_nh).subscribe(imageTopic , 1, &MotionDetection::imageCallback, this );

    //!< Initialize states - robot starts in STATE_OFF 
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    //!< initialize state Managing Variables
    motionNowON 	= false;
      
    clientInitialize();
      
    ROS_INFO("[Motion_node] : Created Motion Detection instance");
  }

  /**
    @brief Destructor
  */
  MotionDetection::~MotionDetection()
  {
    delete _motionDetector;
    ROS_INFO("[motion_node] : Destroying Motion Detection instance");
  }

  /**
   @brief Get parameters referring to view and frame characteristics from
   launch file
   @return void
  */
  void MotionDetection::getGeneralParams()
  {
    // Get the motionDummy parameter if available;
    if (_nh.hasParam("motionDummy")) 
    {
      _nh.getParam("motionDummy", motionDummy);
      ROS_DEBUG("motionDummy: %d", motionDummy);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter motionDummy not found. Using Default");
      motionDummy = false;
    }
    
    // Get the debugMotion parameter if available;
    if (_nh.hasParam("debugMotion")) 
    {
      _nh.getParam("debugMotion", debugMotion);
      ROS_DEBUG_STREAM("debugMotion : " << debugMotion);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter debugMotion not found. Using Default");
      debugMotion = true;
    }
    
    //!< Get the camera to be used by hole node;
    if (_nh.hasParam("camera_name")) 
    {
      _nh.getParam("camera_name", cameraName);
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    }
    else
    {
      ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
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
      ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
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
      ROS_DEBUG("[motion_node] : Parameter frameWidth not found. Using Default");
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
      ROS_DEBUG("[motion_node] : Parameter imageTopic not found. Using Default");
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
      ROS_DEBUG("[motion_node] : Parameter camera_frame_id not found. Using Default");
      cameraFrameId = "/camera";
    }
  }

  /**
    @brief Get parameters referring to motion detection algorithm
    @return void
  */
  void MotionDetection::getMotionParams()
  {
    //!< Get the buffer size parameter if available;
    if (_nh.hasParam("motionBuffer")) 
    {
      _nh.getParam("motionBuffer", _motionDetector->N);
      ROS_DEBUG_STREAM("motionBuffer : " << _motionDetector->N);
    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter motionBuffer not found. Using Default");
      _motionDetector->N = 4;
    }

    //!< Get the difference threshold parameter if available;
    if (_nh.hasParam("motionDiffThres")) 
    {
      _nh.getParam("motionDiffThres", _motionDetector->diff_threshold);
      ROS_DEBUG_STREAM("motionDiffThres : " << _motionDetector->diff_threshold);

    }
    else 
    {
      ROS_DEBUG("[motion_node] : Parameter motionDiffThres not found. Using Default");
      _motionDetector->diff_threshold = 45;
    }

    //!< Get the motion high threshold parameter if available;
    if (_nh.hasParam("motionHighThres")) 
    {
      _nh.getParam("motionHighThres", _motionDetector->motion_high_thres);
      ROS_DEBUG_STREAM("motionHighThres : " << _motionDetector->motion_high_thres);
    }
    else {
      ROS_DEBUG("[motion_node] : Parameter motionHighThres not found. Using Default");
      _motionDetector->motion_high_thres = 7500;
    }

    //!< Get the motion low threshold parameter if available;
    if (_nh.hasParam("motionLowThres")) {
      _nh.getParam("motionLowThres", _motionDetector->motion_low_thres);
      ROS_DEBUG_STREAM("motionLowThres : " << _motionDetector->motion_low_thres);
    }
    else {
      ROS_DEBUG("[motion_node] : Parameter motionLowThres not found. Using Default");
      _motionDetector->motion_low_thres = 200;
    }
  }

  /**
   @brief Function called when new ROS message appears, for front camera
   @param msg [const sensor_msgs::ImageConstPtr&] The message
   @return void
  */
  void MotionDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    motionFrame = in_msg->image.clone();
    motionFrameTimestamp = msg->header.stamp;

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
    //do detection and examine result cases
    switch (_motionDetector->detectMotion(motionFrame))
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
      ROS_INFO_STREAM( "Motion found with probability: "<< motionMessage.probability);
      _motionPublisher.publish(motionMessage);
    }

    if (debugMotion)
    {
      publish_debug_images();
    }
  }

  /**
   @brief Publishing debug images
   @return void
   */
  void MotionDetection::publish_debug_images()
  {
    cv_bridge::CvImage motionDiff;
    motionDiff.encoding = sensor_msgs::image_encodings::MONO8;
    motionDiff.image    = _motionDetector->getDiffImg().clone();
    _motionDiffPublisher.publish(motionDiff.toImageMsg());

    cv_bridge::CvImage motionFrm;
    motionFrm.encoding = sensor_msgs::image_encodings::BGR8;
    motionFrm.image    = motionFrame.clone();
    _motionFrmPublisher.publish(motionFrm.toImageMsg());
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
    motionNowON		=	( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );

    //!< Everytime state changes, Motion Detector needs to be reset so that it will
    //!< discard frames from previous calls in buffer.
    if(motionNowON)
    {
      _motionDetector->resetFlagCounter();
    }

    //!< Shutdown if the robot is switched off
    if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING)
    {
      ros::shutdown();
      return;
    }

    prevState=curState;

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
}
