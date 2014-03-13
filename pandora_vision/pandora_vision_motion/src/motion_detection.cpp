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

using namespace std;

/**
 * Constructor
 */
MotionDetection::MotionDetection() :	_nh() {
	//initialize motion detector
	_motionDetector		=	new MotionDetector();

	//Get Motion Detector Parameters
	getMotionParams();

	// Get General Parameters, such as frame width & height , camera id
	getGeneralParams();

	//memory will be allocated in the imageCallback
	//motionFrame = 0;

	extraFrame = cv::Mat(cv::Size(frameWidth,frameHeight), CV_8UC3);

	//Declare publisher and advertise topic where algorithm results are posted
	_motionPublisher = _nh.advertise<vision_communications::MotionMsg>("motion", 10);

	//Advertise topics for debugging if we are in debug mode
	if (debugMotion)
	{
		_motionDiffPublisher = image_transport::ImageTransport(_nh).advertise("debug_motionDiff", 1);
		_motionFrmPublisher = image_transport::ImageTransport(_nh).advertise("debug_motionFrm", 1);
	}

	//subscribe to input image's topic
	//image_transport::ImageTransport it(_nh);
	_frameSubscriber = image_transport::ImageTransport(_nh).subscribe(imageTopic , 1, &MotionDetection::imageCallback, this );

	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;

	//initialize state Managing Variables
	motionNowON 	= false;
    
    clientInitialize();
    
	ROS_INFO("[Motion_node] : Created Motion Detection instance");
}

/**
 * Destructor
 */
MotionDetection::~MotionDetection()
{
	ROS_INFO("[motion_node] : Destroying Motion Detection instance");
	delete _motionDetector;
}

/**
 * Get parameters referring to view	and frame characteristics
 */
void MotionDetection::getGeneralParams()
{
  // Get the motionDummy parameter if available;
  if (_nh.hasParam("motionDummy")) {
    _nh.getParam("motionDummy", motionDummy);
    ROS_DEBUG("motionDummy: %d", motionDummy);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter motionDummy not found. Using Default");
    motionDummy = false;
  }
  
  // Get the debugMotion parameter if available;
  if (_nh.hasParam("debugMotion")) {
    _nh.getParam("debugMotion", debugMotion);
    ROS_DEBUG_STREAM("debugMotion : " << debugMotion);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter debugMotion not found. Using Default");
    debugMotion = true;
  }
  
  //!< Get the camera to be used by hole node;
  if (_nh.hasParam("camera_name")) {
    _nh.getParam("camera_name", cameraName);
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
    cameraName = "camera";
  }

  //!< Get the Height parameter if available;
  if (_nh.hasParam("/" + cameraName + "/image_height")) {
    _nh.getParam("/" + cameraName + "/image_height", frameHeight);
    ROS_DEBUG_STREAM("height : " << frameHeight);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter frameHeight not found. Using Default");
    frameHeight = DEFAULT_HEIGHT;
  }
  
  //!< Get the Width parameter if available;
  if (_nh.hasParam("/" + cameraName + "/image_width")) {
    _nh.getParam("/" + cameraName + "/image_width", frameWidth);
    ROS_DEBUG_STREAM("width : " << frameWidth);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter frameWidth not found. Using Default");
    frameWidth = DEFAULT_WIDTH;
  }
  
  //!< Get the images's topic;
  if (_nh.hasParam("/" + cameraName + "/topic_name")) {
    _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter imageTopic not found. Using Default");
    imageTopic = "/camera_head/image_raw";
  }

  //!< Get the images's frame_id;
  if (_nh.hasParam("/" + cameraName + "/camera_frame_id")) {
    _nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId);
    ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
  }
  else {
    ROS_DEBUG("[motion_node] : Parameter camera_frame_id not found. Using Default");
    cameraFrameId = "/camera";
  }
}

/**
 * Get parameters referring to motion detection algorithm
 */
void MotionDetection::getMotionParams()
{
	// Get the buffer size parameter if available;
	if (_nh.hasParam("motionBuffer")) {
		_nh.getParam("motionBuffer", _motionDetector->N);
		ROS_DEBUG_STREAM("motionBuffer : " << _motionDetector->N);
	}
	else {
		ROS_DEBUG("[motion_node] : Parameter motionBuffer not found. Using Default");
		_motionDetector->N = 4;
	}

	// Get the difference threshold parameter if available;
	if (_nh.hasParam("motionDiffThres")) {
		_nh.getParam("motionDiffThres", _motionDetector->diff_threshold);
		ROS_DEBUG_STREAM("motionDiffThres : " << _motionDetector->diff_threshold);

	}
	else {
		ROS_DEBUG("[motion_node] : Parameter motionDiffThres not found. Using Default");
		_motionDetector->diff_threshold = 45;
	}

	// Get the motion high threshold parameter if available;
	if (_nh.hasParam("motionHighThres")) {
		_nh.getParam("motionHighThres", _motionDetector->motion_high_thres);
		ROS_DEBUG_STREAM("motionHighThres : " << _motionDetector->motion_high_thres);
	}
	else {
		ROS_DEBUG("[motion_node] : Parameter motionHighThres not found. Using Default");
		_motionDetector->motion_high_thres = 7500;
	}

	// Get the motion low threshold parameter if available;
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
 * Called only when a ROS image message is present
 * @param msg
 */
void MotionDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if(!motionNowON){
		return;
	}

	cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	motionFrame = in_msg->image.clone();
	motionFrameTimestamp = msg->header.stamp;

	if ( motionFrame.empty() )
	{               
		ROS_ERROR("[motion_node] : No more Frames or something went wrong with bag file");
		ros::shutdown();
		return;
	}
	motionFrame.copyTo(extraFrame);

	motionDetectAndPost();
}


/**
 * Detects motion and publishes a message
 */
void MotionDetection::motionDetectAndPost()
{
	//create message of Motion Detector
	vision_communications::MotionMsg motionMessage;

	if (motionDummy)
	{
		/*
		 * Motion Dummy Message
		 */
		int temp = 1;
		switch (temp)
		{
		case 0:
			motionMessage.probability = 0;
			break;
		case 1:
			motionMessage.probability = 0.5;
			break;
		case 2:
			motionMessage.probability = 1;
			break;
		default:
			motionMessage.probability = -1;
			ROS_INFO("Unable to get frame for motion detection");
			break;
		}
		//motionMessage.x = 0;
		//motionMessage.y = 0;
		//motionMessage.area = 1000;
		motionMessage.header.frame_id="Motion";
		//motionMessage.type = vision_communications::victimIdentificationDirectionMsg::MOTION;
		motionMessage.header.stamp = ros::Time::now();
		_motionPublisher.publish(motionMessage);

		//dummy delay
		usleep(1000 * 60);
	}
	else
	{
		/*
		 * Motion Message
		 */

		//do detection and examine result cases
		switch (_motionDetector->detectMotion(extraFrame))
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
		if(motionMessage.probability > 0.1){
			//motionMessage.x = -1;
			//motionMessage.y = -1;
			//motionMessage.area = _motionDetector->getCount();
			motionMessage.header.frame_id = cameraFrameId;
			//motionMessage.type = vision_communications::victimIdentificationDirectionMsg::MOTION;
			motionMessage.header.stamp = ros::Time::now();
			std::cout << "Motion found with probability: "<< motionMessage.probability<<std::endl;
			_motionPublisher.publish(motionMessage);
		}

		if (debugMotion){
			publish_debug_images();
		}
	}
}

/**
 * Publishes dummy ROS messages, for debugging purposes
 */
void MotionDetection::publish_debug_images()
{
	cv_bridge::CvImage motionDiff;
	motionDiff.encoding = sensor_msgs::image_encodings::MONO8;
	motionDiff.image    = _motionDetector->getDiffImg().clone();
	_motionDiffPublisher.publish(motionDiff.toImageMsg());

	cv_bridge::CvImage motionFrm;
	motionFrm.encoding = sensor_msgs::image_encodings::BGR8;
	motionFrm.image    = extraFrame.clone();
	_motionFrmPublisher.publish(motionFrm.toImageMsg());
}

/**
 * Starts node's state transition
 * @param newState
 */
void MotionDetection::startTransition(int newState){
	
    curState = newState;
	//check if motion algorithm should be running now
	motionNowON		=	( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );

	//everytime state changes, Motion Detector needs to be reset so that it will
	//discard frames from previous calls in buffer.
	if(motionNowON){
		_motionDetector->resetFlagCounter();
	}

	//shutdown if the robot is switched off
	if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING){
		ros::shutdown();
		return;
	}

	prevState=curState;

	transitionComplete(curState); //this needs to be called everytime a node finishes transition
}

/**
 * Called when state transition is completed
 */
void MotionDetection::completeTransition(void){
	ROS_INFO("[motion_node] : Transition Complete");
}

/**
 * Node's main method
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{	
	ros::init(argc,argv,"motion_node");
	MotionDetection motionDetection;
	ros::spin();
	return 0;	
}
