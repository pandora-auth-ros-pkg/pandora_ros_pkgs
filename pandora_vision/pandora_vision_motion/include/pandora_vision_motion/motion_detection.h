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
 
 #ifndef MOTIONDETECTION_H
#define  MOTIONDETECTION_H

#include "ros/ros.h"

#include "vision_communications/MotionMsg.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "motion_detector.h"


#include "state_manager/state_client.h"

#include <iostream>
#include <stdlib.h>

#define DEFAULT_HEIGHT			480		//default frame height
#define DEFAULT_WIDTH			640		//default frame width


class MotionDetection : public StateClient {
	private:

		//nodeHandle
		ros::NodeHandle _nh;
		MotionDetector*	_motionDetector;
		float ratioX;
		float ratioY;
		
		float hfov;		//horizontal Field Of View (rad)
		float vfov;		
		int frameWidth;		//frame width
		int frameHeight;	//frame height
		
		cv::Mat		motionFrame;				// frame processed by MotionDetector
		cv::Mat		extraFrame;					// copy frame processed by MotionDetector
		
		ros::Time		motionFrameTimestamp;		// MotionDetector frame timestamp

		string imageTopic;
		string cameraName;
		string cameraFrameId;
		
		//publishers for MotionDetector result messages
		ros::Publisher _motionPublisher;
		
		//the subscriber that listens to the frame topic advertised by the central node
		image_transport::Subscriber _frameSubscriber;
		
		//debug publisher for MotionDetector
		image_transport::Publisher _motionDiffPublisher;
		image_transport::Publisher _motionFrmPublisher;
		
		// variables for changing in dummy msg mode for debugging
		bool motionDummy;
		// variables for changing in debug mode. Publish images for debugging
		bool debugMotion;
		
		//variable used for State Managing
		bool motionNowON;
	
		void publish_debug_images();
	public:
				
		//constructor
		MotionDetection();
					
		//destructor			
		~MotionDetection();	
		
		//get parameters from launch file
		void getGeneralParams();
		void getMotionParams();
		
		//executes motion Detector when in state STATE_ARM_SEARCH_COMPLETED
		void motionDetectAndPost();

		//get a new image
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		
		//Implemented from StateClient
		void startTransition(int newState);
		void completeTransition(void);
		
		int curState;		//Current state of robot
		int prevState;		//Previous state of robot
};

#endif
		
		
