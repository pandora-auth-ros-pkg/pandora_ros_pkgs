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
/////////////////////////////////////////////////////////////////

#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

using namespace std;

class MotionDetector
{
public:
	int			N;					//buffer size
	int 		diff_threshold;		//threshold between pixel (grayscale) values to be considered "different" between 2 frames
	double 		motion_high_thres;	//evaluation threshold: higher value means a lot of movement
	double 		motion_low_thres;	//evaluation threshold: higher value means a little movement - less means no movement at all

private:
	int 		flagCounter;		//Used to avoid wrong results in the first calculations
	int 		last;				//Last frame stored in buffer
	int 		count;				//Sum of different pixels between two frames (after thresholding)
	//due to empty initial frames in buffer
	cv::Mat  	*buf; 				//N-sized buffer
	cv::Mat 	tmp;				//Temporary copy of frame
	cv::Mat 	dif; 				//Image with the difference between 2 frames' pixels' values

public:
	MotionDetector();
	~MotionDetector();
	int 		detectMotion(cv::Mat frame);	//the main Motion Detector function
	int 		getCount();
	void 		resetFlagCounter();	//
	cv::Mat		getDiffImg();
};

#endif
