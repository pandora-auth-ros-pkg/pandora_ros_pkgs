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

#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

namespace pandora_vision 
{
  class MotionDetector
  {
    private:
    //!< Used to avoid wrong results in the first calculations
    int flagCounter;
    //!< Last frame stored in buffer		
    int last;
    //!< Sum of different pixels between two frames (after thresholding)
    int count;				
    //!< N-sized buffer
    cv::Mat *buf;
    //!< Temporary copy of frame 			
    cv::Mat tmp;
    //!< Image with the difference between 2 frames' pixels' values				
    cv::Mat dif; 				

    public:
     //!< buffer size
    int	N;					
    //!< Threshold between pixel (grayscale) values to be considered "different" between 2 frames
    int diff_threshold;		
    //!< Evaluation threshold: higher value means a lot of movement
    double motion_high_thres;	
    //!< Evaluation threshold: higher value means a little movement - less means no movement at all
    double motion_low_thres;	
    
    /**
      @brief Class Constructor
      Initializes varialbes used in detecting. Here is
      were desirable threashold values will be determined. 
    */
    MotionDetector();
    
    /**
      @brief Class Destructor
      Deallocates memory used for storing images
    */
    virtual ~MotionDetector();
    
    /**
      @brief Using a N-sized buffer compares current given frame
      with last - (N-1)th frame and calculates an the difference 
      between the two frames giving value to a variable integer "count" 
      with the number of different pixels. According to given thresholds,
      returns:
                -1				Error in frame input
                 0				Insignificant Motion
                 1				Slight Motion
                 2				Extensive Motion						
      Function detectMotion() of a specific MotionDetector is run in a loop.	  
      @param	frame [cv::Mat] The current frame given as input 
      @return [int] Index of evaluation of Motion in N frames.
    */
    int detectMotion(cv::Mat frame);	
    
    /**
      @brief Returns the number of different pixels found
      in a (thresholded) difference between frames.
      According to this value, motion is evaluated internally in detectMotion().
      @return 		integer count 
    */
    int getCount();
    
    /**
      @brief Function called in the ROS node, used to reset the flagCounter 
      value, which causes the algorithm to re-wait until the buffer of frames 
      is full and results can be trusted again.		   
      @return 		void 
    */
    void resetFlagCounter();	
    
    /**
      @brief Function that returns the last image of the buffer of frames
      which holds the difference image. Being called
      in the ROS Node after the agorithm has run,
      retrieves the result image for debugging.		   
      @return [int] count 
    */
    cv::Mat getDiffImg();
  };
}
#endif
