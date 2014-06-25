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
* Author:  Despoina Pascahlidou
*********************************************************************/

#ifndef PANDORA_VISION_MOTION_MOTION_DETECTOR_H
#define PANDORA_VISION_MOTION_MOTION_DETECTOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "pandora_vision_motion/motion_parameters.h"

//~ #define SHOW_DEBUG_IMAGE

namespace pandora_vision 
{
  class MotionDetector
  {
    private:
    //!< Current frame to be processed
    cv::Mat frame;
    //!< Background image
    cv::Mat background;
    //!< Foreground mask
    cv::Mat foreground;
    
    //!< Class instance for Gaussian Mixture-based backgound 
    //!< and foreground segmentation 
    cv::BackgroundSubtractorMOG2 bg;
    
    ///!< Erode kernel
    cv::Mat kernel_erode;
    public:
    
    //!< Number of pixels, that differ from current frame and background 
    int countDiff;
    //!< Identifier of motion type
    int typeOfMovement;
    
    int max_deviation;
    
    cv::Mat result;

    /**
      @brief Class Constructor
      Initializes all varialbes for thresholding
    */
    MotionDetector();
    
    /**
      @brief Class Destructor
      Deallocates memory used for storing images
    */
    ~MotionDetector();
    
    /**
      @brief Function that detects motion, according to substraction
      between background image and current frame. According to predifined 
      thresholds motion is detected. According to the type of motion
      the suitable value is returned.
      @param _frame [cv::Mat] current frame to be processed 
      @return [int] Index of evaluation of Motion in current frame.
    */
    int detectMotion(cv::Mat _frame);
    
    /**
      @brief Returns the number of different pixels found
      in a (thresholded) difference between background image and current frame.
      @return [int] countDiff 
    */
    int getCount();
    
    /**
      @brief Function that defines the type of movement 
      according to the number of pixels, that differ from current
      frame and background. In case insignificant motion 0 is detected
      0 is returned. If there is slight motion 1 is returned and last
      bust not least in case extensive motion is detected 2 is returned
      @return void
    */ 
    void motionIdentification(cv::Mat diff);
    
    /**
      @brief Function used for debug reasons, that shows background
      foreground and contours of motion trajectories in current frame
      @return void
    */ 
    void debugShow(cv::Mat diff);
    
    /**
     @brief Function that calculates motion's postion
     @param
     @return void 
    */
    void detectMotionPosition(cv::Mat diff); 
    
    /**@brief Creates the continuous table of motion in current frame
    @return int[] table of motion position and size
    */
    int* getMotionPosition();
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_MOTION_DETECTOR_H
