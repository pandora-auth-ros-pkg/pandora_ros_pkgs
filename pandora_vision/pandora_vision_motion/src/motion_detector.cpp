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
* Author:  Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_motion/motion_detector.h"

namespace pandora_vision 
{
  
  /**
    @brief Class Constructor
    Initializes all varialbes for thresholding
  */
  MotionDetector::MotionDetector()
  { 
    kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    bg = 
    cv::BackgroundSubtractorMOG2 (MotionParameters::history,
      MotionParameters::varThreshold, MotionParameters::bShadowDetection);
    countDiff = 0;
    max_deviation = 50;
    ROS_INFO("Created MotionDetector instance");
  }

  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  MotionDetector::~MotionDetector()
  {
    ROS_INFO("Destroying MotionDetector instance");
  }

  /**
    @brief Function that detects motion, according to substraction
    between background image and current frame. According to predifined 
    thresholds motion is detected. According to the type of motion
    the suitable value is returned.
    @param _frame [cv::Mat] current frame to be processed 
    @return [int] Index of evaluation of Motion in current frame.
  */
  int MotionDetector::detectMotion(cv::Mat _frame)
  {  
    frame = _frame.clone(); 
    result = _frame.clone(); 
    
    /// Upadate the background model and create 
    /// binary mask for foreground objects
    bg.operator ()(frame, foreground);
    bg.getBackgroundImage(background);
        
    cv::Mat temp;
    cv::subtract(frame, background, temp);
    cv::cvtColor(temp, temp, CV_BGR2GRAY);
    cv::threshold(temp, temp, MotionParameters::diff_threshold, 
      255, cv::THRESH_BINARY);
    motionIdentification(temp);
    cv::Mat kernel = 
      getStructuringElement(cv::MORPH_CROSS , cv::Size(3, 3), cv::Point( -1, -1 ));
    cv::morphologyEx(temp, temp, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 8);
 
    detectMotionPosition(temp);
     
    ROS_INFO_STREAM("MotionParameters::visualization" << MotionParameters::visualization); 
    if(MotionParameters::visualization)
      debugShow(temp);
   
    return typeOfMovement;
  }
  
  /**
    @brief Function that calculates motion's postion
    @param
    @return void 
  */
  void MotionDetector::detectMotionPosition(cv::Mat diff)
  {
    /// Calculate the standard deviation
    cv::Scalar mean, stddev;
    meanStdDev(diff, mean, stddev);
    /// If not to much changes then the motion is real 
    if(stddev[0] < max_deviation)
    {
      int number_of_changes = 0;
      int min_x = diff.cols, max_x = 0;
      int min_y = diff.rows, max_y = 0;
      /// Loop over image and detect changes
      for(int j = 1; j < diff.cols-11; j+=2){ // height
        for(int i = 1; i < diff.rows-11; i+=2){ // width
            if(static_cast<int>(diff.at<uchar>(j, i)) == 255){
                number_of_changes++;
                if(min_x > i) 
                  min_x = i;
                if(max_x < i) 
                  max_x = i;
                if(min_y > j) 
                  min_y = j;
                if(max_y < j) 
                  max_y = j;
                }
            }
        }
        if(number_of_changes){
          cv::Point _tlcorner(min_x, min_y);
          cv::Point _brcorner(max_x, max_y);
          
          rectangle(result, _tlcorner, _brcorner, cv::Scalar(0, 255, 255), 1);
          cv::Rect rect(_tlcorner.x, _tlcorner.y, max_x- min_x, max_y -min_y);
          _bounding_box = rect;
        }  
    }
  }
  
  /**
    @brief Creates the continuous table of motion in current frame
    @return int[] table of motion position and size
  */
  int* MotionDetector::getMotionPosition()
  {
    int* table = new int[4];
    
    //!< Center_x
    table [0] = round( _bounding_box.x + _bounding_box.width * 0.5 );
    //!< Center_y
    table [1] = round( _bounding_box.y + _bounding_box.height * 0.5 );
    
    //!< Rectangle width
    table[2] = _bounding_box.width;

    //!< Rectangle height
    table[3] = _bounding_box.height;
    return table;
  }
    
  /**
    @brief Function that defines the type of movement 
    according to the number of pixels, that differ from current
    frame and background. In case insignificant motion 0 is detected
    0 is returned. If there is slight motion 1 is returned and last
    bust not least in case extensive motion is detected 2 is returned
    @return void
  */ 
  void MotionDetector::motionIdentification(cv::Mat diff)
  {
    //!< counts value of non zero pixels in binary image
    countDiff = countNonZero(diff);
    if (countDiff > MotionParameters::motion_high_thres)
      typeOfMovement = 2;
    else if (countDiff > MotionParameters::motion_low_thres)
      typeOfMovement = 1;
    else
      typeOfMovement = 0;
  }
  
  /**
    @brief Returns the number of different pixels found
    in a (thresholded) difference between background image and current frame.
    @return [int] countDiff 
  */
  int MotionDetector::getCount()
  {
    return countDiff;
  }
  
  /**
    @brief Function used for debug reasons, that shows background
    foreground and contours of motion trajectories in current frame
    @return void
  */ 
  void MotionDetector::debugShow(cv::Mat diff)
  {
    std::vector<std::vector<cv::Point> > contours;
    cv::erode(foreground, foreground, cv::Mat());
    cv::dilate(foreground, foreground, cv::Mat());
    cv::findContours(foreground, contours, CV_RETR_EXTERNAL, 
      CV_CHAIN_APPROX_NONE);
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);
    cv::imshow("Frame", frame);
    cv::imshow("Background", background);
    cv::imshow("Diff", diff);
    cv::imshow("Result", result);
    cv::waitKey(10);
  }
     
}// namespace pandora_vision
