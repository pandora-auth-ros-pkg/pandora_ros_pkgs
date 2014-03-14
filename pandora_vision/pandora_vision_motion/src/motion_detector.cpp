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

#include "pandora_vision_motion/motion_detector.h"

namespace pandora_vision 
{
  /**
    @brief Class Constructor
    Initializes varialbes used in detecting. Here is
    were desirable threashold values will be determined. 
   */
  MotionDetector::MotionDetector()
  {
    //!< Buffer size depends on how "old" will be the frame that is
    //!< compaired with the current frame.
    N = 4;		
    
    //!< Threshold of absolute difference in value between 2 pixels
    //!< so that the pixels can be considered as "different"				
    diff_threshold = 45;	
    
    //!< Minimum mumber of pixels that must be "different" in the
    //!< two frames so that motion can be evaluated as Extensive
    motion_high_thres = 7500;	
    
    //!< Minimum number of pixels that must be "differenet" in the
    //!< two frames so that motion can be evaluated as Slight             
    motion_low_thres = 200;		
                  
    buf = 0;
   
    last = 0;
    count = 0;
    flagCounter = 0;

    ROS_INFO("Created MotionDetector instance");
  }

  /**
   @brief Class Destructor
   Deallocates memory used for storing images
  */
  MotionDetector::~MotionDetector()
  {	
    if ( buf != 0 )
    {
      delete(buf);
    }
    
    ROS_INFO("Destroying MotionDetector instance");
  }

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
  int MotionDetector::detectMotion(cv::Mat frame)
  {	
    int idx1, idx2, result = 0;
    //!< Check if frame was retrieved successfully
    if(frame.empty()) 
        return -1;
    //!< Allocate buffer if it's the first time
    if( buf == 0 ) 
    {
      buf = new cv::Mat[N];
      for(int i=0 ; i<N ; i++)
      {
        buf[i]=cv::Mat::zeros(frame.size(),CV_8UC1);
      }
    }
    //!< Make a temporary copy of current frame
    tmp = frame.clone();
    //!< Smooth image
    GaussianBlur(tmp, tmp, cv::Size(21,21),0 ); 
    idx1 = last;
    cv::cvtColor(tmp,buf[last],CV_BGR2GRAY);	
    //!< Index of (last - (N-1))th frame
    idx2 = (last + 1) % N; 						
    last = idx2;
    dif = buf[idx2];
    //!< Calculate absolute difference and threshold frame
    absdiff(buf[idx1],buf[idx2],dif);			
    threshold(dif,dif,diff_threshold,255,CV_THRESH_BINARY);	
    erode(dif,dif,cv::Mat(),cv::Point(),4);

    //!< Buffer will be emty for the first frames so comparison will be wrong
    //!< A safe value N+1 is chosen to start returning results
    if(flagCounter>N+1)
    {
      //!< counts value of non zero pixels in binary image
      count = countNonZero(dif);
      
      if (count > motion_high_thres)
      {
        result = 2;
      }
      else if (count > motion_low_thres)
      {
        result = 1;
      }
      else
      { 
        result = 0; 
      }
    }
    //!< Checks because flagCounter isn't needed after first few frames
    //!< avoids getting huge values after long frame processing
    if (flagCounter < 2*N) 
        flagCounter++; 	

    return result;
  }

  /**
   @brief Returns the number of different pixels found
   in a (thresholded) difference between frames.
   According to this value, motion is evaluated internally in detectMotion().
   @return 		integer count 
  */
  int MotionDetector::getCount()
  {
    return count;
  }

  /**
   @brief Function called in the ROS node, used to reset the flagCounter 
   value, which causes the algorithm to re-wait until the buffer of frames 
   is full and results can be trusted again.		   
   @return 		void 
  */
  void MotionDetector::resetFlagCounter()
  {
    this->flagCounter = 0;
  }

  /**
   @brief Function that returns the last image of the buffer of frames
   which holds the difference image. Being called
   in the ROS Node after the agorithm has run,
   retrieves the result image for debugging.		   
   @return [int] count 
   */
  cv::Mat MotionDetector::getDiffImg()
  {
    if ( buf!=0 )
      return buf[last];
    else
      return cv::Mat();
  }
}
