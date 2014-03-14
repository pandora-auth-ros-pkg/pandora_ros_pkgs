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
* Authors:  Tsakalis Vasilis, Despoina Paschalidou
*********************************************************************/
 
#ifndef PANDORA_VISION_HAZMAT_HAZMAT_DETECTION_H
#define PANDORA_VISION_HAZMAT_HAZMAT_DETECTION_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/**
@namespace pandora_vision
@brief The general purpose pandora vision namespace
**/
namespace pandora_vision
{

  /**
  @class HazmatDetection
  @brief The hazmat detection class. Inherits the StateClient
  **/
  class HazmatDetection : public StateClient 
  {
    private:
      
      //nodeHandle
      ros::NodeHandle nh_;
      HazmatEpsilonDetector* hazmatDetector_;
      float ratioX_;
      float ratioY_;
      
      double hfov_;  //horizontal Field Of View (rad)
      double vfov_;
      int frameWidth_; //frame width
      int frameHeight_;  //frame height
      
      cv::Mat hazmatFrame_;  // frame processed by HazmatDetector
      
      ros::Time hazmatFrameTimestamp_; // HazmatDetector frame timestamp
      
      std::string packagePath_;
      std::string saveImagePath_;
      std::string imageTopic_;
      std::string cameraName;
      std::string cameraFrameId;
      
      int hazmatNumber_;
      
      //publisher
      ros::Publisher hazmatPublisher_;

      image_transport::Subscriber sub_;
      
      // variables for changing in dummy msg mode for debugging
      bool hazmatDummy_;
      
      //variable used for State Managing
      bool hazmatNowOn_;
      
      int curState; //Current state of robot
      
      int prevState;  //Previous state of robot
      
    public:
          
      /**
      @brief Default constructor
      @return void
      **/
      HazmatDetection(void);
            
      /**
      @brief Default destructor
      @return void
      **/
      ~HazmatDetection(void);
      
      /**
      @brief Reads the general parameters from the launch file
      @return void
      **/
      void getGeneralParams(void);
      
      /**
      @brief Reads the hazmat - specific parameters from the launch file
      @return void
      **/
      void getHazmatParams(void);

      /**
      @brief Method called only when a new image message is present
      @return void
      **/
      void hazmatCallback(void);
      
      /**
      @brief Callback for a new image
      @param msg [const sensor_msgs::ImageConstPtr&] The new image
      @return void
      **/
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);
      
      /**
      @brief Implemented from state manager. Called when a new transition \
      happens
      @param newState [int] The new state of the system
      @return void
      **/
      void startTransition(int newState);
      
      /**
      @brief Called when the transition completes
      @return void
      **/
      void completeTransition(void);

  };

} // namespace pandora_vision

#endif  // PANDORA_VISION_HAZMAT_HAZMAT_DETECTION_H
    
    
