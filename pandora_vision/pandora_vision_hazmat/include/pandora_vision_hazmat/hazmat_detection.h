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
      ros::NodeHandle _nh;
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
      std::string imageTopic;
      std::string cameraName;
          
      //publisher
      ros::Publisher hazmatPublisher_;
      
      //!< The topics subscribed to all cameras
      std::vector<std::string> _imageTopics;
      
      //!< The subscribers that listens to the frame topic advertised by the
      //!< central node for all cameras
      std::vector<ros::Subscriber> _frameSubscribers;
      
      ros::Subscriber frameSubscriber;
            
      //variable used for State Managing
      bool hazmatNowOn_;
      
      int curState; //Current state of robot
      
      int prevState;  //Previous state of robot
      
      //!< The dynamic reconfigure (motion's) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hazmat::hazmat_cfgConfig>
        server;
      //!< The dynamic reconfigure (depth) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hazmat::hazmat_cfgConfig>
        ::CallbackType f; 
      
      /**
        @brief Method called only when a new image message is present
        @return void
      **/
      void hazmatDetect(std::string frame_id);
      
      /**
        @brief Callback for a new image
        @param msg [const sensor_msgs::Image&] The new image
        @return void
      **/
      void imageCallback(const sensor_msgs::Image& msg);
      
      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hazmat::hazmat_cfgConfig&]
        @param[in] level [const uint32_t] The level 
        @return void
      **/
      void parametersCallback(
        const pandora_vision_hazmat::hazmat_cfgConfig& config,
        const uint32_t& level);
    public:
          
      /**
      @brief Default constructor
      @return void
      **/
      explicit HazmatDetection(const std::string& ns);
            
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
      
      std::string param;
  };

} // namespace pandora_vision

#endif  // PANDORA_VISION_HAZMAT_HAZMAT_DETECTION_H
    
    
