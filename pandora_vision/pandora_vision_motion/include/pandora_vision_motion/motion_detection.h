/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.cd
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
 
#ifndef PANDORA_VISION_MOTION_MOTION_DETECTION_H 
#define PANDORA_VISION_MOTION_MOTION_DETECTION_H 

#include "ros/ros.h"

#include "pandora_vision_motion/motion_detector.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "state_manager/state_client.h"

#include <iostream>
#include <stdlib.h>
#include <urdf_parser/urdf_parser.h>
#include <map>

namespace pandora_vision
{
  class MotionDetection : public StateClient 
  {
    private:
      /// nodeHandle
      ros::NodeHandle _nh;
      /// Instance of class MotionDetector
      MotionDetector _motionDetector;
            
      /// Horizontal Field Of View (rad)
      double hfov;
      /// Vertical Field Of View (rad)
      double vfov;
      
      /// Frame width
      int frameWidth; 
      /// Frame height
      int frameHeight; 
      ///Current frame to be processed
      cv::Mat motionFrame; 
      /// MotionDetector frame timestamp
      ros::Time motionFrameTimestamp; 

      std::string imageTopic;
      std::string cameraName;
      
      std::string _parent_frame_id; 
      std::string _frame_id;
      
      /// Publishers for MotionDetector result messages
      ros::Publisher _motionPublisher;
      
      /// Subscriber that listens to the frame topic advertised by the central node
      ros::Subscriber _frameSubscriber;
                
      /// Variable used for State Managing
      bool motionNowON;
      
      //!< The dynamic reconfigure (motion's) parameters' server
      dynamic_reconfigure::Server<pandora_vision_motion::motion_cfgConfig>
        server;
      //!< The dynamic reconfigure (depth) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_motion::motion_cfgConfig>
        ::CallbackType f;  
      
       /**
        @brief This method uses a MotionDetector instance to detect motion
        in current frame.
        @return void
      */
      void motionCallback();

       /**
        @brief Function called when new ROS message appears, for front camera
        @param msg [const sensor_msgs::Image&] The message
        @return void
      */
      void imageCallback(const sensor_msgs::Image& msg);
      
      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
        @param[in] level [const uint32_t] The level 
        @return void
      **/
      void parametersCallback(
        const pandora_vision_motion::motion_cfgConfig& config,
        const uint32_t& level);
      
      /**
        @brief Get parameters referring to view and frame characteristics from
        launch file
        @return void
      */
      void getGeneralParams();
      
      /**
        @brief Function that retrieves the parent to the frame_id
        @return bool Returns true is frame_id found or false if not 
      */ 
      bool getParentFrameId();
      
      std::map<std::string, std::string> _frame_ids_map;
           
    public:
          
      /**
        @brief Constructor
      **/
      explicit MotionDetection(const std::string& ns);
            
      /**
        @brief Destructor
      */
      ~MotionDetection();
      
      /**
        @brief Node's state manager
        @param newState [int] The robot's new state
        @return void
      */
      void startTransition(int newState);
      
      /**
         @brief After completion of state transition
         @return void
      */
      void completeTransition(void);
      
      //!< Current state of robot
      int curState;
      //!< Previous state of robot
      int prevState;
      
      std::string param;
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_MOTION_DETECTION_H
