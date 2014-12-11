/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/
#ifndef PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H
#define PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H

#include "ros/ros.h"
#include <ros/package.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <stdlib.h>
#include "state_manager/state_client.h"
#include "pandora_vision_msgs/LandoltcAlertsVectorMsg.h"
#include "pandora_vision_landoltc/landoltc_2d/landoltc_detector.h"
#include <urdf_parser/urdf_parser.h>
#include <map>


namespace pandora_vision
{
class LandoltCDetection : public StateClient {
  
private:
  //!<Subscriber of RGB Image
  ros::Subscriber _inputImageSubscriber;
  
  //!<Node Handler
  ros::NodeHandle _nh;
  
  //!< Current frame to be processed
  cv::Mat landoltCFrame;
  
  //!< Landoltc frame timestamp
  ros::Time landoltcFrameTimestamp;
  
  //!<Landoltc Detector object
  LandoltCDetector _landoltcDetector;
  
  //!<Current package path
  std::string packagePath;
  
  //!<Current pattern path
  std::string patternPath;
  
  /// Frame height
  int frameHeight;
  
  /// Frame width
  int frameWidth;
  
  //!< Horizontal Field Of View (rad)
  double hfov;

  //!< Vertical Field Of View (rad)
  double vfov;
  
  std::string cameraName;
  
  std::string _frame_id;
  std::string _parent_frame_id;

  //!< The topic subscribed to for the front camera
  std::string imageTopic;
  
  //!< Variable used for State Managing
  bool landoltcNowON;
  
  //!< Publishers for LandoltcDetector result messages
  ros::Publisher _landoltcPublisher;
  
  //!< The dynamic reconfigure (landoltc) parameters' server
  dynamic_reconfigure::Server<pandora_vision_landoltc::landoltc_cfgConfig>
  server;
  
  dynamic_reconfigure::Server<pandora_vision_landoltc::landoltc_cfgConfig>::CallbackType f;
  
  ros::Time _lastTimeProcessed;
   
  /**
  @brief Callback for the RGB Image
  @param msg [const sensor_msgs::ImageConstPtr& msg] The RGB Image
  @return void
  **/
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
  @brief main function called for publishing messages in
    data fusion. In this function an object of class landoltcDetector
    is created to identify landoltCs in current frame.
  @param void
  @return void
  **/
  void landoltcCallback();
  
  /**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_landoltc::landoltc_cfgConfig&]
  @param[in] level [const uint32_t] The level 
  @return void
  **/
  void parametersCallback(
  const pandora_vision_landoltc::landoltc_cfgConfig& config,
  const uint32_t& level);
  
  /**
    @brief Function that retrieves the parent to the frame_id
    @return bool Returns true is frame_id found or false if not 
  */ 
  bool getParentFrameId();
  
  std::map<std::string, std::string> _frame_ids_map;
      
public:

  /**
  @brief Default Constructor
  @param ref [cv::Mat&] Reference Image
  @return void
  **/
  explicit LandoltCDetection(const std::string& ns);

  /**
  @brief Default Destructor
  @return void
  **/
  ~LandoltCDetection();

  /**
  @brief Get parameters referring to view and frame characteristics
  @return void
  **/
  void getGeneralParams();
  
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
    
  int curState;
  int prevState;
  std::string param;
};
} // namespace pandora_vision
#endif  // PANDORA_VISION_LANDOLTC_LANDOLTC_DETECTION_H
