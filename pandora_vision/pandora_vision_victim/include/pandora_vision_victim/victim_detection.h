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
* Author: Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_VICTIM_DETECTION_H 
#define PANDORA_VISION_VICTIM_VICTIM_DETECTION_H 

#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "vision_communications/EnhancedHolesVectorMsg.h"
#include "vision_communications/EnhancedHoleMsg.h"
#include "state_manager/state_client.h"
#include <urdf_parser/urdf_parser.h>
#include <map>
#include "pandora_vision_victim/victim_detector.h"

namespace pandora_vision
{
class VictimDetection : public StateClient
{
private:

  /// The NodeHandle
  ros::NodeHandle _nh;

  /// Horizontal field of view in rad
  double hfov;

  /// Vertical Field Of View (rad)
  double vfov;

  int frameWidth;
  int frameHeight;

  std::string cameraName;
  std::string packagePath;
  
  /// Rgb Frame processed by FaceDetector
  cv::Mat _rgbImage;
  /// Depth Frame processed by FaceDetector
  cv::Mat _depthImage;

  /// FaceDetector frame timestamp
  ros::Time victimFrameTimestamp;

  /// The topic subscribed to for the camera
  std::string _enhancedHolesTopic;
  std::string cameraFrameId;
   
  /// Publishers for FaceDetector result messages
  ros::Publisher _victimDirectionPublisher;

  /// The subscriber that listens to the frame
  /// topic advertised by the central node
  ros::Subscriber _frameSubscriber;

  /// Variable used for State Managing
  bool victimNowON;

  /// Current state of robot
  int curState;
  /// Previous state of robot
  int prevState;
    
  /// Parameters for the FaceDetector instance
  std::string cascade_path;
  std::string model_path;
  std::string model_url;
  std::string rgb_classifier_path;
  std::string depth_classifier_path;
  
  int bufferSize;
  
  /// Flag that indicates if we have depth information
  /// from kinect sensor
  bool isDepthEnabled;
  /// Flag that indicates if there is one or more holes in current
  /// frame in order to enable a suitable mask
  bool isHole;
  
  ///Vector of holes found in current frame
  //~ std::vector<vision_communications::EnhancedHoleMsg> _enhancedHoles;
  vision_communications::EnhancedHolesVectorMsg _enhancedHoles;
  
  /// Flag that indicates current state, according to the information
  /// received from hole_detector_node
  int _stateIndicator;
  
  /// Instance of class VictimDetector
  VictimDetector* _victimDetector;
  
  std::vector<cv::Mat> _rgbdImages;
  /**
   * @brief Get parameters referring to view and frame characteristics from
   * launch file
   * @return void
  */
  void getGeneralParams();

  /**
    *@brief Get parameters referring to the face detection algorithm
    *@return void
  **/
  void getVictimDetectorParameters();

  /**
   * @brief This method uses a FaceDetector instance to detect all
   * present faces in a given frame
   * @return void
  */
  void victimDetect(DetectionImages imgs);
  
  
  /**
   * @brief This method check in which state we are, according to
   * the information sent from hole_detector_node
   * @return void
  */
  void checkState();

  /**
   * Function called when new message appears from hole_detector_node
   * @param msg [vision_communications::EnhancedHolesVectorMsg&] The message
   * @return void
  */
  void imageCallback(const vision_communications::EnhancedHolesVectorMsg& msg);

  /**
    @brief Function that retrieves the parent to the frame_id
    @return bool Returns true is frame_id found or false if not 
  */ 
  bool getParentFrameId();
  
  std::map<std::string, std::string> _frame_ids_map;
  
  std::string _frame_id;
  std::string _parent_frame_id;
  
  VictimParameters params;
   
public:

  //!< The Constructor
  explicit VictimDetection(const std::string& ns);

  //!< The Destructor
  virtual ~VictimDetection();

  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
  */
  void startTransition(int newState);

  /**
   * @brief After completion of state transition
   * @return void
  */
  void completeTransition(void);
  
  std::string param;
};
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_DETECTION_H


