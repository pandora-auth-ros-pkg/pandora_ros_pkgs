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

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "state_manager/state_client.h"
#include "vision_communications/DataMatrixAlertsVectorMsg.h"
#include "pandora_vision_datamatrix/datamatrix_detector.h"

#ifndef PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTION_H 
#define PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTION_H 

//!< Default frame height
#define DEFAULT_HEIGHT 480

//!< Default frame width
#define DEFAULT_WIDTH 640

//!< Default horizontal field of view in degrees
#define HFOV 61.14

//!< Default vertical field of view in degrees
#define VFOV 48

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
**/
namespace pandora_vision
{
  /**
    @class DatamatrixDetection
    @brief Class for publishing messages in case that,
    a damatrix is found. In this class i create an object of class
    DatamatrixDetector, which is responsible for identifying
    and decoding damatrixes.
   **/
  class DatamatrixDetection : public StateClient 
  {
    private:
    
    //!< The NodeHandle
    ros::NodeHandle _nh;
    
    //!< pandora_vision_datamatrix package path
    std::string packagePath;
    
    float ratioX;
    float ratioY;

    //!< Horizontal field of view in rad
    double hfov;

    //!< Vertical Field Of View (rad)
    double vfov;
    int frameWidth;
    int frameHeight;
    
    DatamatrixDetector _datamatrixDetector;
    
    //!< Frame processed by MotionDetector
    cv::Mat datamatrixFrame;

    //!< MotionDetector frame timestamp
    ros::Time datamatrixFrameTimestamp;  
   
    //!< The topic subscribed to for the front camera
    std::string imageTopic;
    
    std::string cameraName;
    std::string cameraFrameId;
    
    //!< Publishers for QrCodeDetector result messages
    ros::Publisher _datamatrixCodePublisher;

    //!< The subscriber that listens to the frame topic advertised 
    //!< by the central node for the front camera
    ros::Subscriber _frameSubscriber;
    
    //!< Variable used for State Managing
    bool datamatrixNowON;
    
    /**
     * @brief Get parameters referring to view and frame 
     * characteristics from launch file
     * @return void
    */
    void getGeneralParams();
    
    /**
     * @brief This method uses a DatamatrixDetector instance to detect 
     * all present datamatrixes in a given frame
     * @return void
    */
    void datamatrixCallback();
          
     /**
      @brief Callback for the RGB Image
      @param msg [const sensor_msgs::Image& msg] The RGB Image
      @return void
    **/
    void imageCallback(const sensor_msgs::Image& msg);
     
    //!< Current state of robot
    int curState;

    //!< Previous state of robot
    int prevState;
        
    public:
    
    /**
      @brief Default Constructor
      @return void
    **/
    DatamatrixDetection();

    /**
      @brief Default Destructor
      @return void
    **/
    virtual ~DatamatrixDetection();
   
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

  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_DATAMATRIX_DATAMATRIX_DETECTION_H
