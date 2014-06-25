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
 * Author: Miltiadis-Alexios Papadopoulos
 *********************************************************************/

#ifndef PANDORA_VISION_QRCODE_QRCODE_DETECTION_H
#define PANDORA_VISION_QRCODE_QRCODE_DETECTION_H

#include <iostream>
#include <stdlib.h>
#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "state_manager/state_client.h"
#include <std_srvs/Empty.h>
#include "vision_communications/QRAlertsVectorMsg.h"
#include "pandora_vision_qrcode/qrCode_detector.h"
#include <urdf_parser/urdf_parser.h>
#include <map>

namespace pandora_vision {

  class QrCodeDetection : public StateClient {

    private:

      //!< The NodeHandle
      ros::NodeHandle _nh;

      QrCodeDetector _qrcodeDetector;

      //!< Horizontal Field Of View (rad)
      std::vector<double> _hfov;

      //!< Vertical Field Of View (rad)
      std::vector<double> _vfov;

      std::vector<int> _frameWidth;
      std::vector<int> _frameHeight;
      
      double hfov;
      double vfov;
      int frameWidth;
      int frameHeight;
      
      std::string cameraName;
      
      std::string imageTopic;
      
      std::string _parent_frame_id; 
      std::string _frame_id;
      
      //!< Frame processed by QrCodeDetector
      cv::Mat qrcodeFrame;

      //!< MotionDetector frame timestamp
      ros::Time qrcodeFrameTimestamp;

      //!< The topics subscribed to all cameras
      std::vector<std::string> _imageTopics;
    
      //!< The frame ids subscribed to all cameras
      std::vector<std::string> cameraFrameIds;
      
      //!< Publishers for QrCodeDetector result messages
      ros::Publisher _qrcodePublisher;

      //!< The subscribers that listens to the frame topic advertised by the
      //!< central node for all cameras
      std::vector<ros::Subscriber> _frameSubscribers;
      
      ros::Subscriber frameSubscriber;

      //!< Debug publisher for MotionDetector
      image_transport::Publisher _qrcodeDebugPublisher;

      //!< Variables for changing in dummy msg mode for debugging
      bool qrcodeDummy;

      //!< Variables for changing in debug mode. Publish images for debugging
      bool debugQrCode;

      //!< Variable used for State Managing
      bool qrcodeNowON;

      /**
       * @brief Get parameters referring to view and frame characteristics from
       * launch file
       * @return void
       */
      void getGeneralParams();

      /**
       * @brief Get parameters referring to Qrcode detection algorithm
       * @return void
       */
      void getQrCodeParams();

      /**
       * @brief Publishing debug images
       * @return void
       */
      void publish_debug_images();

      /**
       * @brief This method uses a QrCodeDetector instance to detect all present
       * qrcodes in a given frame
       * @return void
       */
      void qrDetect();

      /**
       * @brief Function called when new ROS message appears, for front camera
       * @param msg [const sensor_msgs::Image&] The message
       * @return void
       */
      void imageCallback(const sensor_msgs::Image& msg);
      
      /**
       *@brief Function that retrieves the parent to the frame_id
       *@return bool Returns true if frame_id found or false if not 
      */ 
      bool getParentFrameId();
       
      //!< Current state of robot
      int curState;

      //!< Previous state of robot
      int prevState;
      
      std::string param;
      
      std::map<std::string, std::string> _frame_ids_map;
      
      int _camera_indicator;
    public:

      //!< The Constructor
      explicit QrCodeDetection(const std::string& ns);

      //!< The Destructor
      ~QrCodeDetection();

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

  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_QRCODE_QRCODE_DETECTION_H
