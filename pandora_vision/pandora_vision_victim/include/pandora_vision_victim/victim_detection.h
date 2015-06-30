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

#include <map>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "pandora_vision_victim/victim_vj_detector.h"
// #include "pandora_vision_victim/victim_parameters.h"
#include "pandora_vision_victim/classifiers/rgb_svm_validator.h"
#include "pandora_vision_victim/classifiers/depth_svm_validator.h"


namespace pandora_vision
{
  class VictimDetection : public StateClient
  {
    private:
      /// The NodeHandle
      ros::NodeHandle _nh;

      /// FaceDetector frame timestamp
      ros::Time victimFrameTimestamp;

      /// The topic subscribed to for the camera
      std::string cameraFrameId;

      /// Publishers for FaceDetector result messages
      ros::Publisher _victimDirectionPublisher;

      /// The subscriber that listens to the frame
      /// topic advertised by the central node
      ros::Subscriber _frameSubscriber;

      /// Current state of robot
      int curState;
      /// Previous state of robot
      int prevState;

      std::vector<cv::Mat> _rgbdImages;

      /// Instance of class face_detector
      VictimVJDetector _rgbViolaJonesDetector;
      // VictimVJDetector _victimDetector;

      /// Instance of RGB SVM Validator
      boost::shared_ptr<RgbSvmValidator> rgbSvmValidator_;
      /// Instance of Depth SVM Validator
      boost::shared_ptr<DepthSvmValidator> depthSvmValidator_;

      /**
      @brief This method check in which state we are, according to
      the information sent from hole_detector_node
      @return void
      **/
      void detectVictims(
        bool depthEnabled,
        bool holesEnabled,
        const cv::Mat& rgbImage,
        const cv::Mat& depthImage,
        const pandora_vision_msgs::EnhancedHolesVectorMsg& msg);

      /**
      @brief This method check in which state we are, according to
      the information sent from hole_detector_node
      @return void
      **/
      void dummyDetectVictims(bool depthEnabled, const cv::Mat& rgbImage, const sensor_msgs::Image& msg);

      /**
      @brief Function called when new ROS message appears, for camera
      @param msg [const sensor_msgs::Image&] The message
      @return void
      **/
      void dummyimageCallback(const sensor_msgs::Image& msg);

      /**
      Function called when new message appears from hole_detector_node
      @param msg [pandora_vision_msgs::EnhancedHolesVectorMsg&] The message
      @return void
      **/
      void imageCallback(
        const pandora_vision_msgs::EnhancedHolesVectorMsg& msg);

      /**
      @brief Function that retrieves the parent to the frame_id
      @return bool Returns true is frame_id found or false if not
      **/
      bool getParentFrameId();

      std::map<std::string, std::string> _frame_ids_map;

      std::string _frame_id;
      std::string _parent_frame_id;

      VictimParameters params;

      // Debug purposes
      // The image_transport nodehandle
      image_transport::ImageTransport imageTransport_;
      image_transport::Publisher _debugVictimsPublisher;
      image_transport::Publisher _interpolatedDepthPublisher;
      cv::Mat debugImage;
      std::vector<cv::KeyPoint> rgb_vj_keypoints;
      std::vector<cv::KeyPoint> rgb_svm_keypoints;
      std::vector<cv::KeyPoint> depth_vj_keypoints;
      std::vector<cv::KeyPoint> depth_svm_keypoints;
      std::vector<cv::Rect> rgb_vj_bounding_boxes;
      std::vector<cv::Rect> rgb_svm_bounding_boxes;
      std::vector<cv::Rect> depth_vj_bounding_boxes;
      std::vector<cv::Rect> depth_svm_bounding_boxes;
      std::vector<cv::Rect> holes_bounding_boxes;
      std::vector<float> rgb_vj_p;
      std::vector<float> rgb_svm_p;
      std::vector<float> depth_vj_p;
      std::vector<float> depth_svm_p;

      DetectionImages dImages;

      /**
      @brief Function that enables suitable subsystems, according
      to the current State 
      @param [std::vector<cv::Mat>] vector of images to be processed. Size of
      vector can be either 2 or 1, if we have both rgbd information or not
      @return void
      **/
      std::vector<DetectedVictim> victimFusion(
        DetectionImages imgs,
        DetectionMode detectionMode);

    public:
      /// The Constructor
      explicit VictimDetection(const std::string& ns);

      /// The Destructor
      ~VictimDetection();

      /**
      @brief Node's state manager
      @param newState [int] The robot's new state
      @return void
      **/
      void startTransition(int newState);

      /**
      @brief After completion of state transition
      @return void
      **/
      void completeTransition(void);
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_DETECTION_H


