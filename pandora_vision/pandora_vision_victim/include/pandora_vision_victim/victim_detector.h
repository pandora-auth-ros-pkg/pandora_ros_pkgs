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

#ifndef PANDORA_VISION_VICTIM_VICTIM_DETECTOR_H 
#define PANDORA_VISION_VICTIM_VICTIM_DETECTOR_H 

#include "pandora_vision_victim/face_detector.h"
#include "pandora_vision_victim/rgb_system_validator.h"
#include "pandora_vision_victim/depth_system_validator.h"
namespace pandora_vision
{
  
  enum FromHoleDetector
  { 
    GOT_NOTHING = 1,
    GOT_ALL = 4,
    GOT_MASK = 2,
    GOT_DEPTH = 3
  };
  
  struct EnhancedMat
  {
    cv::Mat img;
    cv::Rect bounding_box;
    cv::Point2f keypoint;
  };
  
  
  
  struct DetectionImages
  {
    EnhancedMat rgb;
    EnhancedMat depth;
    std::vector<EnhancedMat> rgbMasks;
    std::vector<EnhancedMat> depthMasks;
  };
  
  class VictimDetector
  {
    private:
      /// Instance of class face_detector
      FaceDetector* _faceDetector;
      ///Instance of class rgbSystemValidator
      RgbSystemValidator _rgbSystemValidator;
      
      ///Instance of class depthSystemValidator
      DepthSystemValidator _depthSystemValidator;
      
      /// Flag that indicates current state, according to the information
      /// received from hole_detector_node
      int _stateIndicator;
        
    public:
    
      DetectionImages dImages;
    
      FromHoleDetector detectionMode;
      
      //!< The Constructor
      explicit VictimDetector(std::string cascade_path, 
          std::string model_path, int bufferSize, std::string rgb_classifier_path,
          std::string depth_classifier_path);

      //!< The Destructor
      virtual ~VictimDetector();
      
      /**
       *@brief Function that enables suitable subsystems, according
       * to the current State 
       * @param [std::vector<cv::Mat>] vector of images to be processed. Size of
       * vector can be either 2 or 1, if we have both rgbd information or not
       * @return void
      */ 
      std::vector<DetectedVictim> victimFusion( DetectionImages imgs );
     
  }; 
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_VICTIM_DETECTOR_H
