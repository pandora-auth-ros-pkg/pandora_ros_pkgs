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

#include "pandora_vision_victim/victim_detector.h"

namespace pandora_vision
{
  /**
   @brief Constructor
  */ 
  VictimDetector::VictimDetector(std::string cascade_path, 
    std::string model_path, int bufferSize, std::string rgb_classifier_path,
    std::string depth_classifier_path):_rgbSystemValidator(rgb_classifier_path),
    _depthSystemValidator(depth_classifier_path) 
  {
    /// Initialize face detector
    _faceDetector = new FaceDetector(cascade_path, model_path, bufferSize);
    
    ROS_DEBUG("[victim_node] : VictimDetector instance created");
  }
  
  /**
    @brief Destructor
  */
  VictimDetector::~VictimDetector()
  {
    delete _faceDetector;
    ROS_DEBUG("[victim_node] : VictimDetector RgbSystemValidator instance");
  }
  
  /**
   *@brief Function that enables suitable subsystems, according
   * to the current State 
   * @param [std::vector<cv::Mat>] vector of images to be processed. Size of
   * vector can be either 2 or 1, if we have both rgbd information or not
   * @return void
  */ 
  int VictimDetector::victimFusion(DetectionImages imgs)
  {
    int faceNum = 0;
    ///Enable Viola Jones for rgb image
    faceNum = _faceDetector->findFaces(imgs.rgb);
    if(detectionMode == GOT_ALL || detectionMode == GOT_DEPTH)
    {
      _faceDetector->findFaces(imgs.depth);
    }
    if(detectionMode == GOT_ALL || detectionMode == GOT_MASK)
    {
      for(int i = 0 ; i < imgs.rgbMasks.size(); i++){
        //~ cv::imshow("rgb mask",imgs.rgbMasks.at(i));
        //~ cv::waitKey(30);
        rgbFeaturesDetect(imgs.rgbMasks.at(i));
      }  
    }
    if(detectionMode == GOT_ALL)
    {
      for(int i = 0 ; i < imgs.depthMasks.size(); i++){
        depthFeaturesDetect(imgs.depthMasks.at(i));
      }
    }
    return faceNum;
  }
  
    
  /**
   *@brief Function that extracts handles rgb subsystem
   *@param [cv::Mat] current frame to be processed
   *@return void
  */ 
  void VictimDetector::rgbFeaturesDetect(cv::Mat _rgbImage)
  {
    _rgbSystemValidator.extractRgbFeatures(_rgbImage); 
  }
  
  /**
   *@brief Function that extracts handles depth subsystem
   *@param [cv::Mat] current frame to be processed
   *@return void
  */ 
  void VictimDetector::depthFeaturesDetect(cv::Mat _depthImage)
  {
    _depthSystemValidator.extractDepthFeatures(_depthImage); 
  }
  
}// namespace pandora_vision

