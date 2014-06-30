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
  std::vector<DetectedVictim> VictimDetector::victimFusion(DetectionImages imgs)
  {
    std::vector<DetectedVictim> final_probabilities;
    
    std::vector<DetectedVictim> rgb_vj_probabilities;
    std::vector<DetectedVictim> depth_vj_probabilities;
    std::vector<DetectedVictim> rgb_svm_probabilities;
    std::vector<DetectedVictim> depth_svm_probabilities;
    
    DetectedVictim temp;
    
    ///Enable Viola Jones for rgb image 
    rgb_vj_probabilities = _faceDetector->findFaces(imgs.rgb.img);
    
    if(detectionMode == GOT_ALL || detectionMode == GOT_DEPTH)
    {
      depth_vj_probabilities = _faceDetector->findFaces(imgs.depth.img);
      
    }
    if(detectionMode == GOT_ALL || detectionMode == GOT_MASK)
    {
      for(int i = 0 ; i < imgs.rgbMasks.size(); i++)
      {
        temp.probability = _rgbSystemValidator.calculateSvmRgbProbability(
            imgs.rgbMasks.at(i).img);
        temp.keypoint = imgs.rgbMasks[i].keypoint;
        rgb_svm_probabilities.push_back(temp);
      }  
    }
    if(detectionMode == GOT_ALL)
    {
      for(int i = 0 ; i < imgs.depthMasks.size(); i++)
      {
        temp.probability = _depthSystemValidator.calculateSvmDepthProbability(
            imgs.depthMasks.at(i).img);
        temp.keypoint = imgs.depthMasks[i].keypoint;
        depth_svm_probabilities.push_back(temp);
      }
    }
    
    // SVM mask merging
    if(detectionMode == GOT_ALL)
    {
      for(unsigned int i = 0 ; i < depth_svm_probabilities.size() ; i++)
      {
        temp.probability = (VictimParameters::depth_svm_weight * depth_svm_probabilities[i].probability + 
          VictimParameters::rgb_svm_weight * rgb_svm_probabilities[i].probability) / 
            (VictimParameters::depth_svm_weight + 
            VictimParameters::rgb_svm_weight);
        temp.keypoint = depth_svm_probabilities[i].keypoint;
        final_probabilities.push_back(temp);
      }
    }
    if(detectionMode == GOT_MASK)
    {
      for(unsigned int i = 0 ; i < depth_svm_probabilities.size() ; i++)
      {
        temp.probability = rgb_svm_probabilities[i].probability * VictimParameters::rgb_svm_weight;
        temp.keypoint = rgb_svm_probabilities[i].keypoint;
        final_probabilities.push_back(temp);
      }
    }
    
    // VJ mask merging (?)
    for(unsigned int i = 0 ; i < rgb_vj_probabilities.size() ; i++)
    {
      temp.probability = rgb_vj_probabilities[i].probability * VictimParameters::rgb_vj_weight;
      temp.keypoint = rgb_vj_probabilities[i].keypoint;
      final_probabilities.push_back(temp);
    }
    for(unsigned int i = 0 ; i < depth_vj_probabilities.size() ; i++)
    {
      temp.probability = depth_vj_probabilities[i].probability * VictimParameters::depth_vj_weight;
      temp.keypoint = depth_vj_probabilities[i].keypoint;
      final_probabilities.push_back(temp);
    }
    
    return final_probabilities;
  }

}// namespace pandora_vision

