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
#ifndef PANDORA_VISION_VICTIM_DEPTH_SYSTEM_VALIDATOR_H 
#define PANDORA_VISION_VICTIM_DEPTH_SYSTEM_VALIDATOR_H 

#include "ros/ros.h"
#include "pandora_vision_victim/edge_orientation_extractor.h"
#include "pandora_vision_victim/channels_statistics_extractor.h"
#include "pandora_vision_victim/haralickfeature_extractor.h"

namespace pandora_vision
{
  class DepthSystemValidator
  {
    ///Feature vector for depth features
    std::vector<double> _depthFeatureVector;
    
    ///Instance of class  ChannelsStatisticsExtractor 
    ///to detect color features for the given frame
    ChannelsStatisticsExtractor _channelsStatisticsDetector;
    ///Instance of class  EdgeOrientationExtractor 
    ///to detect edge orientation features for the given frame
    EdgeOrientationExtractor _edgeOrientationDetector;
    ///Instance of class  HaralickFeatureExtractor 
    ///to detect haralick features for the given frame
    HaralickFeaturesExtractor _haralickFeatureDetector;
    
    std::string _depth_classifier_path;
    
     /// Svm classifier used for rgb subsystem
    CvSVM _depthSvm;
    
    /// Set up SVM's parameters
    CvSVMParams _params;
         
    public:
    
    DepthSystemValidator();
    
    ///Constructor
    explicit DepthSystemValidator(std::string depth_classifier_path);
    
    ///Destructor
    ~DepthSystemValidator();
    
    /**
     * @brief This function extract features according to the
     * predifined features for the depth image
     * @param inImage [cv::Mat] current depth frame to be processed
     * @return void
     */ 
    float calculateSvmDepthProbability(cv::Mat inImage);
    
    /**
     * @brief This function creates feature vector according to the
     * predifined features for the depth image
     * @return void
     */ 
    void setDepthFeatureVector();
    
    /**
     * @brief This function returns current feature vector according
     * to the features found in rgb image
     * @return [std::vector<double>] _rgbFeatureVector, feature vector 
     * for current rgb image
     */ 
    std::vector<double> getDepthFeatureVector();
    
    /**
     * @brief Function that loads the trained classifier and makes a prediction
     * according to the featurevector given for each image
     * @return void
    */ 
    float predict();
    
    /**
     * @brief Function that converts a given vector of doubles
     * in cv:Mat in order to use it to opencv function predict()
     * @param [std::vector <double>] data, input vector to be 
     * converted
     * @return [cv::Mat] output Mat of size size_of_vectorx1
    */ 
    cv::Mat vectorToMat(std::vector<double> data);
    
    /**
     * @brief This function prediction according to the rgb classifier
     * @return [float] prediction
     */ 
    float predictionToProbability(float prediction);
    
  };
}// namespace pandora_vision 
#endif  // PANDORA_VISION_VICTIM_DEPTH_SYSTEM_VALIDATOR_H
