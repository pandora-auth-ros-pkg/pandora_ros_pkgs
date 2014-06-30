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

#include "pandora_vision_victim/depth_system_validator.h"

namespace pandora_vision
{
  /**
   @brief Constructor
  */ 
  DepthSystemValidator::DepthSystemValidator(std::string depth_classifier_path)
  {
    _depth_classifier_path = depth_classifier_path;
    
    ///Load classifier path for rgb subsystem
    _depthSvm.load(depth_classifier_path.c_str());
    
    _params.svm_type = CvSVM::C_SVC;
    _params.kernel_type = CvSVM::RBF;
    _params.C = 312.5;
    _params.gamma = 0.50625;
    _params.term_crit = cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100000, 1e-6);
  }
  
  /**
    @brief Destructor
  */
  DepthSystemValidator::~DepthSystemValidator()
  {
    ROS_DEBUG("[victim_node] : Destroying DepthSystemValidator instance");
  }
  
  /**
   * @brief This function extract features according to the
   * predifined features for the depth image
   * @param inImage [cv::Mat] current depth frame to be processed
   * @return void
  */ 
  float DepthSystemValidator::calculateSvmDepthProbability(cv::Mat inImage)
  {
    ///Extract statistics oriented features for depth image
    _channelsStatisticsDetector.findDepthChannelsStatisticsFeatures(inImage);
    ///Extract edge orientation features for depth image
    _edgeOrientationDetector.findEdgeFeatures(inImage);
     
    ///Extract haralick features for depth image 
    _haralickFeatureDetector.findHaralickFeatures(inImage);
    
    if(!_depthFeatureVector.empty())
      _depthFeatureVector.clear();
    
    setDepthFeatureVector();
    
    return predictionToProbability(predict());
  }
  
  /**
    * @brief This function creates feature vector according to the
    * predifined features for the depth image
    * @return void
  */ 
  void DepthSystemValidator::setDepthFeatureVector()
  {
    ///Append to rgbFeatureVector features according to color
    ///histogramms and other statistics
    std::vector<double> channelsStatictisFeatureVector = 
        _channelsStatisticsDetector.getDepthFeatures();
    for(int i = 0; i < channelsStatictisFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(channelsStatictisFeatureVector[i]);
    
    ///Append to depthFeatureVector features according to edge orientation
    std::vector<double> edgeOrientationFeatureVector = 
        _edgeOrientationDetector.getFeatures();
    for(int i = 0; i < edgeOrientationFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(edgeOrientationFeatureVector[i]);   
    
    ///Append to depthFeatureVector features according to haaralick features
    std::vector<double> haaralickFeatureVector = 
        _haralickFeatureDetector.getFeatures();
    for(int i = 0; i < haaralickFeatureVector.size(); i++ )
          _depthFeatureVector.push_back(haaralickFeatureVector[i]);  
          
    ///Deallocate memory
    channelsStatictisFeatureVector.clear();
    _channelsStatisticsDetector.emptyCurrentDepthFrameFeatureVector();
    
    edgeOrientationFeatureVector.clear();
    _edgeOrientationDetector.emptyCurrentFrameFeatureVector(); 
    
    haaralickFeatureVector.clear();
    _haralickFeatureDetector.emptyCurrentFrameFeatureVector();   
  }
  
  /**
   * @brief This function returns current feature vector according
   * to the features found in rgb image
   * @return [std::vector<double>] _rgbFeatureVector, feature vector 
   * for current rgb image
   */ 
  std::vector<double> DepthSystemValidator::getDepthFeatureVector()
  {
    return _depthFeatureVector;
  }
  
    /**
    * @brief Function that loads the trained classifier and makes a prediction
    * according to the featurevector given for each image
    * @return void
  */ 
  float DepthSystemValidator::predict()
  {
    cv::Mat samples_mat = vectorToMat(_depthFeatureVector);
    
    ///Normalize the data from [-1,1]
    cv::normalize(samples_mat, samples_mat, -1.0, 1.0, cv::NORM_MINMAX, -1);    
    float prediction = _depthSvm.predict(samples_mat, true);
    ROS_INFO_STREAM("Depth_subsystem prediction: "<< prediction);
    return prediction;
  }
  
  /**
   * @brief Function that converts a given vector of doubles
   * in cv:Mat in order to use it to opencv function predict()
   * @param [std::vector <double>] data, input vector to be 
   * converted
   * @return [cv::Mat] output Mat of size size_of_vectorx1
  */ 
  cv::Mat DepthSystemValidator::vectorToMat(std::vector<double> data)
  {
    int size = data.size();
    cv::Mat mat(size, 1, CV_32F);
    for(int i = 0; i < size; ++i)
    {
        mat.at<float>(i, 0) = data[i];
    }
    return mat;
  }
  
  /**
    * @brief This function prediction according to the rgb classifier
    * @return [float] prediction
  */ 
  float DepthSystemValidator::predictionToProbability(float prediction)
  {
    float probability;
    //~ Normalize probability to [-1,1]
    probability = tanh(0.5 * (prediction - 7.0) );
    //~ Normalize probability to [0,1]
    probability = (1 + probability) / 2.0;
    return probability;
  }
}// namespace pandora_vision 
