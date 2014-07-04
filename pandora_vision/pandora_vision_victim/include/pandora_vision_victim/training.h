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
* Author: Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_VICTIM_TRAINING_H
#define PANDORA_VISION_VICTIM_TRAINING_H

#include <iostream>
#include <fstream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include "pandora_vision_victim/utilities/channels_statistics_extractor.h"
#include "pandora_vision_victim/utilities/edge_orientation_extractor.h"
#include "pandora_vision_victim/utilities/haralickfeature_extractor.h"
#include "pandora_vision_victim/depth_system_validator.h"
#include "pandora_vision_victim/rgb_system_validator.h"
#include "pandora_vision_victim/training_parameters.h"

#define USE_OPENCV_GRID_SEARCH_AUTOTRAIN 1


namespace pandora_vision
{
class SvmTraining
{
  private:
    
  //!< The NodeHandle
  ros::NodeHandle _nh;
  
  ///Feature vector for rgb features
  std::vector<double> _rgbFeatureVector;
  
  ///Feature vector for depth features
  std::vector<double> _depthFeatureVector;
  
  ///Instance of class ChannelsStatisticsExtractor
  ///to detect color features for the given frame
  ChannelsStatisticsExtractor _channelsStatisticsDetector;
  
  ///Instance of class EdgeOrientationExtractor
  ///to detect edge orientation features for the given frame
  EdgeOrientationExtractor _edgeOrientationDetector;
  
  ///Instance of class HaralickFeatureExtractor
  ///to detect haralick features for the given frame
  HaralickFeaturesExtractor _haralickFeatureDetector;
 
  //!< Variable used for State Managing
  bool trainingNowON;

  std::string package_path;
    
  int num_files;
  int test_num_files;
  int num_feat;
  float accuracy;
  float precision;
  float recall;
  float fmeasure;
  cv::Mat training_mat;
  cv::Mat labels_mat;
  cv::Mat test_mat;
  cv::Mat test_labels_mat;
  
  /// Set up SVM's parameters
  CvSVMParams params;
  CvParamGrid CvParamGrid_gamma, CvParamGrid_C;
  
  /// Train the SVM
  CvSVM SVM;
public:

  //!< The Constructor
  explicit SvmTraining(int _num_files, int _test_num_files, int _num_feat);
  
  //!< The Destructor
  virtual ~SvmTraining();
  
  /**
* @brief This function extract features according to the
* predifined features for the rgb image
* @param inImage [cv::Mat] current rgb frame to be processed
* @return void
*/
  void extractRgbFeatures(const cv::Mat& inImage);
    
  /**
* @brief This function creates feature vector according to the
* predifined features for the rgb image
* @return void
*/
  void setRgbFeatureVector();
  
  /**
* @brief This function returns current feature vector according
* to the features found in rgb image
* @return [std::vector<double>] _rgbFeatureVector, feature vector
* for current rgb image
*/
  std::vector<double> getRgbFeatureVector();
  
  /**
* @brief This method constructs the training matrix
* to be used for the training
* @param [std::string] file name to save the extracted training matrix
* @param [int] type, Value that indicates, if we train depth subsystem,
* or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
* @return void
*/
  void constructTrainingMatrix(std::string file_name, int type);
  
      
  /**
* @brief This function extract features according to the
* predifined features for the depth image
* @param inImage [cv::Mat] current depth frame to be processed
* @return void
*/
  void extractDepthFeatures(cv::Mat inImage);
  
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
* @brief This method constructs the rgb test matrix
* to be used for validation of the training
* @param [std::string] file name to save the extracted test matrix
* @param [int] type, Value that indicates, if we train depth subsystem,
* or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
* @return void
*/
  void constructTestMatrix(std::string file_name, int type);
  
  /**
*@brief Function that implements the training for the subsystems
* according to the given training sets. It applies svm and extracts
* a suitable model
* @param [int] type, Value that indicates, if we train depth subsystem,
* or rgb subsystem. Default value is 1, that corresponds to rgb subsystem
* @return void
*/
  void trainSubSystem(int type);
  
  /**
*@brief Function that checks if a file exists
*@param name Name of file to check if exists
*@return true if the file was found, and false if not
*/
  bool exist(const char *name);
  
  /**
*@brief Function that saves a variable to a file
* @param [std::string] file_name, name of the file to be created
* @param [std::string] var_name, name of the variable to be saved to the file
* @param [cv::Mat] var, variable to be saved to the file
* @return void
*/
  void saveToFile(std::string file_name, std::string var_name, cv::Mat var);
  
   /**
*@brief Function that loads the necessary files for the training
* @param [std::string] training_mat_file, name of the file that contains the training data
* @param [std::string] labels_mat_file, name of the file that contains the labels of each class
* of the training data
* @return void
*/
  void loadFiles(std::string training_mat_file_stream, std::string labels_mat_file_stream,
                  std::string test_mat_file, std::string test_labels_mat_file);
                  
  /**
*@brief Function that evaluates the training
*@param [cv::Mat&] predicted the predicted results
*@param [cv::Mat&] the actual results
*@return void
*/
  void evaluate(const cv::Mat& predicted, const cv::Mat& actual);
  
  /**
*@brief Function that computes the min distance between the features
* of the 2 classes
*@return void
*/
  void calcMinDistance();
  
  /**
*@brief Function that computes the vectors A,B necessary for the computation
* of the probablistic output of the svm bases on platt's binary svm
* probablistic Output
* @param [cv::Mat] dec_values, the distance from the hyperplane of the
* predicted results of the given test dataset
* @param [cv::Mat] labels, the true labels of the dataset
* @param [double&] A, the vector A to be computed
* @param [double&] B, the vector B to be computed
* @return void
*/
  void sigmoid_train(cv::Mat dec_values, cv::Mat labels, double* A, double* B);

  
  std::string path_to_samples;
  
};
}// namespace pandora_vision
#endif // PANDORA_VISION_VICTIM_TRAINING_H
