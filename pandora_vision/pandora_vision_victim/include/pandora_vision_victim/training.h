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

#ifndef PANDORA_VISION_VICTIM_VICTIM_DETECTION_H
#define PANDORA_VISION_VICTIM_VICTIM_DETECTION_H

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include "pandora_vision_victim/channels_statistics_extractor.h"
#include "pandora_vision_victim/edge_orientation_extractor.h"
#include "pandora_vision_victim/haralickfeature_extractor.h"
#include "pandora_vision_victim/depth_system_validator.h"
#include "pandora_vision_victim/rgb_system_validator.h"

#define NUM_OF_POSITIVE_SAMPLES 3500

namespace pandora_vision
{
class SvmTraining
{
  private:
    
  //!< The NodeHandle
  ros::NodeHandle _nh;
  
  RgbSystemValidator _rgbSystem;
  DepthSystemValidator _depthSystem;
 
  //!< Variable used for State Managing
  bool trainingNowON;

  std::string package_path;
    
  int num_files;
  int num_feat;
  int rgb_num_feat;
  int depth_num_feat;
  cv::Mat training_mat;
  cv::Mat labels_mat;
  
  /// Set up SVM's parameters
  CvSVMParams params;
  
  /// Train the SVM
  CvSVM SVM;
public:

  //!< The Constructor
  explicit SvmTraining(int _num_files, int _num_feat);
  
  //!< The Constructor
  explicit SvmTraining(int _num_files, int _rgb_num_files, int _depth_num_files);
  
  //!< The Destructor
  virtual ~SvmTraining();
  
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
*@brief Function that loads the necessary files for the training
* @param [std::string] training_mat_file, name of the file that contains the training data
* @param [std::string] labels_mat_file, name of the file that contains the labels of each class
* of the training data
* @return void
*/
  void loadFiles(std::string training_mat_file_stream, std::string labels_mat_file_stream);

  
  std::string path_to_samples;
  
};
}// namespace pandora_vision
#endif // PANDORA_VISION_FACE_FACE_DETECTION_H
