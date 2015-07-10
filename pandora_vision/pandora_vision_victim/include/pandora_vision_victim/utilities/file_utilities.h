/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors:
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *   Protopapas Marios <protopapas_marios@hotmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_VICTIM_UTILITIES_FILE_UTILITIES_H
#define PANDORA_VISION_VICTIM_UTILITIES_FILE_UTILITIES_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <boost/lambda/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
namespace pandora_vision_victim
{
/**
 * @namespace file_utilities
 * @brief The namespace for a set of file utilities functions.
 */
namespace file_utilities
{
  void saveFeaturesInFile(const cv::Mat& featuresMat,
                          const cv::Mat& labelsMat,
                          const std::string& prefix,
                          const std::string& featuresFileName,
                          const std::string& labelsFileName,
                          const std::string& imageType);

  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param var_name [std::string] : name of the variable to be saved to the file
  @param var [cv::Mat] : variable to be saved to the file
  @return void
  **/
  void saveToFile(const std::string& fileName,
                  const std::string& varName,
                  const cv::Mat& src);

  /**
   * @brief Function that saves a set of matrices in a file.
   * @param fileName [const std::string&] The name of the file to be created
   * @param varNameVec [const std::vector<std::string>&] The names of the
   * matrices to be saved.
   * @param dataVec [std::vector<cv::Mat>&] : The matrices to be saved.
   * @return void
   */
  void saveDataToFile(const std::string& fileName,
      const std::vector<std::string>& varNameVec,
      const std::vector<cv::Mat>& dataVec);

  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param training_mat [cv::Mat] : name of the mat of features to be saved to the file
  @param labels_mat [cv::Mat] : name of the mat of labels to be saved to the file
  @return void
  **/
  void saveToCSV(const std::string& fileName,
                 const cv::Mat& featuresMat,
                 const cv::Mat& labelsMat);

  /**
   * @brief Function that loads a set of descriptors to be used for training.
   * @param dataMatFile [const std::string&] The name of the file to read the
   * desriptors from.
   * @param nameTagVec [const std::vector<std::string>&] The names of the images
   * to load the descriptors from.
   * @param descriptorsVec [std::vector<cv::Mat>*] A set of image descriptors.
   * @return [bool] Variable indicating whether the loading was successful or
   * not.
   */
  bool loadDescriptorsFromFile(const std::string& dataMatFile,
      const std::vector<std::string>& nameTagVec,
      std::vector<cv::Mat>* descriptorsVec);

  /**
  @brief Function that loads the necessary files for the training
  @param [std::string] training_mat_file, name of the file that contains the training data
  @param [std::string] labels_mat_file, name of the file that contains the labels of each class
  of the training data
  @return void
  **/
  cv::Mat loadFiles(const std::string& dataMatFile,
                    const std::string& nameTag);

  /**
  @brief Function that checks if a file exists
  @param [const char*] name, Name of file to check if exists
  @return true if the file was found, and false if not
  **/
  bool exist(const char* fileName);

  /**
   * @brief
   */
  int countFilesInDirectory(const boost::filesystem::path& directory);

  /**
   * @brief
   */
  bool loadAnnotationsFromFile(const std::string& filename,
                               std::vector<cv::Rect>* boundingBox,
                               std::vector<std::string>* annotatedImages,
                               std::vector<int>* classAttributes);

  /**
   * @brief
   */
  int findNumberOfAnnotations(const std::string& filename);
}  // namespace file_utilities
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_UTILITIES_FILE_UTILITIES_H
