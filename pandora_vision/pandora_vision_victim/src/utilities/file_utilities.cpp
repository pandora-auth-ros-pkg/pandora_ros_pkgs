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

#include <vector>
#include <string>

#include "pandora_vision_victim/utilities/file_utilities.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
/**
 * @namespace file_utilities
 * @brief The namespace for a set of file utilities functions.
 */
namespace file_utilities
{
  std::string packagePath = ros::package::getPath("pandora_vision_victim");

  void saveFeaturesInFile(const cv::Mat& featuresMat,
                          const cv::Mat& labelsMat,
                          const std::string& prefix,
                          const std::string& featuresFileName,
                          const std::string& labelsFileName,
                          const std::string& imageType)
  {
    std::string filesDirectory = packagePath + "/data/";

    std::string featuresMatFilePath = filesDirectory + featuresFileName;
    std::string varName = prefix + "features_mat";
    saveToFile(featuresMatFilePath, varName, featuresMat);

    std::cout << featuresMatFilePath << std::endl;
    std::cout << "Size = " << featuresMat.size() << std::endl;

    std::string labelsMatFilePath = filesDirectory + labelsFileName;
    varName = prefix + "labels_mat";
    saveToFile(labelsMatFilePath, varName, labelsMat);

    std::cout << labelsMatFilePath << std::endl;
    std::cout << "Size = " << labelsMat.size() << std::endl;

    std::string featuresMatrixCsvFile = prefix + imageType + "matrix.csv";
    std::string featuresMatCsvFilePath = filesDirectory + featuresMatrixCsvFile;
    saveToCSV(featuresMatCsvFilePath, featuresMat, labelsMat);
  }

  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param var_name [std::string] : name of the variable to be saved to the file
  @param var [cv::Mat] : variable to be saved to the file
  @return void
  **/
  void saveToFile(const std::string& fileName,
                  const std::string& varName,
                  const cv::Mat& src)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);

    if (!fs.isOpened())
    {
      fs.open(fileName, cv::FileStorage::WRITE);
    }
    fs << varName << src;
    fs.release();
  }

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
      const std::vector<cv::Mat>& dataVec)
  {
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);

    if (!fs.isOpened())
    {
      fs.open(fileName, cv::FileStorage::WRITE);
    }
    if (dataVec.size() != varNameVec.size())
    {
      std::cout << "ERROR: Name vector and data vector have different size"
                << std::endl;
      return;
    }
    for (int ii = 0; ii < dataVec.size(); ii++)
      fs << varNameVec[ii] << dataVec[ii];

    fs.release();
  }

  /**
  @brief Function that saves a variable to a file
  @param file_name [std::string] : name of the file to be created
  @param training_mat [cv::Mat] : name of the mat of features to be saved to the file
  @param labels_mat [cv::Mat] : name of the mat of labels to be saved to the file
  @return void
  **/
  void saveToCSV(const std::string& fileName,
                 const cv::Mat& featuresMat,
                 const cv::Mat& labelsMat)
  {
    std::ofstream outFile;
    outFile.open(fileName.c_str(), std::ofstream::out | std::ofstream::trunc);
    if (!outFile)
    {
      std::cout << "Error! Cannot load CSV file!" << std::endl;
      return;
    }
    else
    {
      if (outFile.is_open())
      {
        for (int kk = 0; kk <= featuresMat.cols; kk++)
        {
          outFile << "attr" << kk;
          if (kk < featuresMat.cols)
             outFile << ",";
          else
            outFile << std::endl;
        }
        for (int ii = 0; ii < featuresMat.rows; ii++)
        {
          outFile << labelsMat.at<float>(ii) << ",";
          for (int jj = 0; jj < featuresMat.cols; jj++)
          {
            outFile << featuresMat.at<float>(ii, jj);
            if (jj < featuresMat.cols - 1)
              outFile << ",";
            else
              outFile << std::endl;
          }
        }
      }
      outFile.close();
    }
  }

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
      std::vector<cv::Mat>* descriptorsVec)
  {
    cv::FileStorage fs(dataMatFile, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
      fs.open(dataMatFile, cv::FileStorage::READ);
    }
    for (int ii = 0; ii < nameTagVec.size(); ii++)
    {
      cv::Mat tempDataMat;
      fs[nameTagVec[ii]] >> tempDataMat;
      if (!tempDataMat.data)
      {
        std::cout << "ERROR: Matrix has no data" << std::endl;
        fs.release();
        descriptorsVec->clear();
        return false;
      }
      descriptorsVec->push_back(tempDataMat);
    }

    fs.release();
    return true;
  }

  /**
  @brief Function that loads the necessary files for the training
  @param [std::string] training_mat_file, name of the file that contains the training data
  @param [std::string] labels_mat_file, name of the file that contains the labels of each class
  of the training data
  @return void
  **/
  cv::Mat loadFiles(const std::string& dataMatFile,
                    const std::string& nameTag)
  {
    cv::FileStorage fs1;
    cv::Mat dataMat;
    fs1.open(dataMatFile, cv::FileStorage::READ);
    fs1[nameTag] >> dataMat;
    fs1.release();

    if (dataMat.data)
    {
      std::cout << dataMatFile << " was loaded successfully." << std::endl;
      std::cout << "Size = " << dataMat.size() << std::endl;
      dataMat.convertTo(dataMat, CV_32FC1);
    }
    else
    {
      std::cout << dataMatFile << " was not loaded successfully."
                << std::endl;
    }
    return dataMat;
  }

  /**
  @brief Function that checks if a file exists
  @param [const char*] name, Name of file to check if exists
  @return true if the file was found, and false if not
  **/
  bool exist(const char* fileName)
  {
    std::ifstream file(fileName);
    if (!file)  // if the file was not found, then file is 0, i.e. !file=1 or true
      return false;  // the file was not found
    else  // if the file was found, then file is non-0
      return true;  // the file was found
  }

  /**
   * @brief
   */
  int countFilesInDirectory(const boost::filesystem::path& directory)
  {
    int numFiles = std::count_if(
        boost::filesystem::recursive_directory_iterator(directory),
        boost::filesystem::recursive_directory_iterator(),
        boost::lambda::bind(static_cast<bool(*)(const boost::filesystem::path&)>
          (boost::filesystem::is_regular_file),
            boost::lambda::bind(&boost::filesystem::directory_entry::path,
              boost::lambda::_1)));
    return numFiles;
  }

  /**
   * @brief
   */
  bool loadAnnotationsFromFile(const std::string& filename,
                               std::vector<cv::Rect>* boundingBox,
                               std::vector<std::string>* annotatedImages,
                               std::vector<int>* classAttributes)
  {
    std::ifstream inFile;
    std::string line, imgName, temp, x1, y1, x2, y2;
    cv::Rect rect;
    int category;
    int ii = 0;

    inFile.open(filename.c_str());
    if (!inFile)
    {
      ROS_ERROR("Cannot load Annotations file");
      return false;
    }
    else
    {
      if (inFile.is_open())
      {
        while (std::getline(inFile, line))
        {
          std::stringstream ss(line);
          getline(ss, imgName, ',');
          getline(ss, temp, ',');
          getline(ss, x1, ',');
          getline(ss, y1, ',');
          getline(ss, x2, ',');
          getline(ss, y2);

          rect.x = atoi(x1.c_str());
          rect.y = atoi(y1.c_str());
          rect.width = atoi(x2.c_str()) - atoi(x1.c_str());
          rect.height = atoi(y2.c_str()) - atoi(y1.c_str());
          category = atoi(temp.c_str());
          ROS_INFO_STREAM("Loading Annotation no: " << ii + 1 << " for "
                          << imgName);
          /*/
          ROS_INFO_STREAM(rect.x << "," <<
                          rect.y << "," <<
                          rect.width << "," <<
                          rect.height);
          // */
          annotatedImages->push_back(imgName);
          boundingBox->push_back(rect);
          classAttributes->push_back(category);
          ii++;
        }
      }
    }
    inFile.close();
    return true;
  }
}  // namespace file_utilities
}  // namespace pandora_vision
