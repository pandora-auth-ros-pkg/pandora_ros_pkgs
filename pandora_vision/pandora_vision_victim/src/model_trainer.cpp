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

#include <opencv2/core/core.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include <ros/package.h>

static void read_csv(const std::string& filename,
    const std::vector<cv::Mat>& images, const std::vector<int>& labels,
    char separator = ';')
{
  boost::filesystem::path filePath(filename);
  std::ifstream file(filename.c_str(), std::ifstream::in);
  if (!file)
  {
    std::string error_message = "No valid input file was given, please check the given filename.";
    CV_Error(CV_StsBadArg, error_message);
  }
  std::string line, url, path, classlabel;
  while (getline(file, line))
  {
    std::stringstream liness(line);
    std::getline(liness, url, separator);
    std::getline(liness, path, separator);
    std::getline(liness, classlabel);

    boost::filesystem::path linePath(path);
    boost::filesystem::path abs;

    abs = boost::filesystem::absolute(linePath, filePath.parent_path());

    if (!boost::filesystem::exists(path))
    {
      ROS_WARN("File not found, downloading now...");
      std::string cmd = "wget " + url + " --no-check-certificate -nH --cut-dirs=3 --directory-prefix=" +
                        abs.parent_path().string();
      system(cmd.c_str());
    }

    if (!path.empty() && !classlabel.empty())
    {
      images.push_back(cv::imread(abs.string(), 0));
      labels.push_back(atoi(classlabel.c_str()));
    }
  }
}

int main()
{
  // These vectors hold the images and corresponding labels.
  std::vector<cv::Mat> images;
  std::vector<int> labels;
  // Read in the data. This can fail if no valid
  // input filename is given.

  std::string path_to_images = ros::package::getPath("pandora_vision_victim") + "att_faces";

  std::string fn_csv = ros::package::getPath("pandora_vision_victim") + "/data/csv.ext";
  try
  {
    read_csv(fn_csv, images, labels);
  }
  catch (cv::Exception& e)
  {
    std::cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << std::endl;
    // nothing more we can do
    exit(1);
  }
  int im_width = images[0].cols;
  int im_height = images[0].rows;
  std::cout << "im_width = " << im_width << std::endl;
  std::cout << "im_height = " << im_height << std::endl;
  cv::Mat testSample = images[images.size() - 1];
  int testLabel = labels[labels.size() - 1];
  images.pop_back();
  labels.pop_back();

  cv::Ptr<cv::FaceRecognizer> model = cv::createEigenFaceRecognizer(80);
  model->train(images, labels);
  // The following line predicts the label of a given
  // test image:
  int predictedLabel = model->predict(testSample);

  std::string result_message = cv::format("Predicted class = %d / Actual class = %d.", predictedLabel, testLabel);
  std::cout << result_message << std::endl;

  std::string outFile(ros::package::getPath("pandora_vision_victim") + "/data/model.xml");

  std::cout << "Saving model to " << outFile << std::endl;

  model->save(outFile);
}
