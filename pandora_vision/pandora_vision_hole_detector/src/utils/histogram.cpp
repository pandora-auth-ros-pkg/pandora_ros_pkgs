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
 * Authors: Despoina Paschalidou, Alexandros Philotheou
 *********************************************************************/

#include "utils/histogram.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Function for calculating the backprojection of an image,
    based on the histogram @param modelHistogram
    @param[in] inImage [const cv::Mat&] Current frame to be processed
    @param[in] modelHistogram [const std::vector<cv::MatND>&]
    A vector of model histograms based on which
    the @param backprojection is produced
    @param[out] backprojection [cv::Mat*] Backprojection of the current
    frame
    @param[in] secondaryChannel [const int&] Which channel to use, aside the
    hue one. 1 for the Saturation channel, 2 for the Value channel
    @return void
   **/
  void Histogram::getBackprojection(const cv::Mat& inImage,
    const std::vector<cv::MatND>& modelHistogram,
    cv::Mat* backprojection,
    const int& secondaryChannel)
  {
#ifdef DEBUG_TIME
    Timer::start("applyBackprojection");
#endif

    // The vector of backprojection images corresponding to each
    // discrete model histogram
    std::vector<cv::Mat> backprojections;

    for (int i = 0; i < modelHistogram.size(); i++)
    {
      // Convert the inImage image from RGB to HSV
      cv::Mat hsv;
      cvtColor(inImage, hsv, CV_BGR2HSV);

      // hue varies from 0 to 180
      float hranges[] = { 0, 180 };

      // saturation or value varies from 0 to 256
      float sec_ranges[] = { 0, 256 };

      const float* ranges[] = { hranges, sec_ranges};

      if (secondaryChannel == 1 || secondaryChannel == 2)
      {
        // Use the 0-th (Hue) and secondaryChannel channels
        int channels[] = {0, secondaryChannel};

        cv::Mat tempBackproject;

        // Calculate the backproject of the input image depending on the
        // i-th histogram
        cv::calcBackProject(&hsv, 1, channels, modelHistogram[i],
          tempBackproject, ranges, 1, true);

        backprojections.push_back(tempBackproject);
      }
      else
      {
        return;
      }
    }

    // Combine all backprojections into one. The value of each pixel of the
    // overall backprojection shall be the maximum value of the
    // sub-backprojections for that pixel
    for (int i = 0; i < backprojections.size(); i++)
    {
      unsigned char* ptr = backprojections[i].ptr();
      for (int rows = 0; rows < inImage.rows; rows++)
      {
        for (int cols = 0; cols < inImage.cols; cols++)
        {
          int ind = rows * inImage.cols + cols;
          if (ptr[ind] > backprojection->at<unsigned char>(rows, cols))
          {
            backprojection->at<unsigned char>(rows, cols) = ptr[ind];
          }
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("applyBackprojection");
    #endif
  }



  /**
    @brief Computes a cv::MatND histogram from images loaded in directory
    ${pandora_vision_hole_detector}/src/walls
    @param [out] histogram [std::vector<cv::MatND>*] The calculated
    vector of histograms
    @param [in] secondaryChannel [const int&] Which channel to use, aside the
    hue one. 1 for the Saturation channel, 2 for the Value channel
    @return void
   **/
  void Histogram::getHistogram (
    std::vector<cv::MatND>* histogram,
    const int& secondaryChannel)
  {
    #ifdef DEBUG_TIME
    Timer::start("getHistogram", "", true);
    #endif

    // The path to the package where the wall pictures directory lies in
    std::string packagePath =
      ros::package::getPath("pandora_vision_hole_detector");

    // The actual wall pictures directory
    std::string wallsDirectory = packagePath + "/walls/";

    int fileLength;

    // The number of textures inside the wallsDirectory directory
    int numTextures = 0;

    struct dirent* result = NULL;

    int nameMax = pathconf(wallsDirectory.c_str(), _PC_NAME_MAX);
    int len = offsetof(struct dirent, d_name) + nameMax + 1;
    struct dirent *theDir = static_cast<struct dirent*>(malloc(len));

    DIR *directory;

    directory = opendir(wallsDirectory.c_str());


    // The walls directory should exist; otherwise abort.
    if (directory == NULL)
    {
      ROS_FATAL_NAMED(PKG_NAME, "/walls not found. Create it, fill it with "
        "images of walls and continue..");

      ros::shutdown();

      return;
    }

    // Find how many texture directories there are inside the walls directory.
    // The name of each directory shall be "texture_X" where X denotes an
    // integer starting from 0.
    if (theDir != NULL)
    {
      while ((readdir_r(directory, theDir, &result)) == 0 && result != NULL)
      {
        // The name of each listing in the walls directory in string format
        std::string textureDir(theDir->d_name);

        // The position of the "_" separator
        std::size_t underscore_pos = textureDir.find("texture_");
        if (underscore_pos != std::string::npos)
        {
          numTextures++;
        }
      }

      closedir (directory);
    }

    // Traverse all textures and create a histogram for each one
    for (int texture = 0; texture < numTextures; texture++)
    {
      char wallPicturesPath[250];

      // The actual wall pictures directory
      std::string tempWallPicturesPath =
        packagePath + "/walls/texture_" + "%d/";

      sprintf(wallPicturesPath, tempWallPicturesPath.c_str(), texture);

      int fileLength;

      // The number of wall picture files inside the textures directory
      int numPictures = 0;

      struct dirent* result = NULL;

      int nameMax = pathconf(wallPicturesPath, _PC_NAME_MAX);
      int len = offsetof(struct dirent, d_name) + nameMax + 1;
      struct dirent *theDir = static_cast<struct dirent*>(malloc(len));

      DIR *directory;

      directory = opendir(wallPicturesPath);

      // Find out how many images there are in each texture_X directory
      if (theDir != NULL)
      {
        while ((readdir_r(directory, theDir, &result)) == 0 && result != NULL)
        {
          fileLength = strlen(theDir->d_name);
          if (strcmp (".png", &(theDir->d_name[fileLength - 4])) == 0)
          {
            numPictures++;
          }
        }

        closedir (directory);
      }

      // If no images are found in the "texture_X" directory, shutdown.
      if (numPictures == 0)
      {
        ROS_FATAL_NAMED(PKG_NAME, "one of the textures directory is empty."
          "Fill it with images of walls and continue. Shutting down..");

        ros::shutdown();

        return;
      }

      // Read the pictures inside the wallPicturesPath, convert them to HSV
      // and calculate their histogram
      cv::Mat* wallImagesHSV = new cv::Mat[numPictures];
      for(int i = 0; i < numPictures; i++)
      {
        char temp_name[250];

        std::string wallPicturesPathString(wallPicturesPath);

        std::string temp = wallPicturesPathString + "%d.png";

        sprintf(temp_name, temp.c_str(), i);

        cv::cvtColor(
          Visualization::scaleImageForVisualization(cv::imread(temp_name),
            Parameters::Image::scale_method),
          wallImagesHSV[i], cv::COLOR_BGR2HSV);
      }

      int* histSize = new int[2];

      // The first value will always be with regard to Hue
      histSize[0] = Parameters::Histogram::number_of_hue_bins;

      if (secondaryChannel == 1)
      {
        histSize[1] = Parameters::Histogram::number_of_saturation_bins;
      }

      if (secondaryChannel == 2)
      {
        histSize[1] = Parameters::Histogram::number_of_value_bins;
      }
      else
      {
        histSize[1] = Parameters::Histogram::number_of_value_bins;
      }

      // hue varies from 0 to 179, saturation and value from 0 to 255
      float h_ranges[] = { 0, 180 };
      float sec_ranges[] = { 0, 256 };

      const float* ranges[] = { h_ranges, sec_ranges };

      // Use the 0-th and secondaryChannel-st channels
      int channels[] = { 0, secondaryChannel };

      // A temporary histogram where the computed histogram will be put
      cv::MatND tempHistogram;

      // Calculate the histogram for the walls
      cv::calcHist(wallImagesHSV, numPictures, channels, cv::Mat(),
        tempHistogram, 2, histSize, ranges, true, false);

      // Append the histogram of this collection of texture images to
      // the overall vector of histograms
      histogram->push_back(tempHistogram);

      delete[] wallImagesHSV;
      delete[] histSize;
    }

    #ifdef DEBUG_TIME
    Timer::tick("getHistogram");
    #endif
  }

} // namespace pandora_vision
