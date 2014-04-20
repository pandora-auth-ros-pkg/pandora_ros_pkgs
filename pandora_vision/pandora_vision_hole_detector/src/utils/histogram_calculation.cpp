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

#include "utils/histogram_calculation.h"

namespace pandora_vision
{
    /**
      @brief Computes a cv::MatND histogram from images loaded in directory
      ${pandora_vision_hole_detector}/src/walls
      @param[in] secondaryChannel [const int&] Which channel to use, aside the
      hue one. 1 for the Saturation channel, 2 for the Value channel
      @param[out] The calculated histogram
      @return void
     **/
  void HistogramCalculation::getHistogram (
    const int& secondaryChannel,
    cv::MatND* histogram)
  {
    #ifdef DEBUG_TIME
    Timer::start("getHistogram", "", true);
    #endif

    //!< The path to the package where the wall pictures directory lies in
    std::string packagePath =
      ros::package::getPath("pandora_vision_hole_detector");

    //!< The actual wall pictures directory
    std::string wallPicturesPath = packagePath + "/walls/";

    int fileLength;

    //!< The number of wall picture files inside the wallPicturesPath directory
    int numPictures = 0;

    struct dirent* result = NULL;

    int nameMax = pathconf(wallPicturesPath.c_str(), _PC_NAME_MAX);
    int len = offsetof(struct dirent, d_name) + nameMax + 1;
    struct dirent *theDir = static_cast<struct dirent*>(malloc(len));

    DIR *directory;

    directory = opendir(wallPicturesPath.c_str());
    if (theDir != NULL)
    {
      while ((readdir_r(directory, theDir, &result)) == 0 && result!= NULL)
      {
        fileLength = strlen(theDir->d_name);
        if (strcmp (".png", &(theDir->d_name[fileLength - 4])) == 0)
        {
          numPictures++;
        }
      }
      closedir (directory);
    }

    //!< Read the pictures inside the wallPicturesPath, convert them to HSV
    //!< and calculate their histogram
    cv::Mat* wallImagesHSV = new cv::Mat[numPictures];
    for(int i = 0; i < numPictures; i++)
    {
      char temp_name[250];

      std::string temp = wallPicturesPath +"%d.png";

      sprintf(temp_name, temp.c_str(), i);

      cv::cvtColor(
        Visualization::scaleImageForVisualization(cv::imread(temp_name),
          Parameters::scale_method),
        wallImagesHSV[i], cv::COLOR_BGR2HSV);
    }

    //!< Histogram-related parameters
    int h_bins = Parameters::number_of_hue_bins;

    int* histSize = new int[2];
    histSize[0] = h_bins;

    if (secondaryChannel == 1)
    {
      histSize[1] = Parameters::number_of_saturation_bins;
    }

    if (secondaryChannel == 2)
    {
      histSize[1] = Parameters::number_of_value_bins;
    }

    //!< hue varies from 0 to 179, saturation and value from 0 to 255
    float h_ranges[] = { 0, 180 };
    float sec_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, sec_ranges };

    //!< Use the 0-th and secondaryChannel-st channels
    int channels[] = { 0, secondaryChannel };

    //!< Calculate the histogram for the walls
    cv::calcHist(wallImagesHSV, numPictures, channels, cv::Mat(),
      *histogram, 2, histSize, ranges, true, false);

    delete[] wallImagesHSV;
    delete[] histSize;

    #ifdef DEBUG_TIME
    Timer::tick("getHistogram");
    #endif
  }

} // namespace pandora_vision
