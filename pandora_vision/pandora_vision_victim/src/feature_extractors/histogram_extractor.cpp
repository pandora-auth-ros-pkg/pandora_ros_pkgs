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
* Author: Choutas Vassilis <vasilis4ch@gmail.com>
*********************************************************************/

#include <vector>
#include <string>

#include "pandora_vision_victim/feature_extractors/histogram_extractor.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  HistogramExtractor::HistogramExtractor()
  {
    std::vector<int> channels;
    std::vector<int> histBins;
    std::vector<float> histRanges;
    for (int i = 0; i < 3; ++i)
    {
      channels.push_back(i);
      histBins.push_back(32);
      histRanges.push_back(0.0f);
      histRanges.push_back(256.0f);
    }
    int dims = 1;
    std::string colorSpace("BGR");

    HistogramExtractor(channels, dims, histBins, histRanges,
        colorSpace, false);
  }

  /**
   * @brief: The main constructor for the class that takes as input the
   * desired parameters so as to create the histogram for each desired
   * channel of the input image.
   * @param channels[const std::vector<int>&]: The image channels whose
   * histograms we want.
   * @param dims[int]: The dimensions of the histogram.
   * @param histBins[const std::vector<int>&]: The bins of the histogram
   * for every channel.
   * @param histRanges[const std::vector<float>&]: The range of values for
   * the histogram for each channel.
   * @param colorSpace[const std::string&]: The desired color space.
   * @param jointHistogram[bool]: Flag for creating a joint histogram.
   */
  HistogramExtractor::HistogramExtractor(const std::vector<int>& channels,
      int dims,
      const std::vector<int>& histBins,
      const std::vector<float>& histRanges,
      const std::string& colorSpace, bool jointHistogram)
  {
    // Copy the channel vector.
    channels_ = channels;

    // Get the histogram dimensions.
    dims_ = dims;

    // Copy the histogram size in each dimension.
    histBins_ =  histBins;

    // Copy the range of the histogram for each channel.
    ranges_ = histRanges;

    // Convert the input color code to an OpenCV compliant format.
    colorSpace_ = stringToCvColorCode(colorSpace);

    // Flag that specifies whether a joint histogram will be used.
    jointHistogram_ = jointHistogram;
  }

  /**
   * @brief: Constructor for the histogram extractor class that parses
   * the necessary parameters from an xml/yaml file.
   * @param [const cv::FileStorage&]: The file from which the parameters
   * will be read.
   */
  HistogramExtractor::HistogramExtractor(const cv::FileStorage& fs)
  {
    if (!fs.isOpened())
    {
      std::cerr << "Could not open file params.yaml" << std::endl;
      exit(-1);
    }

    cv::FileNode n = fs["channels"];
    if (n.empty())
    {
      std::cerr << "Could not read the channels parameter from the file" <<
        std::endl;
      exit(-1);
    }

    cv::FileNodeIterator it = n.begin(), it_end = n.end();  // Go through the node
    for (; it != it_end; ++it)
    {
      channels_.push_back(static_cast<int>(*it));
    }

    std::vector<float>  ranges;
    if (fs["histRanges"].empty())
    {
      std::cerr << "Could not read the channels parameter from the file" <<
        std::endl;
      exit(-1);
    }
    it = n.begin();
    it_end = n.end();  // Go through the node
    for (; it != it_end; ++it)
    {
      ranges_.push_back(static_cast<float>(*it));
    }

    std::vector<int>  histSize;
    if (fs["histSize"].empty())
    {
      std::cerr << "Could not read the channels parameter from the file" <<
        std::endl;
      exit(-1);
    }

    it = n.begin();
    it_end = n.end();
    for (; it != it_end; ++it)
    {
      histBins_.push_back(static_cast<int>(*it));
    }

    if (!fs["dims"].empty())
      dims_ = fs["dims"];
    else
      dims_ = 1;

    if (!fs["colorSpace"].empty())
      colorSpace_ = stringToCvColorCode(fs["colorSpace"]);
    else
      colorSpace_ = -1;
    std::string jointHistogramFlag;
    if (!fs["jointDistribution"].empty())
    {
      fs["jointDistribution"] >> jointHistogramFlag;
      jointHistogram_ =  jointHistogramFlag.compare("true") == 0;
    }
    else
      jointHistogram_ = false;
  }

  /**
   * @brief: Converts the string color format to an OpenCV color conversion
   * flag.
   * @param colorSpace_[const std::string&]: A string the specifies the
   * desired color space.
   * @return The OpenCV enum value that is used to convert an image from the
   * BGR color space to the specified one.
  */
  int HistogramExtractor::stringToCvColorCode(const std::string& colorSpace_)
  {
    if (colorSpace_.compare("BGR") == 0)
      return -1;

    if (colorSpace_.compare("HSV") == 0)
      return CV_BGR2HSV;

    if (colorSpace_.compare("YCrCb") == 0)
      return CV_BGR2YCrCb;

    if (colorSpace_.compare("LAB") == 0)
      return CV_BGR2Lab;

    // If no successful comparison was made the return a negative code
    // so as to inform the feature extractor that no conversion should be
    // made.
    return -1;
  }

  /**
   * @brief: Creates color histograms from the image channels and concatenate
   * them to form a feature vector.
   * @param inImage[const cv::Mat&]: The input image from which the features
   * will be extracted.
   * @param descriptors[cv::Mat*]: The output feature vector that describes
   * the image
   */
  void HistogramExtractor::extractFeatures(const cv::Mat& inImage,
      cv::Mat* descriptors)
  {
    if (!inImage.data)
    {
      std::cout << "The given image is empty! The histogram features"
        "will not be extracted!" << std::endl;
      return;
    }
    // The image that will be processed to produced the histogram features.
    cv::Mat img;

    // Convert the image to another color space if necessary.
    if (colorSpace_ >= 0)
      cv::cvtColor(inImage, img, colorSpace_);
    else
      img = inImage.clone();

    std::vector<cv::Mat> imgChannels;
    // Split the image into separate channels.
    cv::split(img, imgChannels);

    // The final vector that will contain the histograms.
    std::vector<cv::Mat> histograms(3);

    int totalElements = 0;
    int index;
    for (int i = 0; i < channels_.size(); ++i)
    {
      cv::Mat tempHist;
      // Define the ranges for the histogram.
      index = channels_[i];
      float range[] = {ranges_[2 * i], ranges_[2 * i + 1]};
      const float *histRange = {range};
      // Calculate the histogram for the current channel.
      cv::calcHist(&imgChannels[index], 1, 0, cv::Mat(),
          tempHist, 1, &histBins_[i], &histRange);
      // Normalize the histogram to convert it to a probability
      // distribution.
      cv::normalize(tempHist, tempHist, 0, 1, cv::NORM_MINMAX);
      // Store the resulting histogram.
      histograms[i] = tempHist;
      totalElements += tempHist.rows;
    }

    // Concatenate the histograms so as to form a compact feature vector.
    *descriptors = cv::Mat(totalElements, 1, CV_32FC1);
    int offset = 0;
    for (int i = 0; i < histograms.size(); ++i)
    {
      for (int j = 0; j < histograms[i].rows; ++j) {
        descriptors->at<float>(j + offset) = histograms[i].at<float>(j);
      }
      offset += histograms[i].rows;
    }

    return;
  }

  /**
   * @brief: Plots the input histogram.
   * @param histogram[const cv::Mat&]: The histogram that will be drawn.
   */
  void HistogramExtractor::plotFeatures(const cv::Mat& featureVector)
  {
    int windowHeight = featureVector.rows;
    cv::Mat canvas = cv::Mat::zeros(windowHeight, featureVector.rows, CV_8UC3);
    cv::Scalar green(0, 255, 0);
    for (int i = 0; i < featureVector.rows; ++i)
    {
      cv::line(canvas,
          cv::Point(i, windowHeight),
          cv::Point(i, windowHeight - featureVector.at<float>(i) * windowHeight),
          green,
          1,
          CV_AA,
          0);
    }
    cv::namedWindow("calcHist Demo", CV_WINDOW_NORMAL);
    cv::imshow("calcHist Demo", canvas);
    cv::waitKey(0);

    return;
  }
}  // namespace pandora_vision

