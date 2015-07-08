/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
* Authors:
*   Marios Protopapas <protopapas_marios@hotmail.com>
*   Kofinas Miltiadis <mkofinas@gmail.com>
*********************************************************************/

#include <vector>

#include "pandora_vision_victim/feature_extractors/channels_statistics_extractor.h"

namespace pandora_vision
{
namespace pandora_vision_victim
{
  /**
   * @brief This function extracts color related statistic features from a
   * color image.
   * @param src [const cv::Mat&] Color image to be processed.
   * @param colorStatisticsVector [std::vector<double>*] The color
   * statistics vector.
   * @return void
   */
  void ChannelsStatisticsExtractor::findColorChannelsStatisticsFeatures(
      const cv::Mat& src, std::vector<double>* colorStatisticsVector)
  {
    cv::Mat rgbFrame = src.clone();
    cv::Mat hsvFrame;
    /// Transform it to HSV
    cvtColor(rgbFrame, hsvFrame, CV_BGR2HSV);

    /// Preprocess current image to find histograms in HSV planes.
    /// Separate the image in 3 single-channel matrices, one for each of the
    /// Hue, Saturation and Value components.
    std::vector<cv::Mat> hsvPlanes;
    split(hsvFrame, hsvPlanes);

    int hueBins = 180;
    int saturationBins = 256;
    int valueBins = 256;

    /// Set the ranges for each color component.
    float hueRanges[] = {0, 180};
    float saturationRanges[] = {0, 256};
    float valueRanges[] = {0, 256};

    const float* hueHistogramRange = {hueRanges};
    const float* saturationHistogramRange = {saturationRanges};
    const float* valueHistogramRange = {valueRanges};

    cv::Mat hueHistogram, saturationHistogram, valueHistogram;

    bool uniform = true;
    bool accumulate = false;

    /// Compute the histograms for every color component.
    cv::calcHist(&hsvPlanes[0], 1, 0, cv::Mat(), hueHistogram, 1, &hueBins,
        &hueHistogramRange, uniform, accumulate);
    cv::calcHist(&hsvPlanes[1], 1, 0, cv::Mat(), saturationHistogram, 1,
        &saturationBins, &saturationHistogramRange, uniform, accumulate);
    cv::calcHist(&hsvPlanes[2], 1, 0, cv::Mat(), valueHistogram, 1, &valueBins,
        &valueHistogramRange, uniform, accumulate);

    /// Find the mean value and standard deviation value of every color
    /// component.
    std::vector<double> meanStdH = MeanStdDevExtractor(&hsvPlanes[0]).extract();
    std::vector<double> meanStdS = MeanStdDevExtractor(&hsvPlanes[1]).extract();
    std::vector<double> meanStdV = MeanStdDevExtractor(&hsvPlanes[2]).extract();

    /// Find the dominant color component and their density values
    std::vector<double> domValH = DominantColorExtractor(&hueHistogram).extract();
    std::vector<double> domValS = DominantColorExtractor(&saturationHistogram).extract();
    std::vector<double> domValV = DominantColorExtractor(&valueHistogram).extract();

    /// Compute the first 6 Fourier Transform coefficints of the
    /// Hue and Saturation color components.
    std::vector<double> dftH = DFTCoeffsExtractor(&hsvPlanes[0]).extract();
    std::vector<double> dftS = DFTCoeffsExtractor(&hsvPlanes[1]).extract();

    /// Compute the colour angles of the R, G, B color components.
    std::vector<double> colorAnglesAndStd = ColorAnglesExtractor(&rgbFrame).extract();

    /// Append all features to the output feature vector.
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  meanStdH.begin(), meanStdH.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  meanStdS.begin(), meanStdS.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  meanStdV.begin(), meanStdV.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  domValH.begin(), domValH.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  domValS.begin(), domValS.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  domValV.begin(), domValV.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  dftH.begin(), dftH.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  dftS.begin(), dftS.end());
    colorStatisticsVector->insert(colorStatisticsVector->end(),
                                  colorAnglesAndStd.begin(),
                                  colorAnglesAndStd.end());

    if (colorStatisticsVector->size() != 28)
    {
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size = " << colorStatisticsVector->size());
    }
  }

  /**
   * @brief This function extracts color related statistic features from a
   * depth image.
   * @param src [const cv::Mat&] Depth image to be processed.
   * @param depthStatisticsVector [std::vector<double>*] The depth
   * statistics vector.
   * @return void
   */
  void ChannelsStatisticsExtractor::findDepthChannelsStatisticsFeatures(
      const cv::Mat& src, std::vector<double>* depthStatisticsVector)
  {
    cv::Mat inFrame = src.clone();

    if (inFrame.channels() != 1)
    cv::cvtColor(inFrame, inFrame, CV_BGR2GRAY);

    int grayscaleBins = 256;
    /// Set the histogram ranges.
    float grayscaleRanges[] = {0, 256};
    const float* grayscaleHistogramRange = {grayscaleRanges};

    bool uniform = true;
    bool accumulate = false;

    cv::Mat grayscaleHistogram;
    /// Compute the histograms for every color component.
    cv::calcHist(&inFrame, 1, 0, cv::Mat(), grayscaleHistogram, 1,
        &grayscaleBins, &grayscaleHistogramRange, uniform, accumulate);

    /// Find the mean and standard deviation value of the image.
    std::vector<double> meanStd = MeanStdDevExtractor(&inFrame).extract();

    /// Find the dominant color and its density value.
    std::vector<double> domVal = DominantColorExtractor(&grayscaleHistogram).extract();

    /// Compute the first 6 Fourier Transform coefficints of the image.
    std::vector<double> dft = DFTCoeffsExtractor(&inFrame).extract();

    /// Append all features to the output feature vector.
    depthStatisticsVector->insert(depthStatisticsVector->end(),
                                  meanStd.begin(), meanStd.end());
    depthStatisticsVector->insert(depthStatisticsVector->end(),
                                  domVal.begin(), domVal.end());
    depthStatisticsVector->insert(depthStatisticsVector->end(),
                                  dft.begin(), dft.end());

    if (depthStatisticsVector->size() != 10)
    {
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size = " << depthStatisticsVector->size());
    }
  }
}  // namespace pandora_vision_victim
}  // namespace pandora_vision
