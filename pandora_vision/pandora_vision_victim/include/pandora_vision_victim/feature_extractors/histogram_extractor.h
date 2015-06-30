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

#ifndef PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HISTOGRAM_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HISTOGRAM_EXTRACTOR_H

#include <string>
#include <vector>
#include <cstdlib>

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "pandora_vision_victim/feature_extractors/feature_extraction.h"

/**
 * @namespace pandora_vision
 * @brief The main namespace for PANDORA vision
 */
namespace pandora_vision
{
  /**
   * @class HistogramExtractor
   * @brief : A feature extractor class that describes an image by its color
   * histograms
   */
  class HistogramExtractor : public FeatureExtractorFactory
  {
  public:
    HistogramExtractor();

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
    HistogramExtractor(const std::vector<int>& channels,
      int dims,
      const std::vector<int>& histBins,
      const std::vector<float>& histRanges,
      const std::string& colorSpace, bool jointHistogram = false);

    /**
     * @brief: Constructor for the histogram extractor class that parses
     * the necessary parameters from an xml/yaml file.
     * @param [const cv::FileStorage&]: The file from which the parameters
     * will be read.
     */
    explicit HistogramExtractor(const cv::FileStorage& fs);

    virtual void plotFeatures(const cv::Mat& featureVector);

    /**
     * @brief: Default Destructor/
     */
    virtual ~HistogramExtractor()
    {
    }

    /**
     * @brief: Creates color histograms from the image channels and concatenate
     * them to form a feature vector.
     * @param inImage[const cv::Mat&]: The input image from which the features
     * will be extracted.
     * @param descriptors[cv::Mat*]: The output feature vector that describes
     * the image
     */
    virtual void extractFeatures(const cv::Mat& inImage,
        cv::Mat* descriptors);

    /**
     * @brief: Converts the string color format to an OpenCV color conversion
     * flag.
     * @param colorCode[const std::string&]: A string the specifies the 
     * desired color space.
     * @return The OpenCV enum value that is used to convert an image from the
     * BGR color space to the specified one.
     */
    int stringToCvColorCode(const std::string& colorCode);

    /**
     * @brief: Plots the input histogram.
     * @param histogram[const cv::Mat&]: The histogram that will be drawn.
     */
    void plotHistogram(const cv::Mat& histogram);

    /**
     * @brief
     */
    virtual void extractFeatures(const cv::Mat& inImage,
        std::vector<double>* featureVector)
    {
    }

    /**
     * @brief
     */
    virtual void extractFeatures(const cv::Mat& inImage,
        std::vector<float>* featureVector)
    {
    }

    /**
     * @brief: Function used to change the color space for the image
     * histograms
     */
    void setColorSpace(const std::string& colorSpace)
    {
      colorSpace_ = stringToCvColorCode(colorSpace);
    }
  private:
    /* data */
    /// The channels of the image that will be used to create the histogram.
    std::vector<int> channels_;

    /// The dimensions of the histogram;
    int dims_;

    /// The size of the histogram in each dimension.
    std::vector<int> histBins_;

    /// The range of the histogram values in each dimension.
    std::vector<float> ranges_;

    /// Flag that specifies whether we should extract a histogram for each
    /// channel or a joint histogram for all the specified channels.
    bool jointHistogram_;

    /// The color space of the histogram.
    int colorSpace_;
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_FEATURE_EXTRACTORS_HISTOGRAM_EXTRACTOR_H
