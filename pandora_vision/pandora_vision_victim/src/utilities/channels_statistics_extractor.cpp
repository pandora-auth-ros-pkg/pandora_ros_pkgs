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
* Author: Marios Protopapas
*********************************************************************/

#include "pandora_vision_victim/utilities/channels_statistics_extractor.h"

namespace pandora_vision
{
  /**
  @brief This is the main function which calls all others for the computation
  of the color features.
  @param src [cv::Mat] : current frame to be processed
  @return void
 **/
  void ChannelsStatisticsExtractor::findChannelsStatisticsFeatures(const cv::Mat& src,
                                                                   std::vector<double>* rgbStatisticsVector)
  {
    cv::Mat inFrame = src.clone();
    cv::Mat hsv;
    //! Transform it to HSV
    cvtColor( inFrame, hsv, CV_BGR2HSV );

    //!Preprocess current image, to find histogramms in HSV planes
    //!< Separate the image in 3 places (H,S,V) one for each channel

    std::vector<cv::Mat> hsv_planes;
    split( hsv, hsv_planes );

    int h_bins = 180;
    int s_bins = 256;
    int v_bins = 256;

    //!< Set the ranges ( for H,S,V)
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    float v_ranges[] = { 0, 256 };

    const float* h_histRange = { h_ranges };
    const float* s_histRange = { s_ranges };
    const float* v_histRange = { v_ranges };

    cv::Mat h_hist, s_hist, v_hist;

    h_hist = computeHist(hsv_planes[0], h_bins, h_histRange);
    s_hist = computeHist(hsv_planes[1], s_bins, s_histRange);
    v_hist = computeHist(hsv_planes[2], v_bins, v_histRange);

    //!Find the mean value and std value of every color component

    std::vector<double> meanStdH = MeanStdDevExtractor(&hsv_planes[0]).extract();
    std::vector<double> meanStdS = MeanStdDevExtractor(&hsv_planes[1]).extract();
    std::vector<double> meanStdV = MeanStdDevExtractor(&hsv_planes[2]).extract();


    //! Find the dominant color component and their density values

    std::vector<double> domValH = DominantColorExtractor(&h_hist).extract();
    std::vector<double> domValS = DominantColorExtractor(&s_hist).extract();
    std::vector<double> domValV = DominantColorExtractor(&v_hist).extract();

      //ROS_INFO("Dominant values and Densities of HSV");
     //for (int ii = 0; ii < _dominantVal.size(); ii++)
             //ROS_INFO_STREAM(" " << _dominantVal[ii]);

    //!< Compute the modules of first 6 components of a Fourier transform of the
    //!< image components H(hue) and S(saturation).
    std::vector<double> dftH = DFTCoeffsExtractor(&hsv_planes[0]).extract();
    std::vector<double> dftS = DFTCoeffsExtractor(&hsv_planes[1]).extract();

    //!< Compute the colour angles of rgb color components
    std::vector<double> colorAnglesAndStd = ColorAnglesExtractor(&inFrame).extract();

    //!< Append the final rgbStatisticsVector
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        meanStdH.begin(), meanStdH.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        meanStdS.begin(), meanStdS.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        meanStdV.begin(), meanStdV.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        domValH.begin(), domValH.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        domValS.begin(), domValS.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        domValV.begin(), domValV.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        dftH.begin(), dftH.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        dftS.begin(), dftS.end());
    rgbStatisticsVector->insert(rgbStatisticsVector->end(),
        colorAnglesAndStd.begin(), colorAnglesAndStd.end());

    if( rgbStatisticsVector->size() != 28){
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size"<< rgbStatisticsVector->size() );
     }

  }

  /**
  @brief This is the main function which calls all other for the
  computation of the statistics feature for depth image.
  @param src [cv::Mat] : depth image to be processed
  @return void
  **/
  void ChannelsStatisticsExtractor::findDepthChannelsStatisticsFeatures(const cv::Mat& src,
                                                                        std::vector<double>* depthStatisticsVector)
  {

    cv::Mat inFrame = src.clone();

    if(inFrame.channels() != 1)
    cv::cvtColor(inFrame, inFrame, CV_BGR2GRAY);

    //!< Set the ranges
    float ranges[] = { 0, 256 };
    const float* _histRange = { ranges };

    cv::Mat d_hist = computeHist(inFrame, 256, _histRange);

    //!Find the mean value and std value of every color component
    std::vector<double> meanStd = MeanStdDevExtractor(&inFrame).extract();

    //! Find the dominant color component and their density values
    std::vector<double> domVal = DominantColorExtractor(&d_hist).extract();

        //~ ROS_INFO("Dominant values and Densities of depth");
     //~ for (int ii = 0; ii < domVal.size(); ii++)
       //~ ROS_INFO_STREAM(" " <domVal[ii]);

    //!< Compute the modules of first 6 components of a Fourier transform of the
    //!< image components.
    std::vector<double> dft = DFTCoeffsExtractor(&inFrame).extract();


    //~ ROS_INFO("6 first Dft of depth:");
     //~ for (int ii= 0; ii< dft.size(); ii++)
      //~ ROS_INFO_STREAM(" " << dft[ii]);

      //!< Append the final depthStatisticsVector
    depthStatisticsVector->insert(depthStatisticsVector->end(),
        meanStd.begin(), meanStd.end());
      depthStatisticsVector->insert(depthStatisticsVector->end(),
        domVal.begin(), domVal.end());
    depthStatisticsVector->insert(depthStatisticsVector->end(),
        dft.begin(), dft.end());

    if( depthStatisticsVector->size() != 10 )
    {
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size"<< depthStatisticsVector->size() );
     }


  }

  /**
  @brief This function returns the histogram of one color component from
  the src image.
  @param planes [cv::Mat] contains the pixel values of a color component.
  @param bins [int] num of bins where the histogram will be divided.
  @param histRange [const float*] the range of the histogram.
  @return [cv::Mat] the calculated histogram.
  **/
  cv::Mat ChannelsStatisticsExtractor::computeHist(cv::Mat planes, int histSize,
    const float* histRange)
  {
    bool uniform = true;
    bool accumulate = false;
    cv::Mat hist;

    //!< Compute the histograms
    cv::calcHist( &planes, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange,
    uniform, accumulate );
    return hist;
  }
}// namespace pandora_vision
