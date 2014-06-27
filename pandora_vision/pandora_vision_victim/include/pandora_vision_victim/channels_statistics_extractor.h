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
* Author: Marios Protopapas
*********************************************************************/
#ifndef PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H

#include <opencv2/opencv.hpp>
#include "ros/ros.h"

namespace pandora_vision
{
  class ChannelsStatisticsExtractor 
  {
    //!< Establish the number of bins
    int h_bins, s_bins, v_bins;
    
    cv::Mat inFrame;
    
    std::vector<cv::Mat> hsv_planes;
    //!< Creation of 3 Mat objects to save each histogramm
    cv::Mat h_hist, s_hist, v_hist;
    
    //!< Vector of mean value and std value of every color component
    std::vector<double> meanStdHSV;
    //!< Vector of dominant color component and their density values
    std::vector<double> dominantVal;
    //!< Vector of first 6 components of a Fourier transform of the 
    //!< image components H(hue) and S(saturation).
    std::vector<double> huedft;
    std::vector<double> satdft;
    //!< Compute the colour angles of rgb color components
    std::vector<double> colorAnglesAndStd;
    
    //!< Compute a vector of all color features
    std::vector<double> _colorFeatureVector;
    
    //!< Vector of mean value and std value of the depth Image
    std::vector<double> _depthMeanStd;
    //!< Vector of dominant color component and their density values
    std::vector<double> _depthDominantVal;
    //!< Vector of first 6 components of a Fourier transform of the 
    //!< image components.
    std::vector<double> _depthdft;
    
    //!< Compute a vector of all depth features
    std::vector<double> _depthStatisticsVector;
    
    /**
     * @brief This function returns the histogram of one color component from 
     * the src image.
     * @param planes [cv::Mat] contains the pixel values of a color component.
     * @param bins [int] num of bins where the histogram will be divided.
     * @param histRange [const float*] the range of the histogram.
     * @return [cv::Mat] the calculated histogram.
    */ 
    cv::Mat computeHist(cv::Mat planes, int bins, const float* histRange);
    
    /**
     * @brief This function computes the average and standard deviation value  
     * of every color component(HSV).
     * @return void
    */ 
    void computeMeanStdHSV();
    
    /**
     * @brief This function computes the average and standard deviation value  
     * of a grayscale image.
     * @return void
    */ 
    void computeMeanStd();
    
    /**
     * @brief This function computes the dominant Color and it's density value 
     * in a color component.
     * @param hist [cv::Mat] the histogram of one color component of the image.
     * @param histSize [int] the size of the histogram
     * @param value [double&] the dominant color value (to be returned).
     * @param density [double&] the dominant color density (to be returned).
     * @return void
    */ 
    void findDominantColor(cv::Mat hist, int histSize, double*
                value, double* density);
    
    /**
     * @brief This function computes the Dft coefficients . 
     * @param img [cv::Mat] the source image
     * @return [std::vector<double>] the feature vector with the 6 first Dft 
     * coefficients.
    */
    std::vector<double> computeDFT(cv::Mat img);
    
    /**
     * @brief This function computes the color angles of the image and the 
     * normalized intensity std . 
     * @return void
    */
    void computeColorAngles();
    
    public:
    
    //!Constructor
    ChannelsStatisticsExtractor();
    
    //!Destructor
    virtual ~ChannelsStatisticsExtractor();
    
    /**
     * @brief This is the main function which calls all other for the 
     * computation of the color features.
     * @param src [cv::Mat] current frame to be processed
     * @return void
    */ 
    void findChannelsStatisticsFeatures(cv::Mat src);
    
    /**
     * @brief This is the main function which calls all other for the 
     * computation of the statistics feature for depth image.
     * @param src [cv::Mat] depth image to be processed
     * @return void
    */ 
    void findDepthChannelsStatisticsFeatures(cv::Mat src);
    
    /**
     * @brief This function calculates all necessary histogramms
     * for color extraction.
     * @param hsvFrame [cv::Mat] current frame to be processed
     * @return void
    */ 
    void setHistogramms(cv::Mat hsvFrame);
    
    /**
     * @brief This function extract a feature vector according to color
     * and statistcs features.
     * @return void
     */ 
    void extractColorFeatureVector();
    
    /**
     * @brief This function extract a feature vector according to statistcs 
     * features for the depth image.
     * @return void
     */ 
    void extractDepthFeatureVector();
    
    /**
     * @brief Function returning the color statistics feature vector
     * @return featureVector
     */ 
    std::vector<double> getRgbFeatures();
    
    /**
     * @brief Function returning the depth feature vector
     * @return featureVector
    */ 
    std::vector<double> getDepthFeatures();
    
    /**
     * @brief Function that cleans up colorFeatureVector, to add
     * new elements for next frame
     * @return void
     */ 
    void emptyCurrentFrameFeatureVector();
    
    /**
     * @brief Function that cleans up depthFeatureVector, to add
     * new elements for next frame
     * @return void
    */ 
    void emptyCurrentDepthFrameFeatureVector();
  };
  
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_CHANNELS_STATISTICS_EXTRACTOR_H
