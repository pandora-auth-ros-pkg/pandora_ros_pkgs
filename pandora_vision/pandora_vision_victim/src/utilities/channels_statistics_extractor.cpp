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
@brief Constructor
**/
  ChannelsStatisticsExtractor::ChannelsStatisticsExtractor()
  {
    h_bins = 180;
    s_bins = 256;
    v_bins = 256;
    ROS_INFO("[victim_node] : Created Color detection instance");
    
    //!---------------------EXPERIMENTAL-------------------------//
    
  }
  
  /**
@brief Destructor
*/
  ChannelsStatisticsExtractor::~ChannelsStatisticsExtractor()
  {
    ROS_INFO("[victim_node] : Destroying Color Detection instance");
  }
  
  /**
* @brief This is the main function which calls all others for the computation
* of the color features.
* @param src [cv::Mat] current frame to be processed
* @return void
*/
  void ChannelsStatisticsExtractor::findChannelsStatisticsFeatures(
    const cv::Mat& src
  )
  {
    _colorFeatureVector.clear();
    
    inFrame = src.clone();
    cv::Mat hsv;
    //! Transform it to HSV
    cvtColor( inFrame, hsv, CV_BGR2HSV );
    
    //!Preprocess current image, to find histogramms in HSV planes
    //!< Separate the image in 3 places (H,S,V) one for each channel
    split( hsv, hsv_planes );

    //!< Set the ranges ( for H,S,V)
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };
    float v_ranges[] = { 0, 256 };

    const float* h_histRange = { h_ranges };
    const float* s_histRange = { s_ranges };
    const float* v_histRange = { v_ranges };

    bool uniform = true;
    bool accumulate = false;

    h_hist = computeHist(hsv_planes[0], h_bins, h_histRange);
    s_hist = computeHist(hsv_planes[1], s_bins, s_histRange);
    v_hist = computeHist(hsv_planes[2], v_bins, v_histRange);
    
    //!Find the mean value and std value of every color component
    {
      std::vector<double> f1 = MeanStdDevExtractor(&hsv_planes[0]).extract();
      std::vector<double> f2 = MeanStdDevExtractor(&hsv_planes[1]).extract();
      std::vector<double> f3 = MeanStdDevExtractor(&hsv_planes[2]).extract();
      meanStdHSV = f1;
      meanStdHSV.insert(meanStdHSV.end(), f2.begin(), f2.end());
      meanStdHSV.insert(meanStdHSV.end(), f3.begin(), f3.end());
      _colorFeatureVector.insert(
        _colorFeatureVector.end(),
        meanStdHSV.begin(),
        meanStdHSV.end()
      );
    }
    {
      std::vector<double> f1 = DominantColorExtractor(&h_hist).extract();
      std::vector<double> f2 = DominantColorExtractor(&s_hist).extract();
      std::vector<double> f3 = DominantColorExtractor(&v_hist).extract();
      dominantVal = f1;
      dominantVal.insert(dominantVal.end(), f2.begin(), f2.end());
      dominantVal.insert(dominantVal.end(), f3.begin(), f3.end());
      _colorFeatureVector.insert(
        _colorFeatureVector.end(),
        dominantVal.begin(),
        dominantVal.end()
      );
    }
     
    //!< Compute the modules of first 6 components of a Fourier transform of the
    //!< image components H(hue) and S(saturation).
    huedft = DFTCoeffsExtractor(&hsv_planes[0]).extract();
    _colorFeatureVector.insert(
      _colorFeatureVector.end(),
      huedft.begin(),
      huedft.end()
    );
    satdft = DFTCoeffsExtractor(&hsv_planes[1]).extract();
    _colorFeatureVector.insert(
      _colorFeatureVector.end(),
      satdft.begin(),
      satdft.end()
    );

    //!< Compute the colour angles of rgb color components
    colorAnglesAndStd = ColorAnglesExtractor(&inFrame).extract();
    _colorFeatureVector.insert(
      _colorFeatureVector.end(),
      colorAnglesAndStd.begin(),
      colorAnglesAndStd.end()
    );
  }
  
  /**
* @brief This is the main function which calls all other for the
* computation of the statistics feature for depth image.
* @param src [cv::Mat] depth image to be processed
* @return void
*/
  void ChannelsStatisticsExtractor::findDepthChannelsStatisticsFeatures(
    cv::Mat src)
  {
    if(src.channels() != 1)
    cv::cvtColor(src, src, CV_BGR2GRAY);

    inFrame = src.clone();
    
    computeMeanStd();
    
    //!Find the mean value and std value of every color component
    //computeMeanStdHSV();
    //~ ROS_INFO("Mean and Standard Deviation of HSV :");
    //~ for (int ii = 0; ii < _depthMeanStd.size(); ii++)
      //~ ROS_INFO_STREAM(" " <<_depthMeanStd[ii]);
    //~
    //!< Set the ranges
    float ranges[] = { 0, 256 };
    const float* _histRange = { ranges };
    
    cv::Mat d_hist = computeHist(inFrame, 256, _histRange);
     
    double dominantValue1 = 0;
    double dominantValue2 = 0;
    //! Find the dominant color component and their density values
    findDominantColor(d_hist, 256, &dominantValue1, &dominantValue2 );
    _depthDominantVal.push_back(dominantValue1);
    _depthDominantVal.push_back(dominantValue2);
    
        //~ ROS_INFO("Dominant values and Densities of depth");
     //~ for (int ii = 0; ii < _depthDominantVal.size(); ii++)
       //~ ROS_INFO_STREAM(" " <<_depthDominantVal[ii]);
    
    //!< Compute the modules of first 6 components of a Fourier transform of the
    //!< image components H(hue) and S(saturation).
    _depthdft = computeDFT(inFrame);
    
    //~ ROS_INFO("6 first Dft of depth:");
     //~ for (int ii= 0; ii< _depthdft.size(); ii++)
      //~ ROS_INFO_STREAM(" " << _depthdft[ii]);
    
    extractDepthFeatureVector();
  }
    
  /**
* @brief This function returns the histogram of one color component from
* the src image.
* @param planes [cv::Mat] contains the pixel values of a color component.
* @param bins [int] num of bins where the histogram will be divided.
* @param histRange [const float*] the range of the histogram.
* @return [cv::Mat] the calculated histogram.
*/
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

  /**
* @brief This function computes the average and standard deviation value of
* every color component(HSV) and returns a feature vector.
* @return void
*/
  //~ void ChannelsStatisticsExtractor::computeMeanStdHSV()
  //~ {
    //~ //!-----------------_EXPERIMENTAL_----------------------//
    //~ extractors.push_back(new MeanStdDevExtractor(&hsv_planes[0]));
    //~ extractors.push_back(new MeanStdDevExtractor(&hsv_planes[1]));
    //~ extractors.push_back(new MeanStdDevExtractor(&hsv_planes[2]));
    //~ std::vector<double> f1 = extractors[0]->extract();
    //~ std::vector<double> f2 = extractors[1]->extract();
    //~ std::vector<double> f3 = extractors[2]->extract();
    //~ std::vector<double> total = f1;
    //~ total.insert(total.end(), f2.begin(), f2.end());
    //~ total.insert(total.end(), f3.begin(), f3.end());
//~ 
    //~ meanStdHSV = total;
  //~ }
  
  /**
* @brief This function computes the average and standard deviation value
* of a grayscale image.
* @return void
*/
    void ChannelsStatisticsExtractor::computeMeanStd()
    {
      cv::Scalar avg, st;
      cv::meanStdDev(inFrame, avg, st);
      _depthMeanStd.push_back(avg.val[0]);
      _depthMeanStd.push_back(st.val[0]);
    }
    
  /**
* @brief This function computes the dominant Color and it's density value in
* a color component.
* @param hist [cv::Mat] the histogram of one color component of the image.
* @param histSize [int] the size of the histogram
* @param value [double&] the dominant color value (to be returned).
* @param density [double&] the dominant color density (to be returned).
* @return void
*/
  void ChannelsStatisticsExtractor::findDominantColor( cv::Mat hist, int histSize,
    double* value, double* density )
  {
    double maxVal = 0;
    double val = 0;
    for( int ii= 0; ii < histSize; ii++ )
    {
      double binVal = static_cast<double>(hist.at<float>(ii));
      if(binVal > maxVal)
      {
        maxVal = binVal;
        val = ii;
      }
    }

    *value= val;
    *density= maxVal/(inFrame.rows*inFrame.cols);

  }

  /**
* @brief This function computes the Dft coefficients .
* @param img [cv::Mat] the source image
* @return [std::vector<double>] the feature vector with the 6 first Dft
* coefficients.
*/
  std::vector<double> ChannelsStatisticsExtractor::computeDFT(cv::Mat img)
  {
    DFTCoeffsExtractor extr(&img);
    return extr.extract();
  }

  
  /**
* @brief This function extract a feature vector according to statistcs
* features for the depth image.
* @return void
*/
  void ChannelsStatisticsExtractor::extractDepthFeatureVector()
  {
    _depthStatisticsVector.insert(_depthStatisticsVector.end(),
        _depthMeanStd.begin(), _depthMeanStd.end());
    _depthStatisticsVector.insert(_depthStatisticsVector.end(),
        _depthDominantVal.begin(), _depthDominantVal.end());
    _depthStatisticsVector.insert(_depthStatisticsVector.end(),
        _depthdft.begin(), _depthdft.end());
  }
  
  /**
* @brief Function returning the color statistics feature vector
* @return featureVector
*/
  std::vector<double> ChannelsStatisticsExtractor::getRgbFeatures()
  {
    if( _colorFeatureVector.size() != 28){
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size"<< _colorFeatureVector.size() );
     }
    return _colorFeatureVector;
  }
  
  /**
* @brief Function returning the color statistics feature vector
* @return featureVector
*/
  std::vector<double> ChannelsStatisticsExtractor::getDepthFeatures()
  {
    if( _depthStatisticsVector.size() != 10){
      ROS_FATAL("Clean the vector");
      ROS_INFO_STREAM("vector's size"<< _depthStatisticsVector.size() );
     }
    return _depthStatisticsVector;
  }

  /**
* @brief Function that cleans up depthFeatureVector, to add
* new elements for next frame
* @return void
*/
  void ChannelsStatisticsExtractor::emptyCurrentDepthFrameFeatureVector()
  {
    _depthMeanStd.clear();
    _depthDominantVal.clear();
    _depthdft.clear();
    _depthStatisticsVector.clear();
  }
  
  //!--------------------------EXPERIMENTAL--------------------------//
  //~ std::vector<float> extract(const cv::Mat& img)
  //~ {
    //~ std::vector<float> ret;
    //~ for(unsigned int i = 0 ; i < extractors.size() ; i++)
    //~ {
      //~ std::vector<float> f = extractors[i]->extract(inFrame);
      //~ ret.insert(ret.end(), f.begin(), f.end());
    //~ }
    //~ return ret;
  //~ }
}// namespace pandora_vision
