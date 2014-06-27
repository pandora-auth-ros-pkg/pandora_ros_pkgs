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

#include "pandora_vision_victim/channels_statistics_extractor.h"

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
  }
  
  /**
@brief Destructor
*/
  ChannelsStatisticsExtractor::~ChannelsStatisticsExtractor()
  {
    ROS_INFO("[victim_node] : Destroying Color Detection instance");
  }
  
  /**
* @brief This function calculates all necessary histogramms
* for color extraction.
* @param hsvFrame [cv::Mat] current frame to be processed
* @return void
*/
  void ChannelsStatisticsExtractor::setHistogramms(cv::Mat hsvFrame)
  {
    //!< Separate the image in 3 places (H,S,V) one for each channel
    split( hsvFrame, hsv_planes );

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
    
  }
  
  /**
* @brief This is the main function which calls all others for the computation
* of the color features.
* @param src [cv::Mat] current frame to be processed
* @return void
*/
  void ChannelsStatisticsExtractor::findChannelsStatisticsFeatures(cv::Mat src)
  {
    inFrame = src.clone();
    cv::Mat hsv;
    //! Transform it to HSV
    cvtColor( src, hsv, CV_BGR2HSV );
    //!Preprocess current image, to find histogramms in HSV planes
    setHistogramms(hsv);
    
    //!Find the mean value and std value of every color component
    computeMeanStdHSV();
    //~ ROS_INFO("Mean and Standard Deviation of HSV :");
    //~ for (int ii = 0; ii < meanStdHSV.size(); ii++)
      //~ ROS_INFO_STREAM(" " <<meanStdHSV[ii]);
    
    double dominantValue1 = 0;
    double dominantValue2 = 0;
    //! Find the dominant color component and their density values
    findDominantColor(h_hist, h_bins, &dominantValue1, &dominantValue2 );
    dominantVal.push_back(dominantValue1);
    dominantVal.push_back(dominantValue2);
    findDominantColor(s_hist, s_bins, &dominantValue1, &dominantValue2 );
    dominantVal.push_back(dominantValue1);
    dominantVal.push_back(dominantValue2);
    findDominantColor(v_hist, v_bins, &dominantValue1, &dominantValue2 );
    dominantVal.push_back(dominantValue1);
    dominantVal.push_back(dominantValue2);
     
    //~ ROS_INFO("Dominant values and Densities of every colorcom HSV");
    //~ for (int ii = 0; ii < dominantVal.size(); ii++)
      //~ ROS_INFO_STREAM(" " <<dominantVal[ii]);
     
    //!< Compute the modules of first 6 components of a Fourier transform of the
    //!< image components H(hue) and S(saturation).
    huedft = computeDFT(hsv_planes[0]);
    satdft = computeDFT(hsv_planes[1]);
     
    //~ ROS_INFO("6 first Dft of Hue:");
    //~ for (int ii= 0; ii< huedft.size(); ii++)
      //~ ROS_INFO_STREAM(" " << huedft[ii]);
    //~ ROS_INFO("6 first Dft of Sat");
    //~ for (int ii = 0; ii< satdft.size(); ii++)
      //~ ROS_INFO_STREAM(" " << satdft[ii]);
    
    //!< Compute the colour angles of rgb color components
    computeColorAngles();
    //~ ROS_INFO("Color Angles and normalized intensity std");
    //~ for (int ii = 0; ii< colorAnglesAndStd.size(); ii++)
      //~ ROS_INFO_STREAM(" " << colorAnglesAndStd[ii]);
    //~
    extractColorFeatureVector();
  }
  
  /**
* @brief This is the main function which calls all other for the
* computation of the statistics feature for depth image.
* @param src [cv::Mat] depth image to be processed
* @return void
*/
  void ChannelsStatisticsExtractor::findDepthChannelsStatisticsFeatures(cv::Mat src)
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
  void ChannelsStatisticsExtractor::computeMeanStdHSV()
  {
    if(!meanStdHSV.empty())
      meanStdHSV.erase(meanStdHSV.begin(), meanStdHSV.begin()+meanStdHSV.size());
    cv::Scalar avg, st;
    cv::meanStdDev(hsv_planes[0], avg, st);
    meanStdHSV.push_back(avg.val[0]);
    meanStdHSV.push_back(st.val[0]);
    cv::meanStdDev(hsv_planes[1], avg, st);
    meanStdHSV.push_back(avg.val[0]);
    meanStdHSV.push_back(st.val[0]);
    cv::meanStdDev(hsv_planes[2], avg, st);
    meanStdHSV.push_back(avg.val[0]);
    meanStdHSV.push_back(st.val[0]);
  }
  
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
    std::vector<double>temp(6);
    cv::Mat padded;
    
    //!< Expand input image to optimal size
    int rows = cv::getOptimalDFTSize( img.rows );
    int cols = cv::getOptimalDFTSize( img.cols );
    
    //!< On the border add zero values
    copyMakeBorder(img, padded, 0, rows - img.rows, 0, cols - img.cols,
    cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = {cv::Mat_<float>(padded),
                        cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    
    //!< Add to the expanded another plane with zeros
    merge(planes, 2, complexI);
    
    //!< This way the result may fit in the source matrix
    dft(complexI, complexI);
    
    //!< Normalize the dft coeffs
    for (int ii = 0; ii < complexI.rows; ii++)
      for(int jj = 0; jj < complexI.cols; jj++)
          complexI.at<float>(ii, jj)=complexI.at<float>(ii, jj) /
                                      (complexI.cols * complexI.rows);
              

    //!< Compute the magnitude
    // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    split(complexI, planes);
    
    //!< planes[0] = magnitude
    magnitude(planes[0], planes[1], planes[0]);
    cv::Mat magI = planes[0];
    temp[0] = static_cast<double>(magI.at<float>(0, 0));
    temp[1] = static_cast<double>(magI.at<float>(0, 1));
    temp[2] = static_cast<double>(magI.at<float>(1, 0));
    temp[3] = static_cast<double>(magI.at<float>(0, 2));
    temp[4] = static_cast<double>(magI.at<float>(1, 1));
    temp[5] = static_cast<double>(magI.at<float>(2, 0));
    return temp;
  }

  /**
* @brief This function computes the color angles of the image and the
* normalized intensity std .
* @return void
*/
  void ChannelsStatisticsExtractor::computeColorAngles()
  {
    //!< Separate the image in 3 places (R,G,B) one for each channel
    std::vector<cv::Mat> rgb_planes;
    split( inFrame, rgb_planes );
    //!< Compute the average pixel value of each r,g,b color component
    cv::Scalar bmean = mean(rgb_planes[0]);
    cv::Scalar gmean = mean(rgb_planes[1]);
    cv::Scalar rmean = mean(rgb_planes[2]);
    
    //!< Obtain zero-mean colour vectors r0, g0 and b0 by subtracting the
    //!< Corresponding average pixel value of each original colour vector
    cv::Mat r0 = cv::Mat::zeros(inFrame.rows, inFrame.cols, CV_64FC1);
    cv::Mat b0 = cv::Mat::zeros(inFrame.rows, inFrame.cols, CV_64FC1);
    cv::Mat g0 = cv::Mat::zeros(inFrame.rows, inFrame.cols, CV_64FC1);
    
    for (int ii = 0; ii < inFrame.rows; ii++)
      for (int jj = 0; jj < inFrame.cols; jj++)
        b0.at<double>(ii, jj) = rgb_planes[0].at<uchar>(ii, jj) - bmean.val[0];
    for (int ii = 0; ii < inFrame.rows; ii++)
      for (int jj = 0; jj < inFrame.cols; jj++)
        g0.at<double>(ii, jj) = rgb_planes[1].at<uchar>(ii, jj) - gmean.val[0];
    for (int ii = 0; ii < inFrame.rows; ii++)
      for (int jj = 0; jj < inFrame.cols; jj++)
        r0.at<double>(ii, jj) = rgb_planes[2].at<uchar>(ii, jj) - rmean.val[0];

    double rgdot = r0.dot(g0);
    double gbdot = g0.dot(b0);
    double rbdot = r0.dot(b0);
    double rsum = 0, bsum = 0, gsum = 0;
      
    //!< Compute the dot product of rgb color components
    for (int ii = 0; ii < r0.rows; ii++)
      for(int jj = 0; jj < r0.cols; jj++)
        {
           rsum+= pow(r0.at<double>(ii, jj), 2);
           gsum+= pow(g0.at<double>(ii, jj), 2);
           bsum+= pow(b0.at<double>(ii, jj), 2);
        }
      
    double rlength = sqrt(rsum);
    double glength = sqrt(gsum);
    double blength = sqrt(bsum);
    rgdot/= (rlength*glength);
    gbdot/= (glength*blength);
    rbdot/= (rlength*blength);

    //!< Compute the color angles
    double rgAngle = acos(rgdot);
    double gbAngle = acos(gbdot);
    double rbAngle = acos(rbdot);
    
    cv::Mat hsv;
    //!< Normalised intensity standard deviation
    //!< Transform the src image to grayscale
    cvtColor( inFrame, hsv, CV_BGR2GRAY );
    
    //!< Compute the mean intensity value
    cv::Scalar meanI = mean(hsv);
    
   
    //!< Find the maximum intensity value
    double maxVal, std, sum = 0;
    minMaxLoc( hsv, NULL, &maxVal );
        
    for(int ii = 0; ii < hsv.rows; ii++)
      for(int jj = 0; jj < hsv.cols; jj++)
      {
        sum+= pow((hsv.at<uchar>(ii, jj) - meanI.val[0]), 2);
      }
      
    std = 2.0 / (maxVal * hsv.cols * hsv.rows) * sqrt(sum);
   
    //!< Construct the final feature vector
    colorAnglesAndStd.push_back(rgAngle);
    colorAnglesAndStd.push_back(gbAngle);
    colorAnglesAndStd.push_back(rbAngle);
    colorAnglesAndStd.push_back(std);
  }
  
  /**
* @brief This function extract a feature vector according to color
* and statistcs features.
* @return void
*/
  void ChannelsStatisticsExtractor::extractColorFeatureVector()
  {
    _colorFeatureVector.insert(_colorFeatureVector.end(),
        meanStdHSV.begin(), meanStdHSV.end());
    _colorFeatureVector.insert(_colorFeatureVector.end(),
        dominantVal.begin(), dominantVal.end());
    _colorFeatureVector.insert(_colorFeatureVector.end(),
        huedft.begin(), huedft.end());
    _colorFeatureVector.insert(_colorFeatureVector.end(),
        satdft.begin(), satdft.end());
    _colorFeatureVector.insert(_colorFeatureVector.end(),
        colorAnglesAndStd.begin(), colorAnglesAndStd.end());
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
* @brief Function that cleans up colorFeatureVector, to add
* new elements for next frame
* @return void
*/
  void ChannelsStatisticsExtractor::emptyCurrentFrameFeatureVector()
  {
    meanStdHSV.clear();
    dominantVal.clear();
    huedft.clear();
    satdft.clear();
    colorAnglesAndStd.clear();
    _colorFeatureVector.clear();
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
}// namespace pandora_vision
