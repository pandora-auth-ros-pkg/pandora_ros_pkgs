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
* Author: Victor Daropoulos
*********************************************************************/


#include "utils/haralickfeature_extractor.h"

  //!<Constructor
  HaralickFeaturesExtractor:: HaralickFeaturesExtractor()
  {
  }

  /**
    @brief Function for updating the values of the GLCM matrix.
    @param y1 [int] The y coordinate of the first pixel
    @param x1 [int] The x coordinate of the first pixel
    @param y2 [int] The y coordinate of the second pixel
    @param x2 [int] The x coordinate of the second pixel
    @param out [cv::Mat*] The updated GLCM matrix
    @param in [cv::Mat&] The GLCM matrix
    @return void
  **/  
  void HaralickFeaturesExtractor::updateGLCM(int y1, int x1, int y2, int x2, 
      cv::Mat* out, const cv::Mat& in)
  {
    if(y2<0 || y2 >= in.rows || x2<0 || x2 >= in.cols) return;
    
    out->at<double>(in.at<uchar>(y1, x1), in.at<uchar>(y2, x2))+=1.0;
    
  }

  /**
    @brief Function for calculating the normalized GLCM matrix, considering an
    horizontal relationship between pixels with offset equal to 1. The returned 
    matrix is symmetric with respect to the main diagonal.
    @param in [cv::Mat&] A grayscale image with 8 bit values.
    @return [cv::Mat] The GLCM matrix.
  **/
  cv::Mat HaralickFeaturesExtractor::calculateGLCM(const cv::Mat& in)
  {
    cv::Mat out = cv::Mat::zeros(256, 256, CV_64FC1);
    
    cv::Mat temp;
    
    int offset = 1;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        updateGLCM(y, x, y+offset, x, &out, in);
      }
    }
    
    cv::transpose(out, temp);
    
    out = out+temp;  
    
    normalizeGLCM(&out);
    
    return out;
  }

  /**
    @brief Function for normalizing the values of the GLCM matrix.
    @param in [cv::Mat*] The matrix to be normalized
    @return [cv::Mat] The normalized matrix
  **/  
  cv::Mat HaralickFeaturesExtractor::normalizeGLCM(cv::Mat* in)
  {
    cv::Scalar totalSum = cv::sum(*in);
    
    for(int y = 0; y < in->rows; y++)
    {
      for(int x = 0; x < in->cols; x++)
      {
        in->at<double>(y, x) =
        static_cast<double>(in->at<double>(y, x)/static_cast<double>(totalSum[0]));
      }
    }
    return *in;
  }

  /**
    @brief Function for calculating the Angular Second Moment,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/    
  void HaralickFeaturesExtractor::getAngularSecondMoment(const cv::Mat& in)
  {
    double sum = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        sum+=in.at<double>(y, x)*in.at<double>(y, x);
      }
    }
    
    _haralickFeatures.push_back(sum);
  }


  /**
    @brief Function for calculating entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/      
  void HaralickFeaturesExtractor::getEntropy(const cv::Mat& in)
  {
    double sum = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        if(in.at<double>(y, x) != 0)
        {
          sum+=in.at<double>(y, x)*log(in.at<double>(y, x));
        }
      }
    }
    
    sum = -sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating contrast,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/      
  void HaralickFeaturesExtractor::getContrast(const cv::Mat& in)
  {
    double sum= 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        sum+=pow(static_cast<double>(y-x), 2.0)*in.at<double>(y, x);
      }
    }
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/
  void HaralickFeaturesExtractor::getVariance(const cv::Mat& in)
  {
    double w_mean = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        w_mean+=y*in.at<double>(y, x);
      }
    }
    
    double variance = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        variance+=pow(static_cast<double>(y-w_mean), 2.0)*in.at<double>(y, x);
      }
    }
    
    _haralickFeatures.push_back(variance);
  }


  /**
    @brief Function for calculating correlation,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/
  void HaralickFeaturesExtractor::getCorrelation(const cv::Mat& in)
  {
    double mean = 0;
    double std = 0;
    double corr = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        mean+=y*in.at<double>(y, x);
      }
    }
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        std+=pow(static_cast<double>(y-mean), 2.0)*in.at<double>(y, x);
      }
    }
    
    std = sqrt(std);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        corr+=(in.at<double>(y, x)*(y-mean)*(x-mean))/pow(std, 2.0);
      }
    }
    
    _haralickFeatures.push_back(corr);
  } 

  /**
    @brief Function for calculating homogeneity,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/
  void HaralickFeaturesExtractor::getHomogeneity(const cv::Mat& in)
  {
    double temp = 0;
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp+=(1./(1.+pow(static_cast<double>(y-x), 2)))*in.at<double>(y, x);
      }
    }
    
    _haralickFeatures.push_back(temp);
  }    

  /**
    @brief Function for calculating sum average,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/
  void HaralickFeaturesExtractor::getSumAverage(const cv::Mat& in)
  {
    cv::Mat temp = cv::Mat::zeros(1, 2*(in.rows-1)+1, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp.at<double>(x+y)+=in.at<double>(y, x);
      }
    }
    
    for(int i = 0; i < temp.cols; i++)
    {
      temp.at<double>(i) = i*temp.at<double>(i);
    }
    
    cv::Scalar totalSum = cv::sum(temp);
    
    _haralickFeatures.push_back(totalSum[0]); 
    
    
  }  

  /**
    @brief Function for calculating sum variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/  
  void HaralickFeaturesExtractor::getSumVariance(const cv::Mat& in)
  {
    cv::Mat temp = cv::Mat::zeros(1, 2*(in.rows-1)+1, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp.at<double>(x+y)+=in.at<double>(y, x);
      }
    }
    
    double f7 = _haralickFeatures[6];
    
    float sum = 0;
    
    for(int i = 0; i < temp.cols; i++)
    {
      sum+=pow(static_cast<double>(i-f7), 2.0)*temp.at<double>(i);
    }
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating sum entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/  
  void HaralickFeaturesExtractor::getSumEntropy(const cv::Mat& in)
  {
    cv::Mat temp = cv::Mat::zeros(1, 2*(in.rows-1)+1, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp.at<double>(x+y)+=in.at<double>(y, x);
      }
    }
    
    double sum = 0;
    
    for(int i = 0; i < temp.cols; i++)
    {
      if(temp.at<double>(i) != 0)
      {
        sum+=temp.at<double>(i)*log(temp.at<double>(i));
      }
    }
    
    sum=-sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating difference variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/   
  void HaralickFeaturesExtractor::getDifferenceVariance(const cv::Mat& in)
  {
    cv::Mat temp = cv::Mat::zeros(1, in.rows, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp.at<double>(abs(y-x))+=in.at<double>(y, x);
      }
    }
    
    double f10_ = 0;
    
    for(int i = 0; i < temp.cols; i++)
    {
      f10_+=i*temp.at<double>(i);
    }
    
    double f10 = 0;
    
    for(int i = 0; i < temp.cols; i++)
    {
      f10+=pow(static_cast<double>(i-f10_), 2.0)*temp.at<double>(i);
    }
    
    _haralickFeatures.push_back(f10);
  }

  /**
    @brief Function for calculating difference entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/   
  void HaralickFeaturesExtractor::getDifferenceEntropy(const cv::Mat& in)
  {
    cv::Mat temp = cv::Mat::zeros(1, in.rows, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        temp.at<double>(abs(y-x))+=in.at<double>(y, x);
      }
    }
    
    double sum = 0;
    
    for(int i = 0; i < temp.cols; i++)
    {
      if(temp.at<double>(i) != 0)
      {
        sum+=temp.at<double>(i)*log(temp.at<double>(i));
      }
    }
    
    sum = -sum;
    
    _haralickFeatures.push_back(sum);
  }

  /**
    @brief Function for calculating Info Measure of Correlation 1 and 2,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
  **/
  void HaralickFeaturesExtractor::getInfoMeasuresCorr(const cv::Mat& in)
  {
    cv::Mat px = cv::Mat::zeros(1, in.rows, CV_64FC1);
    
    cv::Mat py = cv::Mat::zeros(1, in.rows, CV_64FC1);
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        px.at<double>(y)+=in.at<double>(y, x);
        py.at<double>(y)+=in.at<double>(x, y);
      }
    }
    
    double HX = 0;
    double HY = 0;
     
    for(int i = 0; i < px.cols; i++)
    {
      if(px.at<double>(i) != 0)
      {
        HX+=px.at<double>(i)*log(px.at<double>(i));
      }
      if(py.at<double>(i) != 0)
      {
        HY+=py.at<double>(i)*log(py.at<double>(i));
      }
    }
    
    HX = -HX;
    HY = -HY;
    
    
    double HXY1 = 0;
    double HXY2 =0;
    double HXY = 0; 
    
    for(int y = 0; y < in.rows; y++)
    {
      for(int x = 0; x < in.cols; x++)
      {
        if((px.at<double>(y)*py.at<double>(x)) != 0)
        {
          HXY1+=in.at<double>(y, x)*log(px.at<double>(y)*py.at<double>(x));
          HXY2+=px.at<double>(y)*py.at<double>(x)*log(px.at<double>(y)*py.at<double>(x));
        }
        if(in.at<double>(y, x) != 0)
        {
          HXY+=in.at<double>(y, x)*log(in.at<double>(y, x));
        }
      }
    }
    
    HXY1 = -HXY1;
    HXY2 = -HXY2;
    HXY = -HXY;
    
    double f12 = (HXY-HXY1)/std::max(HX, HY);
    
    _haralickFeatures.push_back(f12);
    
    double f13 = sqrt(1-exp(-2*(HXY2-HXY)));
    
    _haralickFeatures.push_back(f13);
    
  }

  /**
    @brief Function returning the haralick feature vector,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @return std::vector [float] The vector containing the haralick features
  **/     
  std::vector<double> HaralickFeaturesExtractor::getFeatures()
  {
    //~ for(int i = 0; i < _haralickFeatures.size(); i++)
      //~ ROS_INFO_STREAM(" "<< _haralickFeatures.at(i));
    return _haralickFeatures;
  }
  
  /**
    @brief Function that cleans up haralickFeatureVector, to add
    new elements for next frame
    @return void
  */ 
  void HaralickFeaturesExtractor::emptyCurrentFrameFeatureVector()
  {
    _haralickFeatures.clear();
  }
    
  /**
    @brief This is the main function called to extract haralick features
    @param image [cv::Mat] The current frame to be processed
    @param void return
  */ 
  void HaralickFeaturesExtractor::findHaralickFeatures(cv::Mat image)
  {
    cv::Mat temp = cv::Mat(image.rows, image.cols, CV_8UC1);
     if(image.channels() !=1)
      cv::cvtColor(image, temp, CV_BGR2GRAY);
    cv::Mat out = calculateGLCM(temp);
    
    //getAngularSecondMoment(out);
    //getContrast(out);
    //getEntropy(out);
    //getVariance(out);
    //getCorrelation(out);
    getHomogeneity(out);
    //getSumAverage(out);
    //getSumVariance(out);
    //getSumEntropy(out);
    //getDifferenceVariance(out);
    //getDifferenceEntropy(out);
    //getInfoMeasuresCorr(out);
  }
  
  

