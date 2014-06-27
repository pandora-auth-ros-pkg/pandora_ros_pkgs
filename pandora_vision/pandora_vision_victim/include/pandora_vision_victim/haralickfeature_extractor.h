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
#ifndef PANDORA_VISION_VICTIM_HARALICKFEATURE_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_HARALICKFEATURE_EXTRACTOR_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"

namespace pandora_vision
{
  class HaralickFeaturesExtractor
  {
    
    private:
    
    //!<Vector containing 13 Haralick Features
    std::vector<double> _haralickFeatures;
    
    public:
    
    //!<Constructor
    HaralickFeaturesExtractor();
    
    /**
    @brief Function for calculating the normalized GLCM matrix, considering an
    horizontal relationship between pixels with offset equal to 1. The returned 
    matrix is symmetric with respect to the main diagonal.
    @param in [cv::Mat&] A grayscale image with 8 bit values.
    @return [cv::Mat] The GLCM matrix.
    **/
    cv::Mat calculateGLCM(const cv::Mat& in);
    
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
    void updateGLCM(int y1, int x1, int y2, int x2, 
        cv::Mat* out, const cv::Mat& in);
    
    /**
    @brief Function for normalizing the values of the GLCM matrix.
    @param in [cv::Mat*] The matrix to be normalized
    @return [cv::Mat] The normalized matrix
    **/  
    cv::Mat normalizeGLCM(cv::Mat* in);
    
    /**
    @brief Function for calculating the Angular Second Moment,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/    
    void getAngularSecondMoment(const cv::Mat& in);
    
    /**
    @brief Function for calculating entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/      
    void getEntropy(const cv::Mat& in);
    
    /**
    @brief Function for calculating contrast,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/      
    void getContrast(const cv::Mat& in);
    
    /**
    @brief Function for calculating variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/        
    void getVariance(const cv::Mat& in);
    
    /**
    @brief Function for calculating correlation,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getCorrelation(const cv::Mat& in);
    
    /**
    @brief Function for calculating homogeneity,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getHomogeneity(const cv::Mat& in);
    
    /**
    @brief Function for calculating sum average,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getSumAverage(const cv::Mat& in);
    
    /**
    @brief Function for calculating sum variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getSumVariance(const cv::Mat& in);
    
    /**
    @brief Function for calculating sum entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getSumEntropy(const cv::Mat& in);
    
    /**
    @brief Function for calculating difference variance,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getDifferenceVariance(const cv::Mat& in);
    
    /**
    @brief Function for calculating difference entropy,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getDifferenceEntropy(const cv::Mat& in);
    
    /**
    @brief Function for calculating Info Measure of Correlation 1 and 2,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @param in [cv::Mat&] The normalized GLCM matrix
    @return void
    **/          
    void getInfoMeasuresCorr(const cv::Mat& in);
    
    /**
    @brief Function returning the haralick feature vector,
    http://murphylab.web.cmu.edu/publications/boland/boland_node26.html  
    @return std::vector [float] The vector containing the haralick features
    **/          
    std::vector<double> getFeatures();
    
    /**
    @brief This is the main function called to extract haralick features
    @param image [cv::Mat] The current frame to be processed
    @param void return
    */ 
    void findHaralickFeatures(cv::Mat image);
    
    /**
    @brief Function that cleans up haralickFeatureVector, to add
    new elements for next frame
    @return void
    */ 
    void emptyCurrentFrameFeatureVector();
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_HARALICKFEATURE_EXTRACTOR_H
