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
#ifndef PANDORA_VISION_VICTIM_EDGE_ORIENTATION_EXTRACTOR_H
#define PANDORA_VISION_VICTIM_EDGE_ORIENTATION_EXTRACTOR_H

#include "pandora_vision_victim/victim_parameters.h"

//~ #define SHOW_DEBUG_IMAGE

namespace pandora_vision
{
  
  enum ConvolutionType
{
     
  //!<Return the full convolution, including border
  CONVOLUTION_FULL, 
  
  //!<Return only the part that corresponds to the original image 
  CONVOLUTION_SAME,
  
  //!<Return only the submatrix containing elements that were not influenced 
  //!<by the border 
  CONVOLUTION_VALID
  
};
  class EdgeOrientationExtractor
  {
    /// Vector containing features according to edge orientation
    std::vector<double> edgeFeatures;
    
    /**
     *@brief This is the function which divides the image 
     *into 16 subblocks and calls the findLocalEdgeFeatures to compute the 
     *features of the current subblock
     * @param currFrame [cv::Mat] the current subblock.
     * @param colsBlockSize [int] the cols size of the subblock.
     * @param rowsBlockSize [int] the rows size of the subblock.
     * @return [std::vector<double>] the computed  80 edgeFeatures vector.
    */ 
    std::vector<double> partition(cv::Mat currFrame, int colsBlockSize, 
                                  int rowsBlockSize );

    /**
     * @brief This is the function which  computes the edge histogram 
     *  feature of the current subblock.
     * @param currFrame [cv::Mat] the current subblock.
     * @return [std::vector<double>] the computed 1x5 edgeFeatures vector
    */ 
    std::vector<double> findLocalEdgeFeatures(cv::Mat currFrame);
    
    /**
     * @brief This function applies the sobel filters on the src image.  
     * @param img [const cv::Mat&] the src img.
     * @param kernel [const cv::Mat&] the sobel kernel filter.
     * @param type [ConvolutionType] the type of convolution.
     * @return dest [cv::Mat&] the convoluted image.
    */ 
    void conv2(const cv::Mat &img, const cv::Mat& kernel, ConvolutionType type, 
          cv::Mat* dest);

    
    /**
     * @brief This function displays the calculated histogram to the screen.
     * @param bins [int] the number of bins.
     * @param hist [cv::Mat] the calculated histogram.
     * @param colorComp [const char*] the name of the window.
    */ 
    void show_histogramm (int bins, cv::Mat hist, const char* colorComp);
    public:
    
    //!Constructor
    EdgeOrientationExtractor();
    
    //!Destructor
    ~EdgeOrientationExtractor();
    
    /**
     * @brief This is the main function which calls all the others and 
     * computes the final edge histogram features.
     * @param src [cv::Mat] the current image.
    */ 
    void findEdgeFeatures(cv::Mat src);
    
    std::vector<double> getFeatures();
    
    /**
     * @brief Function that cleans up EdgeFeatureVector, to add
     * new elements for next frame
     * @return void
    */ 
    void emptyCurrentFrameFeatureVector();
  };
  
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_EDGE_ORIENTATION_EXTRACTOR_H
