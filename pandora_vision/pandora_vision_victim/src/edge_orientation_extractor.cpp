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

#include "pandora_vision_victim/edge_orientation_extractor.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  EdgeOrientationExtractor::EdgeOrientationExtractor()
  {
   
   ROS_INFO("[victim_node] : Created Edge Orientation detection instance"); 
   
  }
  
  /**
    @brief Destructor
  */
  EdgeOrientationExtractor::~EdgeOrientationExtractor()
  {
    ROS_INFO("[victim_node] : Destroying Edge Orientation detection instance");
  }
  
  /**
   * @brief This is the main function which calls all the others and 
   * computes the final edge histogram features.
   * @param src [cv::Mat] the current image.
  */ 
  
  void EdgeOrientationExtractor::findEdgeFeatures(cv::Mat src)
  {
    GaussianBlur( src, src, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT );
    if(src.channels() !=1)
      cvtColor( src, src, CV_BGR2GRAY );
    
    //!<block size
    int colsBlockSize = static_cast<int>(src.cols/4) + 1;
    int rowsBlockSize = static_cast<int>(src.rows/4) + 1;
    cv::resize(src, src, cv::Size(
      static_cast<int>(src.cols/4 + 1) * 4,
      static_cast<int>(src.rows/4 + 1) * 4)
    );
    if(src.rows % 4 != 0 && src.cols % 4 != 0)
      ROS_ERROR("[victim_node] : Incorrect Division");
      
    edgeFeatures = partition(src, colsBlockSize, rowsBlockSize);
    
    //~ ROS_INFO_STREAM("EdgeFeatures= ");
    //~ for (int ii = 0; ii < edgeFeatures.size(); ii++)
       //~ ROS_INFO_STREAM( " " << edgeFeatures[ii]);
  
  }
  
  /**
   *@brief This is the function which divides the image 
   *into 16 subblocks and calls the findLocalEdgeFeatures to compute the 
   *features of the current subblock
   * @param currFrame [cv::Mat] the current subblock.
   * @param colsBlockSize [int] the cols size of the subblock.
   * @param rowsBlockSize [int] the rows size of the subblock.
   * @return [std::vector<double>] the computed  1x80 edgeFeatures vector.
  */ 
  
  std::vector<double> EdgeOrientationExtractor::partition(cv::Mat currFrame, 
        int colsBlockSize, int rowsBlockSize )
  {   
    
    std::vector<double> temp(5);
    std::vector<double> edgeFeatures;
    
    for(int ii = 0; ii < currFrame.cols; ii += colsBlockSize )
    { 
        for( int jj = 0; jj < currFrame.rows; jj += rowsBlockSize )
        {
            cv::Mat subblock( currFrame, cv::Rect( ii, jj, colsBlockSize, 
                            rowsBlockSize ) );
            
      
            //!< Do stuff with subblock here
            temp = findLocalEdgeFeatures(subblock);
            for(int kk = 0; kk < 5; kk++)
              edgeFeatures.push_back(temp[kk]);
        }
    }
    return edgeFeatures;   
  }
    
  /**
   * @brief This is the function which  computes the edge histogram 
   *  feature of the current subblock.
   * @param currFrame [cv::Mat] the current subblock.
   * @return [std::vector<double>] the computed 1x5 edgeFeatures vector.
  */ 
  
  std::vector<double> EdgeOrientationExtractor::findLocalEdgeFeatures(cv::Mat 
                                                                  currFrame)
{
  
  cv::Mat img;
  currFrame.convertTo(img, CV_64F);
 
  //!< Build a vector of the same size of the image and 5 dimensions  
  //!<to save the gradients
  std::vector<cv::Mat> convImg(5);
 
  
  //!< Define the Scharr filters for the 5 types of edges
  std::vector<cv::Mat> kernel(5);
  /*double vals[5][9] = {1, 2, 1, 0, 0, 0, -1, -2, -1, -1, 0, 1, -2, 0, 2, -1, 
                  0, 1, 2, 2, -1, 2, -1, -1, -1, -1, -1, -1, 2, 2, -1, -1, 2, 
                  -1, -1, -1, -1, 0, 1, 0, 0, 0, 1, 0, -1}; */
  double vals[5][9] = {-3, -10, -3, 0, 0, 0, 3, 10, 3, -3, 0, 3, -10, 0, 10, -3,
                  0, 3, 0, 3, 10, -3, 0, 3, -10, -3, 0, -10, -3, 0, -3, 
                  0, 3, 0, 3, 10, -3, 0, 3, 0, 0, 0, 3, 0, -3};

  for (int ii = 0; ii < kernel.size(); ii++)
    kernel[ii] = cv::Mat(3, 3, CV_64F, vals[ii]);

  //!<iterate over the posible directions and apply the filters
 
  for (int ii = 0; ii < kernel.size(); ii++)
  {
    conv2(img, kernel[ii], CONVOLUTION_SAME, &convImg[ii]);
    convImg[ii]=abs(convImg[ii]);
  }
    
  //!< Calculate the max sobel gradient and save the type of the orientation
  double maxVal;
  cv::Mat maxGrad = cv::Mat::zeros(convImg[0].rows, convImg[0].cols, CV_64F);
  for(int jj = 0; jj < convImg[0].rows; jj++)
    for(int kk = 0; kk < convImg[0].cols; kk++)
    {
      double maxVal= - std::numeric_limits<double>::infinity();
      for(int ii = 0; ii < convImg.size(); ii++)
      {
        if(maxVal < convImg[ii].at<double>(jj, kk))
        { maxVal = convImg[ii].at<double>(jj, kk);
          maxGrad.at<double>(jj, kk) = ii+1;
        }
      }
    }
    
  //!<Detect the edges
  cv::Mat edges;
  Canny(currFrame, edges, 0, 30, 3);
  edges.convertTo(edges, CV_64F);
  for (int ii = 0; ii < edges.rows; ii++)
    for(int jj = 0; jj < edges.cols; jj++)
        if(edges.at<double>(ii, jj) == 255)
          edges.at<double>(ii, jj)=1;
  

  
  //!<multiply against the types of orientations detected by the Sobel masks
  cv::Mat data = cv::Mat::zeros(convImg[0].rows, convImg[0].cols, CV_64F);

    for(int jj = 0; jj < data.rows; jj++)
      for(int kk = 0; kk < data.cols; kk++)
        data.at<double>(jj, kk) = maxGrad.at<double>(jj, kk) * 
                                  edges.at<double>(jj, kk);
                                  
  data.convertTo(data, CV_32F);

  //!< Establish the number of bins
  int bins = 6; 
  
  //!< Set the range 
  float range[] = { 0, 6 };

  const float* histRange = { range };

  bool uniform = true; 
  bool accumulate = false;
  
  cv::Mat hist;
  cv::calcHist( &data, 1, 0, cv::Mat(), hist, 1, &bins, &histRange, uniform, 
                accumulate );
  
  show_histogramm(bins, hist, "Edge Histogramm");
  
  //!< save the final edgeFeatures for the 5 types of oriented gradients
  std::vector<double> edgeFeatures(5);
  
  for(int ii = 1; ii < 6; ii++)
  {
     edgeFeatures[ii-1]=hist.at<float>(ii) / (data.rows *data.cols);
  }
    
  return edgeFeatures;
}

  
    /**
     * @brief This function applies the sobel filters on the src image.  
     * @param img [const cv::Mat&] the src img.
     * @param kernel [const cv::Mat&] the sobel kernel filter.
     * @param type [ConvolutionType] the type of convolution.
     * @return dest [cv::Mat&] the convoluted image.
    */ 
  void EdgeOrientationExtractor::conv2(const cv::Mat &img, 
            const cv::Mat& kernel, ConvolutionType type, cv::Mat* dest)
  {
    cv::Mat source = img;
    cv::Mat dest2;
    if(CONVOLUTION_FULL == type) {
      source = cv::Mat();
      const int additionalRows = kernel.rows - 1, additionalCols = kernel.cols- 1;
      copyMakeBorder(img, source, (additionalRows + 1) / 2, additionalRows / 2, 
                    (additionalCols + 1) / 2, additionalCols / 2, 
                    cv::BORDER_CONSTANT, cv::Scalar(0) );
    }
   
    //cv::Point anchor(kernel.cols - kernel.cols / 2 - 1, 
    //                 kernel.rows - kernel.rows / 2 - 1);
    cv::Point anchor(-1, -1);
    int borderMode = cv::BORDER_CONSTANT;
    //cv::flip(kernel,kernel, -1);
    filter2D(source, dest2, img.depth(), kernel, anchor, 0, borderMode);
   
    if(CONVOLUTION_VALID == type) {
      dest2 = dest2.colRange((kernel.cols - 1) / 2, dest2.cols - kernel.cols / 2)
                 .rowRange((kernel.rows - 1) / 2, dest2.rows - kernel.rows / 2);
    }
    *dest = dest2;
  }


  /**
     * @brief This function displays the calculated histogram to the screen.
     * @param bins [int] the number of bins.
     * @param hist [cv::Mat] the calculated histogram.
     * @param colorComp [const char*] the name of the window.
  */ 
  void EdgeOrientationExtractor::show_histogramm(int bins, cv::Mat hist, 
                                               const char* colorComp)
  { 
    int w = 400; int h = 400;
    double max_val = 0;
    minMaxLoc(hist, 0, &max_val);
    int bin_w = cvRound( static_cast<double> (w / bins ));
    cv::Mat histImg = cv::Mat::zeros( w, h, CV_8UC3 );
    
    //!< visualize each bin
    for(int ii = 0; ii < bins; ii++) 
    {
      float const binVal = hist.at<float>(ii);
      int   const height = cvRound(binVal * h / max_val);
      rectangle( histImg, cv::Point( ii*bin_w, h ), 
                cv::Point( (ii+1)*bin_w, h-height ), 
                cv::Scalar( 112, 255, 112 ), -1 ); 

    }
    #ifdef SHOW_DEBUG_IMAGE
      imshow( colorComp, histImg );
    #endif    
  }
  
  std::vector<double> EdgeOrientationExtractor::getFeatures()
  {
    return edgeFeatures;
  }
  
  /**
   * @brief Function that cleans up EdgeFeatureVector, to add
   * new elements for next frame
   * @return void
  */ 
  void EdgeOrientationExtractor::emptyCurrentFrameFeatureVector()
  {
    edgeFeatures.clear();
  }
}// namespace pandora_vision
