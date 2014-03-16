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
* Author:  Skartados Evangelos
*********************************************************************/

#ifndef PANDORA_VISION_FACE_SKIN_DETECTOR_H
#define PANDORA_VISION_FACE_SKIN_DETECTOR_H

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
namespace pandora_vision
{
class SkinDetector
{

public:

  cv::Mat imgSrc;
  cv::Mat imgHistogrammSkin;
  cv::Mat imgHistogrammWall;
  cv::Mat imgHistogrammWall2;
  cv::Mat imgThreshold;
  cv::Mat imgThresholdFiltered;
  cv::Mat imgContours;

  std::vector< std::vector <cv::Point> > Contour;

  cv::Point* contourCenter;

  int contourCounter;

  float* contourProbability;
  int* contourSize;

  int sizeThreshold;

  float imageHeight;
  float imageWidth;

  //!< Constructor
  SkinDetector(std::string skinHist, std::string wallHist, std::string wall2Hist );

  //!< Destructor
  virtual ~SkinDetector();

  /**
    @brief Function used for the communication between SkinDetector
    and FaceDetector insatnce
    @return imgContours [cv::IplImage] contains the areas
    which have been detected as skin blobs.
  */
  cv::Mat getImgContoursForFace();

  /**
    @brief Function that initializes all the pointers used
    for the detection of skin blobs
    @return void
  */
  void init();


  int detectSkin(cv::Mat imgInput);

  /**
    @brief Function that releases all the pointers
    after the detection of skin blobs is done
    @return void
  */
  void deallocateMemory();

  /**
    @brief Function that given the already loaded histogramm images
    decided which areas of the current image are skin blobs
    @return imgInput [cv::IplImage] the current image
    that must be checked for skin areas.
  */
  void createCalculationImages(cv::Mat imgInput);

  /**
    @brief Function that checks if histogramms are loaded correctly
    @return [bool], true if all patterns are loaded correctly,
    false if not
  */
  bool histogrammsLoaded();

  /**
    @brief Function that calculates all necessary parameters
    for skin detection
    @return void
  */
  void getCalculationParams(int *stepSrc, uchar *dataSrc , int *channels,
      int *stepThreshold, uchar *dataHistogrammSkin, int *stepHistogrammSkin,
      uchar *dataHistogrammWall, int *stepHistogrammWall, 
      uchar *dataHistogrammWall2, int *stepHistogrammWall2, 
      uchar *dataContours, int *stepContours);

  /**
    @brief Function that scans given frame and checks if skin is detected
    @return void
  */
  void scanForSkin(int stepSrc, uchar* dataSrc , int channels,
        int stepThreshold, uchar* dataHistogrammSkin, int stepHistogrammSkin,
        uchar* dataHistogrammWall, int stepHistogrammWall, 
        uchar* dataHistogrammWall2, int stepHistogrammWall2);

  /**
    @brief Function that calculates probability to have skin
    in current frame
    @return void
  */
  void calculatePropability(uchar *dataContours, int stepContours);
};
}// namespace pandora_vision
#endif  // PANDORA_VISION_FACE_SKIN_DETECTOR_H


