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
* Author: Michael Skolarikis
*********************************************************************/

#ifndef HOLEFINDER_H
#define HOLEFINDER_H
#define HOLEFINDER_DEBUG_MODE false

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include "thing.h"
#include "texture_filter.h"
#include "pattern.h"

#define DEFAULT_HEIGHT 480
#define DEFAULT_WIDTH 640

namespace pandora_vision
{
  class HoleFinder
  { 	
    
    private:
      bool convertToGray;
    
    public:
    
      /// Parameters
      
      //Modify Source Image Params
      int _modeA;
      int _equalizeA;
      int _smoothA;
      int _erodeA;
      int _dilateA;
      int _openA;
      int _closeA;
      
      //Edge Image Params
      int _modeB;
      
      //	mode 0 params
      int _cannyConvKernelB;
      int _cannyLowThresB;
      int _cannyHighThresB;
      int _dilateB;
      int _cannyConvKernelB1;
      int _cannyLowThresB1;
      int _cannyHighThresB1;
      
      //	mode 1 params
      int _gradientB;
      int _equalizeB;
      int _thresholdLowThresB;
      int _thresholdHighThresB;
      int _dilateB1;
      int _cannyConvKernelB2;
      int _cannyLowThresB2;
      int _cannyHighThresB2;
      
      //Threshold Image Params
      int _closeC;
      int _openC;
      int _erodeC;
      int _dilateC;
      int _thresholdLowThresC;
      int _thresholdHighThresC;
      
      //Texture Image Params
      int _smoothD;
      int _closeD;
      int _openD;
      int _erodeD;
      int _dilateD;
      int _thresholdLowThresD;
      int _thresholdHighThresD;
      
      //Contour Parameters
      int _lengthContour;
      int _areaContour;
      double _maxAreaContour;
      int _rectHeightContour;
      int _rectWidthContour;
      double _heightToWidthContour;
      double _widthToHeightContour;
      double _pixelToRectAreaContour;
      double _darkToPixelAreaContour;
      double _textureToPixelAreaContour;
      
      //Blob Parameters
      double _minFormFactor;
      double _ellipseFactorMax;
      double _ellipseFactorMin;
      double _minAxisValue;
      double _verticalAxisRatio;
      double _horizontalAxisRatio;
      double _thetaLow;
      double _thetaHigh;

      /// ------------------- ///
      
      TextureFilter* filter;
    
      CvSize size;
      cv::Mat imgSrc;
      IplImage* imgEdge;
      IplImage* imgContours;
      IplImage* imgThreshold;
      IplImage* imgTexture;
      
      CvMemStorage* strgContours;
      CvSeq* seqContours;
      CvSeq* seqCurrent;
      
      vector<Thing*> resultBlobs;
    
      int contourCounter;
      
      HoleFinder();
      ~HoleFinder();
            
      void modifySourceImage(cv::Mat capturedImage);
      void findThresholdImage(cv::Mat imgInput);
      void findEdgeImage(IplImage* imgInput);
      void findTextureImage(cv::Mat capturedImage);
      void findContours(IplImage* edgeImage);
      void findBlobs(IplImage* contourImage);
      vector<Thing*> findHoles(cv::Mat sourceImage);
      
      void createWindows();
      void showImages();
      void destroyWindows();
      
      void cleanup();
      
      void setModeMod(int mode);
      void setEqualizeMod(int flag);
      void setSmoothMod(int block);
      void setDilateMod(int iterations);
      void setErodeMod(int iterations);
      void setOpenMod(int iterations);
      void setCloseMod(int iterations); 

      void setModeEdge(int mode);
      void setCannyThresEdge(int threshold1 , int threshold2);
      void setDilateEdge(int iterations);
      void setGradientEdge(int iterations);
      void setEqualizeEdge(int flag);
      void setThresholdEdge(int threshold1, int threshold2); 

      void setErodeThres(int iterations);
      void setDilateThres(int iterations);
      void setCloseThres(int iterations);
      void setOpenThres(int flag);
      void setThresholdThres(int threshold1, int threshold2); 
      
      void setMinLengthContour(int length);
      void setMinAreaContour(int area);
      void setMinRectHeightContour(int height);
      void setMinRectWidthContour(int width);
      void setHeightToWidthContour(double ratio);
      void setWidthToHeightContour(double ratio);
      void setPixelToRectAreaContour(double ratio);
      void setDarkToPixelAreaContour(double ratio);
      
      void setTexturePath(string path);
  };
}
#endif
