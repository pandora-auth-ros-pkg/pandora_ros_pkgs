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

#ifndef PATTERN_H
#define PATTERN_H

#include <stdint.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"

#define POSITIVE 0
#define NEGATIVE 1

#define	BGR						0
#define YCrCb					1
#define CrCb					2

#define EUCLIDEAN				0
#define EUCLIDEAN_SIMPLE		1

using namespace cv;
namespace pandora_vision
{
  struct histogram 
  {
    int* values;
    double* valuesNorm;
    int bins;
    double norm2;	// norm ^ 2
  };

  class Pattern
  {
    
    private:
          
                                        
    public:
      
      uintptr_t m_id;
      
      histogram m_hist;
    
      IplImage* m_imgPatternGray;
      IplImage* m_imgPatternBGR;
      IplImage* m_imgPatternYCrCb;
      
      bool isPositive;
    
      Pattern();		
      ~Pattern();
      
      void createPattern(IplImage* img, int type);
      void destroyPattern();
      void calculateHistogram(CvPoint3D32f* colors3d, int bins, int colorspace);
      void calculateHistogram(Mat_<Vec2f>* colors2d, int bins, int colorspace);
      void visualizeHistogram(CvPoint3D32f* colors3d);
      void visualizeHistogram(Mat_<Vec2f>* colors2d);
      
      double calculatePointDistance(CvPoint3D32f pt1, CvPoint3D32f pt2, int type);
      double calculatePointDistance(CvPoint2D32f pt1, CvPoint2D32f pt2, int type);
      
      CvScalar BGR2YCrCb(CvScalar bgr);
      CvScalar YCrCb2BGR(CvScalar ycrcb);
      
  };
}
#endif
