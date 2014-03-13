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

#include "pandora_vision_hole/pattern.h"

/*
 * * File Description :
 *	This is a class for patterns, used from the
 *  color/texture segmentation algorithm.
 * 
 */ 
namespace pandora_vision
{

  /**
   * Constructor
   */
  Pattern::Pattern()
  {		
    this->m_id = (uintptr_t)this;
    
    this->isPositive = NULL;
    this->m_imgPatternGray = NULL;
    this->m_imgPatternBGR = NULL;
    this->m_imgPatternYCrCb = NULL;
    
    this->m_hist.values = NULL;
    this->m_hist.valuesNorm = NULL;
    this->m_hist.bins = 0;
  }

  /**
   * Destructor
   */
  Pattern::~Pattern()
  {	
    //cout << "Pattern's destructor called" << endl;
  }

  /**
   * Changes a pixel(scalar) from the colorspace BGR to the colorspace YCrCb
   * @param bgr
   * @return
   */
  CvScalar Pattern::BGR2YCrCb(CvScalar bgr)
  {
      CvScalar ycrcb;
      float r = (float)bgr.val[2];
      float g = (float)bgr.val[1];
      float b = (float)bgr.val[0];

      ycrcb.val[0] = 0.299*r + 0.587*g + 0.114*b;
      ycrcb.val[1] = (r-ycrcb.val[0])*0.713 + 128.0;
      ycrcb.val[2] = (b-ycrcb.val[0])*0.564 + 128.0;

      return ycrcb;
  }

  /**
   * Changes a pixel(scalar) from the colorspace YCrCb to the colorspace BGR.
   * @param ycrcb
   * @return
   */
  CvScalar Pattern::YCrCb2BGR(CvScalar ycrcb)
  {
      CvScalar bgr;
      float y = (float)ycrcb.val[0];
      float cr = (float)ycrcb.val[1];
      float cb = (float)ycrcb.val[2];

      bgr.val[2] = y + 1.403*(cr - 128.0);
      bgr.val[1] = y - 0.344*(cr - 128.0) - 0.714*(cb - 128.0);
      bgr.val[0] = y + 1.773*(cb - 128.0);

      return bgr;
  }

  /**
   * Creates different styles of paterns.
   * @param img
   * @param type
   */
  void Pattern::createPattern(IplImage* img, int type)
  {	
    if (img->nChannels == 1)
    {
      this->m_imgPatternGray = (IplImage*)cvClone(img);
    }
    else //channels == 3
    {
      this->m_imgPatternBGR = (IplImage*)cvClone(img);
      
      this->m_imgPatternYCrCb = cvCreateImage( cvGetSize(m_imgPatternBGR) , IPL_DEPTH_8U , 3);
      cvCvtColor(this->m_imgPatternBGR, this->m_imgPatternYCrCb, CV_BGR2YCrCb);
      
      //for (int i=0; i<21; i++)
      //{
      //	for (int j=0; j<21; j++)
      //	{
      //		CvScalar clr = cvGet2D(this->m_imgPatternYCrCb , i , j);
      //		cout << clr.val[0] << " " << clr.val[1] << " " << clr.val[2] << endl;
      //	}
      //}
      //cvShowImage("1" , this->m_imgPatternYCrCb);
      //cvWaitKey(0);
      
      this->m_imgPatternGray = cvCreateImage( cvGetSize(m_imgPatternBGR) , IPL_DEPTH_8U , 1);
      cvCvtColor(this->m_imgPatternBGR, this->m_imgPatternGray, CV_BGR2GRAY);
    }
    
    if (type == POSITIVE)
    {
      this->isPositive = true;
    }
    else if (type == NEGATIVE)
    {
      this->isPositive = false;
    }
    else
      ROS_ERROR("[Pattern] : Wrong Pattern type at pattern creation!"); 
  }

  /**
   * Calculates the distance of two given 3D points, using various methods.
   * @param pt1
   * @param pt2
   * @param type
   * @return
   */
  double Pattern::calculatePointDistance(CvPoint3D32f pt1, CvPoint3D32f pt2, int type)
  {
    double result;
    
    switch (type)
    {
      case EUCLIDEAN :
        result = sqrt( pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2) +  pow((pt1.z-pt2.z),2) );
        break;
      
      case EUCLIDEAN_SIMPLE : 
        result = pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2) +  pow((pt1.z-pt2.z),2);
        break;
        
      default:
        result = pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2) +  pow((pt1.z-pt2.z),2);
        break;
    }
    
    return result;
  }

  /**
   * Calculates the distance of two given 2D points, using various methods.
   * @param pt1
   * @param pt2
   * @param type
   * @return
   */
  double Pattern::calculatePointDistance(CvPoint2D32f pt1, CvPoint2D32f pt2, int type)
  {
    double result;
    
    switch (type)
    {
      case EUCLIDEAN :
        result = sqrt( pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2) );
        break;
      
      case EUCLIDEAN_SIMPLE : 
        result = pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2);
        break;
        
      default:
        result = pow((pt1.x-pt2.x),2) +  pow((pt1.y-pt2.y),2);
        break;
    }
    
    return result;
  }

  /**
   * Deallocates related memory.
   */
  void Pattern::destroyPattern()
  {
    if (m_imgPatternGray)
      cvReleaseImage(&m_imgPatternGray);
      
    if (m_imgPatternBGR)
      cvReleaseImage(&m_imgPatternBGR);
      
    if (m_imgPatternYCrCb)
      cvReleaseImage(&m_imgPatternYCrCb);		
    
    if (m_hist.values)
      delete m_hist.values;
      
    if (m_hist.valuesNorm)
      delete m_hist.valuesNorm;
  }

  /**
   * Creates the histogram of a given texture, for backprojection purposes.
   * Uses an array of 3D points.
   * @param colors3d
   * @param bins
   * @param colorspace
   */
  void Pattern::calculateHistogram(CvPoint3D32f* colors3d, int bins, int colorspace)
  {
    if (m_hist.values != NULL)
    {
      delete m_hist.values;
      delete m_hist.valuesNorm;
      m_hist.values = NULL;
      m_hist.valuesNorm = NULL;
    }
    m_hist.values = new int[bins];
    m_hist.valuesNorm = new double[bins];
    m_hist.bins = bins;
    
    IplImage* img;
    if (colorspace == BGR)
      img = this->m_imgPatternBGR;
    else if (colorspace == YCrCb)
      img = this->m_imgPatternYCrCb;
    else
      ROS_ERROR("[Pattern] : Wrong colorspace at histogram calculation"); 
          
    /// initialize histogram to 0 values
    for (int i=0; i<bins; i++)
    {
      m_hist.values[i] = 0;
      m_hist.valuesNorm[i] = 0;
    }
        
    int height     			= img->height;
    int width     			= img->width;
    int step      			= img->widthStep/sizeof(uchar);
    int channels  			= img->nChannels;
    uchar* data   			= (uchar*)img->imageData;
                  
    for ( int i=0; i<height; i++ )
    {
      for ( int j=0; j<width; j++ )
      {
        CvPoint3D32f pt1;
        pt1.x = data[i*step+j*channels+0];
        pt1.y = data[i*step+j*channels+1];
        pt1.z = data[i*step+j*channels+2];
          
        double distance = 0;
        double minDistance = -1;
        int index = 0;
        
        for (int k=0; k<bins; k++)
        {
          CvPoint3D32f pt2 = colors3d[k];
          
          distance = calculatePointDistance(pt1, pt2, EUCLIDEAN_SIMPLE);
                  
          if ( (minDistance == -1) || (minDistance > distance) )
          {
            minDistance = distance;
            index = k;
          }
        }
        m_hist.values[index]++;
      }
    }	
    
    double sum = 0;
    for (int k=0; k<bins; k++)
      sum += m_hist.values[k];
    
    for (int k=0; k<bins; k++)
      m_hist.valuesNorm[k] = (double)m_hist.values[k] / sum;
  }

  /**
   * Creates the histogram of a given texture, for backprojection purposes.
   * Uses a matrix of vectors (2D points).
   * @param centroids2d
   * @param bins
   * @param colorspace
   */
  void Pattern::calculateHistogram(Mat_<Vec2f>* centroids2d, int bins, int colorspace)
  {
    
    //cout << "From pattern" << endl;
    //for (int i=0; i<bins; i++)
    //	cout << (*centroids2d)(i,0)[0] << " " << (*centroids2d)(i,0)[1] << endl;
    //cout << endl;
    
    if (m_hist.values != NULL)
    {
      delete m_hist.values;
      delete m_hist.valuesNorm;
      m_hist.values = NULL;
      m_hist.valuesNorm = NULL;
    }
    m_hist.values = new int[bins];
    m_hist.valuesNorm = new double[bins];
    m_hist.bins = bins;
    
    IplImage* img;
    if (colorspace == BGR)
      img = this->m_imgPatternBGR;
    else if ( (colorspace == YCrCb) || (colorspace == CrCb) )
      img = this->m_imgPatternYCrCb;
    else
      ROS_ERROR("[Pattern] : Wrong colorspace at histogram calculation"); 
          
    /// initialize histogram to 0 values
    for (int i=0; i<bins; i++)
    {
      m_hist.values[i] = 0;
      m_hist.valuesNorm[i] = 0;
    }
        
    int height     			= img->height;
    int width     			= img->width;
    int step      			= img->widthStep/sizeof(uchar);
    int channels  			= img->nChannels;
    uchar* data   			= (uchar*)img->imageData;
      
    //cout << "pattern " << height << endl;						
    for ( int i=0; i<height; i++ )
    {
      for ( int j=0; j<width; j++ )
      {
        Vec2f pt1;
        pt1[0] = data[i*step+j*channels+1];
        pt1[1] = data[i*step+j*channels+2];
          
        double distance = 0;
        double minDistance = -1;
        int index = 0;
        for (int k=0; k<centroids2d->rows; k++)
        {
          Vec2f pt2 = (*centroids2d)(k,0);
          distance = pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2);
          
          if ( (minDistance == -1) || (minDistance > distance) )
          {
            minDistance = distance;
            index = k;
          }
        }
        m_hist.values[index]++;
      }
    }	
    
    double norm = 0;
    for (int k=0; k<bins; k++)
      norm += pow( m_hist.values[k] , 2);
    
    for (int k=0; k<bins; k++)
      m_hist.valuesNorm[k] = (double)m_hist.values[k] / sqrt( norm );
  }

  /**
   * Histogram visualizer for debugging purposes.
   * Uses an array of 3D points.
   * @param colors
   */
  void Pattern::visualizeHistogram(CvPoint3D32f* colors)
  {	
    ///create histogram image
    IplImage* imgHist = cvCreateImage( cvSize(600,400) , 8 , 3 );
    cvZero(imgHist);
    
    int bins = m_hist.bins;
    int xStep = (int)(imgHist->width / bins);
    
    int totalPixels = m_imgPatternBGR->width * m_imgPatternBGR->height;
    double* visHist = new double[bins];
    for (int i=0; i<bins; i++)
      visHist[i] = (double)(m_hist.values[i]) / totalPixels;
    
    int xBL;
    int yBL;
    int xUR;
    int yUR;
    for (int i=0; i<bins; i++)
    {
      CvScalar clr = cvScalar(colors[i].x , colors[i].y , colors[i].z);
      xBL = i * xStep;
      yBL = imgHist->height-1;
      xUR = (i+1) * xStep;
      yUR = (int)(yBL - visHist[i] * imgHist->height); 
          
      cvRectangle(imgHist, cvPoint(xBL,yBL), cvPoint(xUR,yUR), clr, -1, 8, 0);	
    }
    cvShowImage("1" , m_imgPatternBGR);
    cvShowImage("2" , imgHist);
    cvWaitKey(0);
    
    cvDestroyWindow("1");
    
    delete visHist;
    cvReleaseImage(&imgHist);
    
  }

  /**
   * Histogram visualizer for debugging purposes.
   * Uses a matrix of vectors(2d points).
   * @param colors
   */
  void Pattern::visualizeHistogram(Mat_<Vec2f>* colors)
  {
    ///create histogram image
    IplImage* imgHist = cvCreateImage( cvSize(600,400) , 8 , 3 );
    cvZero(imgHist);
    
    int bins = m_hist.bins;
    int xStep = (int)(imgHist->width / bins);
    
    int totalPixels = m_imgPatternBGR->width * m_imgPatternBGR->height;
    double* visHist = new double[bins];
    for (int i=0; i<bins; i++)
      visHist[i] = (double)(m_hist.values[i]) / totalPixels;
    
    int xBL;
    int yBL;
    int xUR;
    int yUR;
    
    for (int i=0; i<bins; i++)
    {
      //cout << visHist[i] << endl;
      
      xBL = i * xStep;
      yBL = imgHist->height;
      xUR = (i+1) * xStep;
      yUR = (int)( (1-visHist[i]) * imgHist->height); 
      
      float yStep = float(yBL - yUR)/255.0;
        
      //cout << xBL << " " << yBL << "||" << xUR << " " << yUR << " and yStep " << yStep << endl;

      //create gradient effect for luminocity, because we don't know exact value
      if (yStep != 0)
      {
        for (int j=0; j<255; j++)
        {
          CvScalar clr = cvScalar(j, (*colors)(i,0)[0], (*colors)(i,0)[1]);
          CvScalar bgrClr = YCrCb2BGR(clr);
          bgrClr.val[0] = (int)bgrClr.val[0];
          bgrClr.val[1] = (int)bgrClr.val[1];
          bgrClr.val[2] = (int)bgrClr.val[2];
        
          int yB = (int)(yBL - j * yStep);
          int yU = (int)(yBL - (j-1) * yStep);
          cvRectangle(imgHist, cvPoint(xBL,yB), cvPoint(xUR,yU), bgrClr , -1, 8, 0);		
        }
      }
      
    }
    cvShowImage("1" , m_imgPatternBGR);
    cvShowImage("2" , imgHist);
    cvWaitKey(0);
    
    cvDestroyWindow("1");
    
    delete visHist;
    cvReleaseImage(&imgHist);
  }
}
