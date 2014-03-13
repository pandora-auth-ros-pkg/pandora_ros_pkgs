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
* Author: Skartados Evangelos
*********************************************************************/

#include <opencv2/highgui/highgui.hpp>
#include "pandora_vision_face/skin_detector.h"
namespace pandora_vision
{
/**
 @brief Class Constructor
 Loads the images representing the histogramms of skin and background areas.
 The histogramms are 256X256, 8-bit, single-channel (binary) images.
 The height axis of the histogramm represents the Cr channel of the
 initial image and the width axis the Cb channel.
 Each pixel represents the normalized number of times the corresponding
 (Cr,Cb) component appears on the initial images.
 @param skinHist  string The name of the skin histogramm image.
  		  wallHist  string The name of the wall histogramm image.
  		  wall2Hist string The name of the second type of wall histogramm image.
*/
SkinDetector::SkinDetector( std::string skinHist, std::string wallHist, std::string wall2Hist )
{
  imgHistogrammSkin  = cv::imread( skinHist.c_str()  ,  0 );
  std::cout << skinHist.c_str() << "\n";
  imgHistogrammWall  = cv::imread( wallHist.c_str()  ,  0 );
  std::cout << wallHist.c_str() << "\n";
  imgHistogrammWall2 = cv::imread( wall2Hist.c_str() ,  0 );
  std::cout << wall2Hist.c_str() << "\n";
  std::cout << "Created SkinDetector instance" << std::endl;
}

/**
 @brief Class Destructor
 Releases the images that represent the histogramms.
*/
SkinDetector::~SkinDetector()
{

  std::cout << "Destroying SkinDetector instance" << std::endl;
}


/**
 @brief Function that initializes all the pointers used
 for the detection of skin blobs
 @return void
*/
void SkinDetector::init()
{
  contourCounter     = 0;
  contourCenter      = 0;
  contourProbability = 0;
  contourSize        = 0;
}


/**
 @brief Function that releases all the pointers
 after the detection of skin blobs is done
 @return void
*/
void SkinDetector::deallocateMemory()
{
  delete [] contourCenter;
  delete [] contourProbability;
  delete [] contourSize;
}


/**
 @brief Function used for the communication between SkinDetector
 and FaceDetector insatnce
 @return imgContours [cv::IplImage] contains the areas
 which have been detected as skin blobs.
*/
cv::Mat SkinDetector::getImgContoursForFace()
{
  cv::dilate( imgContours, imgContours, cv::Mat(), cv::Point(), 6);
  return imgContours;
}


/**
 @brief Function that given the already loaded histogramm images
 decided which areas of the current image are skin blobs
 @return imgInput [cv::IplImage] the current image
 that must be checked for skin areas.
*/
void SkinDetector::createCalculationImages(cv::Mat imgInput)
{
  imgSrc = imgInput.clone();
  imgThreshold.create(imageHeight, imageWidth, CV_8UC1);

  imgThresholdFiltered.create(imageHeight, imageWidth, CV_8UC1);

  imgContours.create(imageHeight, imageWidth, CV_8UC1);
}

/**
 @brief Function that checks if histogramms are loaded correctly
 @return [bool], true if all patterns are loaded correctly,
 false if not
*/
bool SkinDetector::histogrammsLoaded()
{
  if ( imgHistogrammSkin.empty() )
  {
    std::cout << "couldn't load image histogramm_skin \n";
    return false;
  }

  if ( imgHistogrammWall.empty() )
  {
    std::cout << "couldn't load image histogramm_wall \n";
    return false;
  }

  if ( imgHistogrammWall2.empty() )
  {
    std::cout << "couldn't load image histogramm_wall2 \n";
    return false;
  }

  return true;
}

/**
 @brief Function that calculates all necessary parameters
 for skin detection
 @return void
*/
void SkinDetector::getCalculationParams(int& stepSrc, uchar* &dataSrc , int& channels,
                                        int& stepThreshold, uchar* &dataHistogrammSkin, int& stepHistogrammSkin,
                                        uchar* &dataHistogrammWall, int& stepHistogrammWall, uchar* &dataHistogrammWall2,
                                        int& stepHistogrammWall2, uchar* &dataContours, int& stepContours)
{
  stepSrc = imgSrc.step;
  dataSrc = (uchar *)imgSrc.data;
  channels = imgSrc.channels();

  stepThreshold = imgThreshold.step;

  dataHistogrammSkin = (uchar *)imgHistogrammSkin.data;
  stepHistogrammSkin = imgHistogrammSkin.step;

  dataHistogrammWall = (uchar *) imgHistogrammWall.data;
  stepHistogrammWall = imgHistogrammWall.step;

  dataHistogrammWall2 = (uchar *)imgHistogrammWall2.data;
  stepHistogrammWall2 = imgHistogrammWall2.step;

  dataContours	  = (uchar*)imgContours.data;
  stepContours	  = imgContours.step;

  imgThreshold = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
  imgThresholdFiltered = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);
  imgContours = cv::Mat::zeros(imageHeight, imageWidth, CV_8UC1);

  //cvCvtColor(imgSrc,imgSrc, CV_BGR2HSV);
}

/**
 @brief Function that scans given frame and checks if skin is detected
 @return void
*/
void SkinDetector::scanForSkin(int stepSrc, uchar* dataSrc , int channels,
                               int stepThreshold, uchar* dataHistogrammSkin, int stepHistogrammSkin, uchar* dataHistogrammWall,
                               int stepHistogrammWall, uchar* dataHistogrammWall2, int stepHistogrammWall2)
{
  cv::Mat hsv;
  hsv = imgSrc.clone();
  cv::cvtColor( hsv, hsv, CV_BGR2HSV );

  // Threshold the image
  cv::Mat imageThreshold;
  cv::inRange(hsv, cv::Scalar(0, 58, 89), cv::Scalar(25, 173, 229), imageThreshold);

  imgThreshold = imageThreshold.clone();
}

/**
 @brief Function that calculates probability to have skin
 in current frame
 @return void
*/
void SkinDetector::calculatePropability(uchar *dataContours, int stepContours)
{
  //allocate memory dynamically to save the contours' properties
  contourCenter      = new cv::Point[Contour.size()];
  contourProbability = new float[Contour.size()];
  contourSize        = new int[Contour.size()];
  contourCounter = 0;

  //get contours' properties
  for( int i = 0; i < Contour.size(); ++i )
  {
    if( contourArea(Contour.at(i)) > sizeThreshold )
    {
      std::cout << "IMGCONTOURS CHANGE" << std::endl;
      cv::drawContours( imgContours, Contour, i, CV_RGB(255, 255, 255));
      contourSize[contourCounter] = int(contourArea( Contour.at(i)));

      cv::Rect boundingRect = cv::boundingRect( cv::Mat(Contour[i]) );
      double boundingHeight = boundingRect.height;
      double boundingWidth = boundingRect.width;
      int startX = boundingRect.x;
      int endX = startX + boundingRect.width;
      int startY = boundingRect.y;
      int endY = startY + boundingRect.height;

      contourCenter[contourCounter] = cv::Point( ( startX + boundingWidth / 2 ), ( startY + boundingHeight / 2 ) );

      int positive = 0;
      int negative = 0;
      for (int j = startX; j < endX; j++)
      {
        for ( int i = startY; i < endY; i++)
        {
          if (dataContours[i * stepContours + j] != 0)
          {
            positive++;
          }
          else
          {
            negative++;
          }
        }
      }
      contourProbability[contourCounter] = (float) positive / (positive + negative);
      contourCounter++;
    }
  }
  for( int i = 0; i < contourCounter; i++ )
  {
    std::cout << "CountourCenter = " << "( " << contourCenter[i].x << " , " << contourCenter[i].y << " )" << std::endl;
    std::cout << "CountourSize = " << contourSize[i] << std::endl;
    std::cout << "Propability = " << contourProbability[i] << std::endl << std::endl;
  }
}

int SkinDetector::detectSkin(cv::Mat imgInput)
{

  imageHeight = imgInput.size().height;
  imageWidth = imgInput.size().width;

  createCalculationImages(imgInput);

  if (!histogrammsLoaded())
  {
    std::cout << "Problem loading histogramm images" << std::endl;
    return 1;
  }

  int stepSrc, channels, stepThreshold, stepHistogrammSkin;
  int stepContours, stepHistogrammWall, stepHistogrammWall2;
  uchar *dataSrc, *dataHistogrammSkin, *dataHistogrammWall, *dataHistogrammWall2, *dataContours;

  getCalculationParams(stepSrc, dataSrc, channels,
                       stepThreshold, dataHistogrammSkin, stepHistogrammSkin,
                       dataHistogrammWall, stepHistogrammWall, dataHistogrammWall2,
                       stepHistogrammWall2, dataContours, stepContours);

  scanForSkin(stepSrc, dataSrc, channels,
              stepThreshold, dataHistogrammSkin, stepHistogrammSkin,
              dataHistogrammWall, stepHistogrammWall, dataHistogrammWall2,
              stepHistogrammWall2);

  cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS , cv::Size(3, 3), cv::Point( -1, -1 ) );
  cv::morphologyEx(imgThreshold, imgThresholdFiltered, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 3);

  std::vector<cv::Vec4i> hierarchy;
  findContours( imgThresholdFiltered, Contour, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  if (!Contour.empty())
  {
    calculatePropability(dataContours, stepContours);
  }

  std::cout << "contourCounter = " << contourCounter << std::endl;
  return 0;
}

}
