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

#include "pandora_vision_hole/hole_detector.h"

namespace pandora_vision
{
  HoleFinder::HoleFinder()
  {		
    //imgSrc 					= NULL;
    imgEdge 				= NULL;
    imgThreshold 			= NULL;
    imgContours 			= NULL;
    imgTexture 				= NULL;

    strgContours = NULL;

    seqContours = 0;
    seqCurrent = 0;

    ////////////////////////////////////
    //Initialize HoleFinder Parameters//
    ////////////////////////////////////

    // source image filtering
    _modeA = 0;			// predefined values
    _equalizeA = 0; 	// 0:false , 1:true
    _smoothA = 0; 		// <5:false
    _erodeA = 0; 		// 0:false
    _dilateA = 0; 		// 0:false
    _openA = 0; 		// 0:false
    _closeA = 0; 		// 0:false

    //edge detection
    _modeB = 0;			//0 , 1

    _cannyConvKernelB = 3;
    _cannyLowThresB = 20;
    _cannyHighThresB = 40;
    _dilateB = 2;
    _cannyConvKernelB1 = 3;
    _cannyLowThresB1 = 0;
    _cannyHighThresB1 = 1;

    _gradientB = 0;
    _equalizeB = 1;
    _thresholdLowThresB = 250;
    _thresholdHighThresB = 255;
    _dilateB1 = 2;
    _cannyConvKernelB2 = 3;
    _cannyLowThresB2 = 0;
    _cannyHighThresB2 = 1;

    //source image thresholding
    _closeC = 2;
    _openC = 2;
    _erodeC = 2;
    _dilateC = 0;
    _thresholdLowThresC = 120;
    _thresholdHighThresC = 255;

    //texture image thresholding
    _smoothD = 5;
    _closeD = 6;
    _openD = 6;
    _erodeD = 8;
    _dilateD = 10;
    _thresholdLowThresD = 127;
    _thresholdHighThresD = 255;

    //contour filtering parameters
    _lengthContour = 50;			//minimum contour length
    _areaContour = 100;				//minimum contour area
    _maxAreaContour = 150000.;		//maximum area contour
    _rectHeightContour = 40;		//minimum bounding rect height
    _rectWidthContour = 50;				//minimum bounding rect width
    _heightToWidthContour = 1.5;		//maximum height to width ratio
    _widthToHeightContour = 3.5;		//maximum width to height ratio
    _pixelToRectAreaContour = 0.65;		//minimum blob area to bounding rect area ratio
    _darkToPixelAreaContour = 0.50;		//minimum dark pixels to blob area ratio
    _textureToPixelAreaContour = 0.85; 	//minimum texture area to blob area

    //blob filtering parameters
    _minFormFactor = 0.9;			//minimum form factor
    _ellipseFactorMax = 0.8;		//minimum ellipse area to actual area
    _ellipseFactorMin = 1.3;		//maximum ellipse area to actual area
    _minAxisValue = 60;
    _verticalAxisRatio = 1.2; 
    _horizontalAxisRatio = 3.4;
    _thetaLow = 40;
    _thetaHigh = 70;


    filter = new TextureFilter();
    filter->setColorSpace(CrCb);		

    ROS_INFO("[hole_detector] : Created HoleFinder instance"); 
  #if HOLEFINDER_DEBUG_MODE
    createWindows();
  #endif
  }

  HoleFinder::~HoleFinder()
  {		

  #if HOLEFINDER_DEBUG_MODE
    cv::destroyAllWindows();
  #endif

    delete filter;
    filter = NULL;

    ROS_INFO("[hole_detector] : Destroying HoleFinder instance");
  }

  void HoleFinder::setTexturePath(std::string str)
  {
    filter->setPath(str);

    ROS_INFO("[HoleFinder] : loading centers from file");
    filter->readCentersFromFile();
    filter->clusterPixels();

    ROS_INFO("[HoleFinder] : loading patterns");
    filter->loadPatterns(POSITIVE);
    filter->loadPatterns(NEGATIVE);

    //cout << "[HoleFinder] : showing patterns" << endl;
    //filter->showPatterns(POSITIVE);
    //filter->showPatterns(NEGATIVE);

    ROS_INFO("[HoleFinder] : calculating patterns");
    filter->calculatePatternHistograms();

    //cout << "[HoleFinder] : printing/visualizing patterns" << endl;
    //filter->printPatternHistograms();
    //filter->visualizePatternHistograms();
  }

  /**
   * Threshold input image, using upper and lower threshold. This method clears the
   * result of a previous template matching of the input frame.
   * @param imgInput
   */
  void HoleFinder::findThresholdImage(cv::Mat imgInput)
  {	
    cv::Mat imgTemp = cv::Mat(size, CV_8UC1);
    //cv::Mat imgInput_mat = cv::Mat(imgInput);

    cv::threshold(imgInput, imgTemp, _thresholdLowThresC, _thresholdHighThresC, THRESH_BINARY);

    if (_closeC > 0)
      cv::morphologyEx(imgTemp,imgTemp,MORPH_CLOSE,cv::Mat(),cv::Point(),_closeC);

    if (_openC)
      cv::morphologyEx(imgTemp,imgTemp,MORPH_OPEN,cv::Mat(),cv::Point(),_openC);

    if (_erodeC)
      cv::erode(imgTemp,imgTemp, cv::Mat(), cv::Point(), _erodeC);

    if (_dilateC)	
      cv::dilate(imgTemp,imgTemp, cv::Mat(), cv::Point(), _dilateC);

    IplImage local_temp = imgTemp;
    cvNot(&local_temp,imgThreshold);
  }

  /**
   * Runs texture matching on input image, given a specific texture.
   * @param imgCap
   */
  void HoleFinder::findTextureImage(cv::Mat imgCap)
  {	

    cv::Mat imgTemp = cv::Mat(size, CV_8UC1);

    IplImage local_temp = imgCap;
    IplImage* local_txt = filter->calculateTexture(&local_temp);
    cv::Mat res(local_txt);


    if (res.empty())
    {
      cvZero(imgTexture);
      return;
    }

    //cvConvertScale(res, imgTemp, 255.);
    res.convertTo(imgTemp, CV_8UC1, 255.);
    cvReleaseImage(&local_txt);
  #if HOLEFINDER_DEBUG_MODE
    imshow("Texture probability", imgTemp);
  #endif

    if (_smoothD > 4)
    {
      int smooth = _smoothD + _smoothD%2 - 1;
      cv::GaussianBlur(imgTemp,imgTemp,cv::Size(smooth,smooth), 0, 0);
    }

    if (_closeD > 0)
      cv::morphologyEx(imgTemp,imgTemp,MORPH_CLOSE,cv::Mat(),cv::Point(),_closeD);

    if (_openD)	
      cv::morphologyEx(imgTemp,imgTemp,MORPH_OPEN,cv::Mat(),cv::Point(),_openD);

    if (_erodeD)	
      cv::erode(imgTemp,imgTemp, cv::Mat(), cv::Point(), _erodeD);

    if (_dilateD)	
      cv::dilate( imgTemp, imgTemp, cv::Mat(), cv::Point(), _dilateD);


    if (_thresholdLowThresD != 255)
      cv::threshold(imgTemp, imgTemp, _thresholdLowThresD, _thresholdHighThresD, THRESH_BINARY);

    cv::rectangle(imgTemp, cv::Point(0,0) , cv::Point(639,479), cv::Scalar(255), 38, 8, 0);
    local_temp = imgTemp;
    cvCopy(&local_temp, imgTexture);
  }

  /**
   * Retrieves blobs from contours of potential holes.
   * @param input
   */
  void HoleFinder::findBlobs(IplImage* input)
  {

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat imge_mat = cv::Mat(input).clone();
    cv::findContours( imge_mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    contourCounter = contours.size();
    for(int i=0 ; i< contours.size(); i++){
      imge_mat = cv::Mat::zeros(imge_mat.size(), CV_8UC1);
      cv::drawContours(imge_mat,contours,i,cv::Scalar(255), CV_FILLED);


      Thing* blob = new Thing();
      blob->createThing(imge_mat , contours[i]);
  //		cout << "[Holefinder] blob area:" << blob->area << endl;
  //		cout << "[Holefinder] blob theta:" << blob->theta << endl;
  //		cout << "[Holefinder] maj/min axis:" << (blob->maj_axis) / (blob->min_axis) << endl;

      resultBlobs.push_back(blob);

      //Draw a rectangle around holes on imgSrc for debugging purposes
      int xUL = blob->m_rect.x;
      int yUL = blob->m_rect.y;
      int xBR = blob->m_rect.x + blob->m_rect.width;
      int yBR = blob->m_rect.y + blob->m_rect.height;

      rectangle(imgSrc, cv::Point(xUL , yUL),
          cv::Point(xBR , yBR), cv::Scalar(0, 0, 255, 0),	3, 8, 0);

    }

  }


  /**
   * Retrieves contours using detected edges of input image.
   * @param input
   */
  void HoleFinder::findContours(IplImage* input)
  {	
    strgContours = cvCreateMemStorage(0);

    IplImage* imgTemp = (IplImage*)cvClone(input);

    CvContourScanner scanner;
    scanner = cvStartFindContours( imgTemp, strgContours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

    while( (seqCurrent = cvFindNextContour( scanner )) != 0 ) 
    {
      /////////////////////////////////////////////////////////////////////////
      // Apply filters to exclude most of the contours! Try to keep the holes
      /////////////////////////////////////////////////////////////////////////

      //**************************
      //apply "perimeter" check
      //**************************

      double len = cvContourPerimeter( seqCurrent );   //same as "cvArcLength( c, CV_WHOLE_SEQ, -1 );"

      if( len < _lengthContour ) {
        cvSubstituteContour( scanner, 0 );
        continue;
      }		 

      //**************************
      //apply "area" check
      //**************************

      //the area is computed using the Green formula,
      //thus the returned area and the number
      //of non-zero pixels can be different.
      double area = cvContourArea( seqCurrent );   //same as "cvContourArea( c , CV_WHOLE_SEQ );"

      if( area < _areaContour ) {
        cvSubstituteContour( scanner, 0 );
        continue;
      }

      if( area > _maxAreaContour && _maxAreaContour != 0 ){
        cvSubstituteContour( scanner, 0 );
        continue;
      }

      //***************************
      //apply "axis" and
      //bounding rect checks
      //***************************

      //same as "cvBoundingRect( ((CvContour*)c), 0)" or "cvBoundingRect(c,0)" 
      //in second case the rectangle is calculated, while in the first case, rect
      //is not calculated, but it is taken from rect field of the contour header.
      CvRect boundingRect = ((CvContour*)seqCurrent)->rect;
      double boundingHeight = boundingRect.height;
      double boundingWidth = boundingRect.width;
      double rectArea = boundingWidth*boundingHeight;

      if (rectArea == 0)
      {
        cvSubstituteContour( scanner, 0 );
        ROS_ERROR( "[hole_detector] : bounding rect of contour can't have zero area... Something's wrong");
        continue;
      }

      if ( (boundingHeight < _rectHeightContour) || (boundingWidth < _rectWidthContour) )
      {
        cvSubstituteContour( scanner, 0 );
        continue;
      }

      if ( (boundingHeight/boundingWidth > _heightToWidthContour) || (boundingWidth/boundingHeight > _widthToHeightContour) )
      {
        cvSubstituteContour( scanner, 0 );
        continue;
      }

      //*****************************
      //count actual contour area 
      //			&
      //apply "dark region" check
      //apply "texture region" check
      //*****************************

      IplImage* imgTest = cvCreateImage( size , IPL_DEPTH_8U , 3);
      IplImage* imgCurrent = cvCreateImage( size , IPL_DEPTH_8U , 1);

      cvZero( imgTest );
      cvDrawContours( imgTest, seqCurrent, cvScalarAll(255), cvScalarAll(0), -1, CV_FILLED, 8, cvPoint(0,0));

      cvCvtColor(imgTest,imgCurrent,CV_RGB2GRAY);
      cvReleaseImage( &imgTest);  

      double pixelArea=0;
      double darkArea=0;
      double texturelessArea=0;

      //search only inside the bounding rect
      //find dark areas
      //find textureless areas
      int startX = boundingRect.x;
      int endX = startX + boundingRect.width;
      int startY = boundingRect.y;
      int endY = startY + boundingRect.height;

      int step      = imgCurrent->widthStep;
      int channels  = imgCurrent->nChannels;
      uchar* dataCur   = (uchar *)imgCurrent->imageData;
      uchar* dataThres = (uchar *)imgThreshold->imageData;
      uchar* dataTexture = (uchar *)imgTexture->imageData;

      for (int j=startX; j<endX; j++)
      {
        for (int i=startY; i<endY; i++)
        {
          for(int k=0; k<channels; k++)
          {

            //count area
            if  ( (int)dataCur[i*step+j*channels+k] == 255 ) 
            {
              pixelArea++;

              //count dark area
              if ( (int)dataThres[i*step+j*channels+k] == 255 )
                darkArea++;

              //count textureless area
              if ( (int)dataTexture[i*step+j*channels+k] == 255 )
                texturelessArea++;
            }			
          }
        }
      }

      if (pixelArea/rectArea <= _pixelToRectAreaContour)
      {
        cvSubstituteContour( scanner, 0 );
        cvReleaseImage( &imgCurrent);
        continue;
      }

      if (darkArea/pixelArea <= _darkToPixelAreaContour)
      {
        cvSubstituteContour( scanner, 0 );
        cvReleaseImage( &imgCurrent);
        continue;
      }

      /// check if the blob corresponds to a textureless,
      /// negative area and not a wall
      if (texturelessArea/pixelArea < 0.9)
      {
        cvSubstituteContour( scanner, 0 );
        cvReleaseImage( &imgCurrent);
        continue;
      }

      //**************************************************
      // End of Checks
      //**************************************************

      cvReleaseImage( &imgCurrent);
    }
    seqContours = cvEndFindContours( &scanner );

    // paint the found regions back into the image
    cvZero( imgContours );
    for( seqCurrent=seqContours; seqCurrent != 0; seqCurrent = seqCurrent->h_next )
    {
      cvDrawContours( imgContours, seqCurrent, cvScalarAll(255), cvScalarAll(0), -1, CV_FILLED, 8, cvPoint(0,0));
    }

    cvReleaseImage( &imgTemp);	

    cvReleaseMemStorage(&strgContours);	
  }

  /**
   * Retrieves edges from input image.
   * @param imgInput
   */
  void HoleFinder::findEdgeImage(IplImage* imgInput)
  {
    cv::Mat imgInput_mat = cv::Mat(imgInput);
    cv::Mat imgTemp = cv::Mat(size, CV_8UC1);
    imgInput_mat.copyTo(imgTemp);

    ///////////////////////////////////////////////////////////
    // Depending on the operation mode, different approaches
    // on finding good edges are applied
    ///////////////////////////////////////////////////////////

    switch ( _modeB ) 
    {
    case 0 :

      bool validKernel;
      bool validThresholds;

      validKernel = (_cannyConvKernelB == 3) || (_cannyConvKernelB == 5) || (_cannyConvKernelB == 7);
      validThresholds = (_cannyLowThresB >= 0) && (_cannyHighThresB >= 0);
      if ( validKernel && validThresholds)
      {
        int N = _cannyConvKernelB;
        cv::Canny(imgTemp, imgTemp, _cannyLowThresB, _cannyHighThresB, N );
      }
      else
        ROS_ERROR("[hole_detector] : Invalid Edge Detection Canny Parameters 1 ");

      if (_dilateB > 0)
        cv::dilate(imgTemp,imgTemp,cv::Mat(),cv::Point(),_dilateB);

      validKernel = (_cannyConvKernelB1 == 3) || (_cannyConvKernelB1 == 5) || (_cannyConvKernelB1 == 7);
      validThresholds = (_cannyLowThresB1 >= 0) && (_cannyHighThresB1 >= 0);
      if ( validKernel && validThresholds)
      {
        int N = _cannyConvKernelB1;
        cv::Canny(imgTemp, imgTemp, _cannyLowThresB1, _cannyHighThresB1, N );
      }
      else
        ROS_ERROR("[hole_detector] : Invalid Edge Detection Canny Parameters 2 ");

      break;

    case 1:
      if (_gradientB > 0)
      {
        cv::Mat imgTemp1 = imgTemp.clone();
        cv::morphologyEx(imgTemp, imgTemp, MORPH_GRADIENT, _gradientB);
      }

      if (_equalizeB == 1)
        cv::equalizeHist(imgTemp,imgTemp);

      cv::threshold(imgTemp, imgTemp, _thresholdLowThresB, _thresholdHighThresB, THRESH_BINARY);

      if (_dilateB1 > 0)
        cv::dilate(imgTemp, imgTemp, cv::Mat(), cv::Point(), _dilateB1);


      validKernel = (_cannyConvKernelB2 == 3) || (_cannyConvKernelB2 == 5) || (_cannyConvKernelB2 == 7);
      validThresholds = (_cannyLowThresB2 >= 0) && (_cannyHighThresB2 >= 0);
      if ( validKernel && validThresholds)
      {
        int N = _cannyConvKernelB2;
        cv::Canny( imgTemp, imgTemp, _cannyLowThresB2, _cannyHighThresB2, N );
      }
      else
        ROS_ERROR("[hole_detector] : Invalid Edge Detection Canny Parameters 3 ");

      break;

    default:
      ROS_ERROR("[hole_detector] : Unknown parameter for edge mode!!!");
      break;
    }

    IplImage local_temp = imgTemp;
    cvCopy(&local_temp, imgEdge);
  }

  /**
   * Input image preprocess
   * @param imgCap
   */
  void HoleFinder::modifySourceImage(cv::Mat imgCap)
  {	
    cv::Mat imgTemp;

    //check image channels
    if (imgCap.channels() == 3)
    {
      imgTemp = cv::Mat(size, CV_8UC1);
      cv::cvtColor(imgCap,imgTemp,CV_BGR2GRAY);
    }
    else if (imgCap.channels() == 1)
      imgTemp = imgCap.clone();
    else
      ROS_ERROR("[hole_detector] : Unknown number of channels for input image");


    ///////////////////////////////////////////////////////////
    // Depending on the operation mode, different approaches
    // on modifying input image are applied
    ///////////////////////////////////////////////////////////

    switch ( _modeA ) 	
    {
    case 0 : 	//First mode of operation. Only one exists at the moment

      if (_equalizeA > 0)
        equalizeHist( imgTemp, imgTemp );

      if (_smoothA > 4)
      {
        int smooth = _smoothA + _smoothA%2 - 1;
        cv::GaussianBlur(imgTemp,imgTemp,cv::Size(smooth,smooth), 0, 0);
      }

      if (_erodeA > 0)
        cv::erode(imgTemp,imgTemp,cv::Mat(),cv::Point(),_erodeA);

      if (_dilateA > 0)
        cv::dilate(imgTemp, imgTemp, cv::Mat(), cv::Point(), _dilateA);

      if (_openA > 0)
        cv::morphologyEx(imgTemp, imgTemp,MORPH_OPEN, _openA);

      if (_closeA > 0)
        cv::morphologyEx(imgTemp, imgTemp,MORPH_CLOSE, _closeA);

      break;

    default :
      ROS_ERROR("[hole_detector] : Unknown parameter for edge mode!!!");
      break;
    }
    IplImage local_temp = imgTemp;
    imgSrc = imgTemp.clone();
  }

  /**
   * Basic hole detection method. It detects holes or more accurately "things"
   * @param imgCap
   * @return
   */
  vector<Thing*> HoleFinder::findHoles(cv::Mat imgCap)
  {	
    size = cvSize(imgCap.cols , imgCap.rows);

    imgSrc			= cv::Mat(size, CV_8UC1);
    imgEdge 		= cvCreateImage( size , IPL_DEPTH_8U , 1);
    imgThreshold 	= cvCreateImage( size , IPL_DEPTH_8U , 1);
    imgContours 	= cvCreateImage( size , IPL_DEPTH_8U , 1);
    imgTexture  	= cvCreateImage( size , IPL_DEPTH_8U , 1);

    modifySourceImage(imgCap);	//<---new API

    findTextureImage(imgCap);	//<---new API

    findEdgeImage(imgTexture);	//<---new API

    findThresholdImage(imgSrc);	//<---new API

    findContours(imgEdge);

    findBlobs(imgContours);

  #if HOLEFINDER_DEBUG_MODE
    showImages();
  #endif

    return resultBlobs;
  }

  /**
   * Creates a buntch of windows for debugging purposes.
   */
  void HoleFinder::createWindows()
  {
    //////////////////////////////
    // Create Windows 
    // and place them on screen
    //////////////////////////////
    cv::namedWindow("Source", CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Edge" , CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Threshold" , CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Texture" , CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Result" , CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Tracking_Result" , CV_WINDOW_KEEPRATIO);
    cv::namedWindow("Texture probability",CV_WINDOW_KEEPRATIO);

  }

  /**
   * Displays frames on windows for debugging purposes.
   */
  void HoleFinder::showImages()
  {
    cv::imshow("Source", imgSrc);
    cvShowImage("Edge" , imgEdge);
    cvShowImage("Threshold" , imgThreshold);
    cvShowImage("Texture" , imgTexture);
    cvShowImage("Result" , imgContours);

    cv::waitKey(1);
  }

  /**
   * Releases allocated image memory.
   */
  void HoleFinder::cleanup()
  {
    //std::cout <<"[hole_detector] :  Cleanup" << endl;
    cvReleaseImage( &imgEdge);
    cvReleaseImage( &imgThreshold);	
    cvReleaseImage( &imgContours);
    cvReleaseImage( &imgTexture);

    resultBlobs.clear();
  }
}
