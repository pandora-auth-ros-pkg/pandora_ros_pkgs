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

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>

#include <iostream>
#include <stdlib.h>

#include "pandora_vision_hole/pattern.h"
#include "pandora_vision_hole/texture_filter.h"
#include "pandora_vision_hole/thing.h"

#define NUM_POS_CLUSTERS	50
#define NUM_NEG_CLUSTERS	50



/*
 * File Description :
 * Program to find cluster centers and patterns
 * for the segmentation process. 
 */
using namespace std;

TextureFilter* filter;
IplImage* imgLabels;
IplImage* screenShot;
IplImage* screenShotRect;
IplImage* imgResult;
IplImage* imgResultRect;
IplImage* frame;
CvRect box;
string mode;
int blockSize = 31;

bool drawing_box = false;
int counterPositive = 0; //global flag to use when saving positive images
int counterNegative = 0; //global flag to use when saving negative images

void mouse_callback(int event, int x, int y, int flags, void* param);

//Function to draw a box onto an image
void draw_box()
{
		cvMerge( imgResult, imgResult, imgResult, NULL, imgResultRect ); 
		
		cvRectangle(imgResultRect, cvPoint(box.x, box.y),	cvPoint(box.x+box.width, box.y+box.height),
				cvScalar(0,0,255), 1);
	
		cvRectangle(screenShotRect, cvPoint(box.x, box.y),	cvPoint(box.x+box.width, box.y+box.height),
				cvScalar(0,0,255), 1);
}

bool FileExists(char* strFilename) 
{
	struct stat stFileInfo;
	bool blnReturn;
	int intStat;

	// Attempt to get the file attributes
	intStat = stat(strFilename,&stFileInfo);
	if(intStat == 0) 
		blnReturn = true;
	else
		blnReturn = false;
	
	return(blnReturn);
}

//Function to save Cropped Images in file
void save(string path)
{
	if ( box.width > 0)
	{
		stringstream filename;
		filename.str("");
		
		if ( (path == POSITIVE_PATH) || (path == POSITIVE_PATTERN_PATH) )
		{
			filename << path << "0/" << counterPositive << ".png";
			counterPositive++;
		}
		else if ( (path == NEGATIVE_PATH) || (path == NEGATIVE_PATTERN_PATH) )
		{
			filename << path << "0/" << counterNegative << ".png";
			counterNegative++;
		}
		
		cout << "saving to " << filename.str() << endl;
		
		/// use the drawn rect on screenShot
		IplImage* cropped = cvCreateImage( cvSize(box.width,box.height), 8, 3);
		cvSetImageROI(screenShot, box);
		cvCopy(screenShot, cropped);
		cvResetImageROI(screenShot);
		
		cvSaveImage(filename.str().c_str(), cropped);
	}
}

//This is the mouse callback
void mouse_callback(int event, int x, int y, int flags, void* param)
{
	int offset = (blockSize-1)/2;
	if (x-offset-1<0 || y-offset-1<0 || x+offset+1>frame->width || y+offset+1>frame->height)
		return;
		
	switch( event ) 
	{
		case CV_EVENT_MOUSEMOVE : 
		{
			if ( drawing_box ) 
			{
				if ( mode == "1" )
				{
					box.width = x - box.x;
					box.height = y - box.y;
				}
				else
				{
					box.x = x-(blockSize-1)/2;
					box.y = y-(blockSize-1)/2;
					box.width = blockSize;
					box.height = blockSize;
				}
				
				memcpy(screenShotRect->imageData, screenShot->imageData, screenShot->imageSize);
				draw_box();
			}
		}
		break;

		case CV_EVENT_LBUTTONDOWN : 
		{
			drawing_box = true;
			memcpy(screenShotRect->imageData, screenShot->imageData, screenShot->imageSize);
			if (mode == "1")
			{
				box = cvRect(x, y, 0, 0);
			}
			else if ( mode == "2" )
			{
				box = cvRect(x-(blockSize-1)/2, y-(blockSize-1)/2, blockSize, blockSize);
			}
		}
		break;

		case CV_EVENT_LBUTTONUP: 
		{
			drawing_box = false;
			
			if (box.width < 0) 
			{
				box.x += box.width;
				box.width *= -1;
			}
			
			if (box.height < 0) 
			{
				box.y += box.height;
				box.height *= -1;
			}
			
			draw_box();
		}
		break;
		
		case CV_EVENT_RBUTTONDOWN:
		{
			drawing_box = false;
			
			box.x 		= -1;
			box.y 		= -1;
			box.width 	= -1;
			box.height 	= -1;
			
			memcpy(screenShotRect->imageData, screenShot->imageData, screenShot->imageSize);
		}
		break;
	}
}

void calculateResult(int event, int x, int y, int flags, void* param)
{		
	switch( event ) 
	{
		case CV_EVENT_LBUTTONDOWN : 
		{
			if (mode == "1")	//kmeans 
			{
				filter = new TextureFilter();
				filter->setColorSpace(CrCb);
				
				cout << "loading images" << endl;
				filter->loadImages(POSITIVE);
				filter->loadImages(NEGATIVE);

				filter->findCenters(NUM_POS_CLUSTERS, NUM_NEG_CLUSTERS, ALL_TOGETHER);
				filter->clusterPixels();

				filter->drawLabelsImage(imgLabels);
				filter->writeCentersToFile();
	
				delete filter;
				filter = NULL;
			}
			else if (mode == "2") //filtering
			{
				filter = new TextureFilter();
				filter->setColorSpace(CrCb);
				
				cout << "loading centers from file" << endl;
				filter->readCentersFromFile();
				filter->clusterPixels();
				
				cout << "loading patterns" << endl;
				filter->loadPatterns(POSITIVE);
				filter->loadPatterns(NEGATIVE);

				cout << "showing patterns" << endl;
				//filter->showPatterns(POSITIVE);
				//filter->showPatterns(NEGATIVE);
		
				cout << "calculating patterns" << endl;
				filter->calculatePatternHistograms();
	
				cout << "printing/visualizing patterns" << endl;
				filter->printPatternHistograms();
				//filter->visualizePatternHistograms();
								
				//~ TimeCalculator timer;
				//~ timer.startTimer();

                                IplImage* res = filter->calculateTexture(screenShot);
				//~ cout << "elapsed " << timer.endTimer() << endl;
			
				cvConvertScale(res, imgResult, 255.);
				
				cvMerge(imgResult, imgResult, imgResult, NULL, imgResultRect);
				cvReleaseImage(&res);
				
				delete filter;
				filter = NULL;
			}
		}
		break;
	}
	
	
}

int main(int argc, char* argv[]) {
	
	//Check number of arguments
	if(argc != 3)
	{
		cout << endl;
		cout << "Usage: ./trainTexture [mode] [camID]" << endl;
		cout << "mode : Mode of operation" << endl;
		cout << "		1 for kmeans" << endl;
		cout << "		2 for choosing patterns" << endl;
		cout << "camID : Camera ID" << endl;
		cout << "		-1 for Any camera" << endl;
		cout << "		0,1,2... for /dev/video#" << endl;
		cout << endl;
		cout << "Enter to exit... " << endl;
		
		cin.get();
		return 0;
	}
	mode = argv[1];
	cout << "Mode " << mode << endl;
	
	CvCapture* capture;	
	capture = cvCaptureFromCAM( atoi(argv[2]) );		
	cvSetCaptureProperty(capture , CV_CAP_PROP_FRAME_WIDTH , 640); 
	cvSetCaptureProperty(capture , CV_CAP_PROP_FRAME_HEIGHT , 480);
	
	frame = cvQueryFrame(capture);
	screenShot = cvCreateImage( cvGetSize(frame) , 8 , 3);
	screenShotRect = cvCreateImage( cvGetSize(frame) , 8 , 3);
	imgResult = cvCreateImage( cvGetSize(frame) , 8 , 1);
	imgResultRect = cvCreateImage( cvGetSize(frame) , 8 , 3);
	imgLabels = cvCreateImage( cvSize(256,256) , 8 , 1);
	
	//////////////////////////////////////////////////////
	//Create Ouput Image Windows and place them on screen
	//////////////////////////////////////////////////////
	cout << "Create Windows" << endl;
	
	cvNamedWindow("Source", CV_WINDOW_KEEPRATIO);
	cvMoveWindow("Source", 428 , 0);
	cvResizeWindow("Source", 500 , 375);

	cvNamedWindow("Result" , CV_WINDOW_KEEPRATIO);
	cvMoveWindow("Result", 1000, 500);
	cvResizeWindow("Result", 500 , 375);
	
	cvNamedWindow("ScreenShot" , CV_WINDOW_KEEPRATIO);
	cvMoveWindow("ScreenShot", 1000, 0);
	cvResizeWindow("ScreenShot", 500 , 375);
	//////////////////////////////////////////////////////	
	cvZero(screenShot);
	cvZero(screenShotRect);
	cvZero(imgResult);
	cvZero(imgResultRect);
	cvZero(imgLabels);
	
	cvSetMouseCallback("Result", calculateResult, NULL); 
	cvSetMouseCallback("ScreenShot", mouse_callback, (void*)screenShotRect);
	
	bool isOn = true;
	bool hasScreenShot = false;
	while( isOn )
	{
		while(true)
		{
			cvQueryFrame(capture);
			frame = cvQueryFrame(capture);
			
			cvShowImage("Source" , frame);
			cvShowImage("ScreenShot" , screenShotRect);
			
			if (mode == "1")
				cvShowImage("Result" , imgLabels);
			else if (mode == "2")
				cvShowImage("Result" , imgResultRect);
			
			int key = (cvWaitKey(5) & 255);
			
			if (key == 27)	//esc
			{
				isOn = false;
				break;
			}
			else if (key == 32)	//space
			{
				memcpy(screenShot->imageData, frame->imageData, frame->imageSize);
				memcpy(screenShotRect->imageData, frame->imageData, frame->imageSize);
				hasScreenShot = true;
			}
			else if (key == 'z') //save positives
			{
				if (mode == "1")
					save(POSITIVE_PATH);
				else if (mode == "2")
					save(POSITIVE_PATTERN_PATH);
			}
			else if (key == 'x')	//save to negatives
			{
				if (mode == "1")
					save(NEGATIVE_PATH);
				else if (mode == "2")
					save(NEGATIVE_PATTERN_PATH);
			}

		}
	}
	
	cvReleaseImage(&imgLabels);
	cvReleaseImage(&imgResult);
	cvReleaseImage(&imgResultRect);
	cvReleaseImage(&screenShot);
	cvReleaseImage(&screenShotRect);
	cvReleaseCapture(&capture);
	
	cvDestroyAllWindows();
	return 0;
}
