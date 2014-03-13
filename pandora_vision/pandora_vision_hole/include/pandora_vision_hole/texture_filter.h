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

#ifndef TEXTUREFILTER_H
#define TEXTUREFILTER_H

#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <stdlib.h>

#include "pattern.h"
#include "ros/ros.h"
#define POSITIVE 	0
#define NEGATIVE 	1
#define ALL		 	2

#define ALL_TOGETHER 	0
#define ONE_BY_ONE		1

#define CENTERS_PATH "walls/centers/"
#define POSITIVE_PATH "walls/positive/"
#define NEGATIVE_PATH "walls/negative/"
#define POSITIVE_FRAGMENT_PATH "walls/positive_fragments/"
#define NEGATIVE_FRAGMENT_PATH "walls/negative_fragments/"
#define POSITIVE_PATTERN_PATH "walls/positive_patterns/"
#define NEGATIVE_PATTERN_PATH "walls/negative_patterns/"

#define EXTENSION ".png"

#define MAX_ITERATIONS 10
#define ACCURACY 1
#define ATTEMPTS 10

#define EUCLIDEAN				0
#define EUCLIDEAN_SIMPLE		1

#define CHI_SQUARE				0
#define BHATTACHARYYA			1
#define COSINE_DISTANCE			2

#define WIDTH					640
#define HEIGHT					480
#define BLOCK_HEIGHT			31
#define BLOCK_WIDTH				31
#define STEP_X					5
#define STEP_Y					5

#define	BGR						0
#define YCrCb					1
#define CrCb					2

#define VOTE_NUM				4

using namespace cv;

namespace pandora_vision
{
  struct region
  {
    int i;	//height coordinate of region's center
    int j;	//width coordinate of region's center
    
    uintptr_t id;
    uintptr_t parent_id;
    uintptr_t child_id;
    
    histogram parent_hist;
    histogram child_hist;
    histogram hist;
  };
   
  class TextureFilter
  { 
    private:
      int colorSpace_;
      int dimension_;
      int nPositiveClusters_;						//Number of centers returned by Kmeans, for positive patterns
      int nNegativeClusters_;						//Number of centers returned by Kmeans. for negative patterns
      int nClusters_;								//Total number of different centers, for ALL patterns
      int nRegions_;	
      int nRegionsRows_;
      int nRegionsCols_;	
      
      string path;
      string centersPath;
      string positivePath;
      string negativePath;
      string positivePatternPath;
      string negativePatternPath;
          
    public:
    
      IplImage* wallHistogram;
      IplImage* imgColorLabels;
      int colorLabels[256][256][VOTE_NUM];		// colorLabels [cr][cb]
      region* regionContainer;
      
      vector<IplImage*> positiveImages;			//list that contain positive pattern images
      vector<IplImage*> negativeImages;			//list that contain negative pattern images
      
      CvPoint3D32f* positiveColors3d;				//array that contain 3d positive colors
      CvPoint3D32f* negativeColors3d;				//array that contain 3d negative colors
      
      Mat_<cv::Vec2f>* centroidsPositive2d;		//Mat that holds positive 2d color centroids
      Mat_<cv::Vec3f>* centroidsPositive3d;		//Mat that holds positive 3d color centroids
      Mat_<cv::Vec2f>* centroidsNegative2d;		//Mat that holds negative 2d color centroids
      Mat_<cv::Vec3f>* centroidsNegative3d;		//Mat that holds negative 3d color centroids
      
      Mat_<cv::Vec2f>* centroids2d;				//Mat that holds 2d color centroids
      Mat_<cv::Vec3f> centroids3d	;				//Mat that holds 2d color centroids
      
      CvPoint3D32f* colors3d;						//array that contain all 3d colors	
      
      vector<Pattern*> patterns;					//list that contains all patterns
      vector<Pattern*> positivePatterns;			//list that contains only positive patterns
      vector<Pattern*> negativePatterns;			//list that contains only negative patterns
          
      TextureFilter();
      ~TextureFilter();
      
      void setPath(string str);

      void loadImages(int type);
      void loadPatterns(int type);
      
      void fragmentPositiveImages(int width, int height, CvScalar color);
      void fragmentNegativeImages(int width, int height, CvScalar color);
      void fragmentImage(IplImage* img, int width, int height, string path, string id, CvScalar color);
      
      void readCentersFromFile();
      void writeCentersToFile();
      void printCenters();
      void printCenters(int type);
      
      void findCenters(int positiveClusters, int negativeClusters, int method);	
      void findPatternCenters(int numberOfCenters, int type, int method);
      void findPatternCenters3d_1(int numberOfCenters, int type);
      void findPatternCenters3d_2(int numberOfCenters, int type);
      void findPatternCenters2d_1(int numberOfCenters, int type);
      void findPatternCenters2d_2(int numberOfCenters, int type);
      void mergeCenters();
      void clusterPixels();
      void drawLabelsImage( IplImage* dst );
      
      void calculatePatternHistograms();
      void visualizePatternHistograms();
      void printPatternHistograms();
      
      void setColorSpace(int colorSpace);
      int getNumberOfBins();
      
      void showImages(int type);
      void showPatterns(int type);
      void showSegmentedImage(IplImage* input);
          
      bool comparePoints(CvPoint3D32f pt1, CvPoint3D32f pt2);
      bool comparePoints(CvPoint2D32f pt1, CvPoint2D32f pt2);
      double calculatePointDistance(CvPoint3D32f pt1, CvPoint3D32f pt2, int type);
      double calculatePointDistance(CvPoint2D32f pt1, CvPoint2D32f pt2, int type);
      
      IplImage* calculateTexture(IplImage* img);
      double calculateTextureArea(IplImage* imgInput, int pos_j, int pos_i, int method);
      
      void createRegions();
      void calculateRegionHistogram(IplImage* imgInput);
      double calculateTextureRegion(IplImage* imgInput, int regionId, int method);
      
      CvScalar BGR2YCrCb(CvScalar bgr);
      CvScalar YCrCb2BGR(CvScalar ycrcb);
          
      void calculateWallHistogram();
      void filterWalls(IplImage* dst, IplImage* src, float lowThresh);
  };
}
#endif
