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
	
#include "pandora_vision_hole/texture_filter.h"
	
namespace pandora_vision
{

  /**
   * Constructor
   */
  TextureFilter::TextureFilter()
  {	
    ROS_INFO("Created TextureFilter instance"); 	
    
    this->dimension_ = 3;
    this->colorSpace_ = BGR;
    this->nClusters_ = -1;
    this->nPositiveClusters_ = -1;
    this->nNegativeClusters_ = -1;
    
    this->positiveColors3d = NULL;		
    this->negativeColors3d = NULL;
    this->colors3d = NULL;
    
    this->centroidsPositive2d = NULL;
    this->centroidsPositive3d = NULL;
    this->centroidsNegative2d = NULL;
    this->centroidsNegative3d = NULL;
    this->centroids2d = NULL;
    //this->centroids3d = NULL;
    
    this->wallHistogram = NULL;
    this->imgColorLabels = NULL;
    
    this->path = "";
    this->centersPath = CENTERS_PATH;
    this->positivePath = POSITIVE_PATH;
    this->negativePath = NEGATIVE_PATH;
    this->positivePatternPath = POSITIVE_PATTERN_PATH;
    this->negativePatternPath = NEGATIVE_PATTERN_PATH;
    
    this->regionContainer = NULL;
    
    ROS_INFO("[TextureFilter] : %s" ,path.c_str());
    ROS_INFO("[TextureFilter] : %s" ,centersPath.c_str());
    ROS_INFO("[TextureFilter] : %s" ,positivePath.c_str()); 
    ROS_INFO("[TextureFilter] : %s" , negativePath.c_str()); 
    ROS_INFO("[TextureFilter] : %s" , positivePatternPath.c_str());
    ROS_INFO("[TextureFilter] : %s" , negativePatternPath.c_str()); 
  }

  /**
   * Destructor
   */
  TextureFilter::~TextureFilter()
  {				
    ROS_INFO("Destroying TextureFilter instance");
    
    // Release all positive images
    for (int l=0; l<positiveImages.size(); l++)
    {
      cvReleaseImage(&positiveImages.at(l));
      positiveImages.at(l) = NULL;
    }
    positiveImages.clear();
    
    // Release all negative images
    for (int l=0; l<negativeImages.size(); l++)
    {
      cvReleaseImage(&negativeImages.at(l));
      negativeImages.at(l) = NULL;
    }
    negativeImages.clear();
    
    //Release all patterns
    for (int l=0; l<patterns.size(); l++)
    {
      Pattern* current = patterns.at(l);
      current->destroyPattern();
      delete current;
      current = NULL;
    }
    patterns.clear();
    positivePatterns.clear();
    negativePatterns.clear();
      
    if (positiveColors3d != NULL)
    {
      delete positiveColors3d;
      positiveColors3d = NULL;
    }
    
    if (negativeColors3d != NULL)
    {
      delete negativeColors3d;
      negativeColors3d = NULL;
    }
      
    if (colors3d != NULL)
    {
      delete colors3d;
      colors3d = NULL;
    }
    
    delete (this->centroidsPositive2d);
    this->centroidsPositive2d = NULL;
    
    delete this->centroidsPositive3d;
    this->centroidsPositive3d = NULL;
    
    delete this->centroidsNegative2d;
    this->centroidsNegative2d = NULL;
    
    delete this->centroidsNegative3d;
    this->centroidsNegative3d = NULL;
    
    delete this->centroids2d;
    this->centroids2d = NULL;
    
    //this->centroids3d->release();
    //delete this->centroids3d;
    //this->centroids3d = NULL;
    
    if (wallHistogram != NULL)
      cvReleaseImage(&wallHistogram);
      
    ///delete regions
    if (regionContainer != NULL)
    {
      for (int q=0; q<nRegions_; q++)
      {
        delete regionContainer[q].hist.values;
        delete regionContainer[q].hist.valuesNorm;
        delete regionContainer[q].hist.values;
        delete regionContainer[q].hist.valuesNorm;
        delete regionContainer[q].hist.values;
        delete regionContainer[q].hist.valuesNorm;
      }
      delete regionContainer;
    }
  }

  /**
   * Sets file path for trained textures.
   * @param str
   */
  void TextureFilter::setPath(std::string str)
  {	
    ///Path
    path = str;
    
    ///Centers path
    centersPath = path + CENTERS_PATH;
    
    ///Positive path
    positivePath = path + POSITIVE_PATH;
    
    ///Negative path
    negativePath = path + NEGATIVE_PATH;
    
    ///Positive Pattern path
    positivePatternPath = path + POSITIVE_PATTERN_PATH;
    
    ///Negative Pattern path
    negativePatternPath = path + NEGATIVE_PATTERN_PATH;
    
    ROS_INFO("[TextureFilter] : Setting new path"); 
    ROS_INFO("[TextureFilter] : %s", path.c_str()); 
    ROS_INFO("[TextureFilter] : %s",centersPath.c_str()); 
    ROS_INFO("[TextureFilter] : %s",positivePath.c_str()); 
    ROS_INFO("[TextureFilter] : %s", negativePath.c_str()); 
    ROS_INFO("[TextureFilter] : %s", positivePatternPath.c_str()); 
    ROS_INFO("[TextureFilter] : %s", negativePatternPath.c_str()); 
    
  }

  /**
   * Returns number of histogram bins.
   * @return
   */
  int TextureFilter::getNumberOfBins()
  {
    return this->nClusters_;
  }

  /**
   * Sets used colorspace of texture.
   * @param colorSpace
   */
  void TextureFilter::setColorSpace(int colorSpace)
  {
    if ( (colorSpace != BGR) && (colorSpace != YCrCb) && (colorSpace != CrCb) )
    {
      ROS_ERROR("[TextureFilter] Invalid colorSpace!!! Use BGR , YCrCb or CrCb"); 
      ROS_ERROR("[TextureFilter] Setting colorSpace to BGR"); 
      this->colorSpace_ = BGR;
      return;
    }
    
    this->colorSpace_ = colorSpace;
    if (colorSpace == CrCb)
      this->dimension_ = 2;
    else
      this->dimension_ = 3;
  }

  /**
   * Calculates the histogram of a wall segment.
   */
  void TextureFilter::calculateWallHistogram()
  {	
    /// initialize colorGrid
    float colorGrid[256][256];
    for (int i=0; i<256; i++)
    {
      for (int j=0; j<256; j++)
      {
        colorGrid[i][j] = 0;
      }
    } 
    
    int max = 0;
    int cr_index=0;
    int cb_index=0;
    for (int k=0; k<positiveImages.size(); k++)
    {
      IplImage* current 	= positiveImages.at(k);		
      int width 			= current->width;
      int height 			= current->height;
      int step    		= current->widthStep/sizeof(uchar);
      int channels    	= current->nChannels;
      uchar* data			= (uchar*)current->imageData;
      
      ///smooth images???
      cvSmooth( current , current , CV_GAUSSIAN , 3, 3);
      
      for (int i=0; i<height; i++)
      {
        for (int j=0; j<width; j++)
        {			
          int y  = data[i*step + j*channels + 0];	
          int cr = data[i*step + j*channels + 1];
          int cb = data[i*step + j*channels + 2];
      
          if ( y > 200 || y < 40 )
            continue;
      
          double distance = pow(( cr-21),2 ) + pow((cb-43),2);
          if ( distance < 32)
            continue;	

          colorGrid[cr][cb]++;
      
          if (colorGrid[cr][cb] > max)
          {
            max = colorGrid[cr][cb];
            cr_index=cr;
            cb_index=cb;
          }
        }
      }
    }
    
    for (int i=0; i<256; i++)
    {
      for (int j=0; j<256; j++)
      {
        colorGrid[i][j] = colorGrid[i][j]/max;
      }
    } 
    
    wallHistogram = cvCreateImage( cvSize(256,256) , 8 , 1 );
    int height = wallHistogram->height;
    int width = wallHistogram->width;
    int step = wallHistogram->widthStep/sizeof(uchar);
    uchar* data	= (uchar*)wallHistogram->imageData;
    
    for (int i=0; i<height; i++)
    {
      for (int j=0; j<width; j++)
      {
        data[i*step + j] = (int)(colorGrid[i][j]*255);
      }
    }
    
    //cvDilate(wallHistogram,wallHistogram,NULL,15);
    
    cvSaveImage("wallHistogram.jpg", wallHistogram);
  }

  /**
   * Filters wall regions using histograms, through backprojection method.
   * @param dst
   * @param src
   * @param lowThresh
   */
  void TextureFilter::filterWalls(IplImage* dst, IplImage* src, float lowThresh)
  {
    int width 			= src->width;
    int height 			= src->height;
    
    int channels    	= src->nChannels;
    uchar* dataSrc		= (uchar*)src->imageData;
    uchar* data			= (uchar*)dst->imageData;
    uchar* dataHist		= (uchar*)wallHistogram->imageData;
    
    int step    		= src->widthStep/sizeof(uchar);
    int dstStep			= dst->widthStep/sizeof(uchar);
    int histStep 		= wallHistogram->widthStep/sizeof(uchar);
    
    ///smooth input image???
    cvSmooth( src , src , CV_GAUSSIAN , 3, 3);
      
    for (int i=0; i<height; i++)
    {
      for (int j=0; j<width; j++)
      {
        int crSrc = dataSrc[i*step + j*channels + 1];
        int cbSrc = dataSrc[i*step + j*channels + 2];
        int val = dataHist[crSrc*histStep + cbSrc];
            
        if (val < lowThresh)
          val = 0;
        //val = 5 * val;
        //if (val > 255) 
        //	val = 255;
        //data[i*dstStep+j] = val ;
          
        /*
          else
            val = 255;
        }
        */ 
        data[i*dstStep+j] = val;		
      }
      std::cout << std::endl;
    }
  }


  /**
   * Histogram visualizer for debuging purposes.
   */
  void TextureFilter::visualizePatternHistograms()
  {
    for (int i=0; i<positivePatterns.size(); i++)
    {
      Pattern* pattern = positivePatterns.at(i);
      
      if ( (colorSpace_ == BGR) || (colorSpace_ == YCrCb) )
        pattern->visualizeHistogram(colors3d);
      else if (colorSpace_ == CrCb)
        pattern->visualizeHistogram(centroids2d);
      else
        ROS_ERROR("[Pattern] : Wrong colorspace at histogram visualization");
    }
    
    for (int i=0; i<negativePatterns.size(); i++)
    {
      Pattern* pattern = negativePatterns.at(i);
      
      if ( (colorSpace_ == BGR) || (colorSpace_ == YCrCb) )
        pattern->visualizeHistogram(colors3d);
      else if (colorSpace_ == CrCb)
        pattern->visualizeHistogram(centroids2d);
      else
        ROS_INFO("[Pattern] : Wrong colorspace at histogram visualization"); 
    }
  }
    

  /**
   * Prints histograms on console.
   */
  void TextureFilter::printPatternHistograms()
  {
    for (int i=0; i<patterns.size(); i++)
    {
      Pattern* pattern = patterns.at(i);
      for (int k=0; k<pattern->m_hist.bins; k++)
        std::cout << pattern->m_hist.valuesNorm[k] << " ";
    }
  }

  /**
   * Loads wall segments as images.
   * @param type
   */
  void TextureFilter::loadImages(int type)
  {
    bool isPositive = false;
    isPositive = (type == POSITIVE);
    
    std::string path;
    if (isPositive)
      path = positivePath;
    else
      path = negativePath;
    
    std::stringstream strStream;
    std::stringstream folderStream;
    bool folderExists = true;
      
    /// Read Patterns recursively
    int i=0;
    int j=0;
    while(true)
    {
      ///get in i positive or negative folder 
      strStream.str("");
      strStream << path << i << "/"; 
      
      ROS_INFO_STREAM(strStream.str()); 
      
      j=0;
      while (true)
      {
        ///load file
        folderStream.str("");
        folderStream << strStream.str().c_str() << j << EXTENSION;

        IplImage* frame = 0;
          ROS_INFO("opening %s",folderStream.str().c_str() ); 
        frame = cvLoadImage( folderStream.str().c_str());
            
        if (!frame)
        {
          /// if we can't find 0.png in current folder,
          /// assume that there is no such folder and break
          if (j == 0)
            folderExists = false;
            
          cvReleaseImage(&frame);
          break;
        }
        
        if (this->colorSpace_ == BGR)
        {			
          if (isPositive)
            positiveImages.push_back(frame);	
          else
            negativeImages.push_back(frame);	
        }
        else if ( (this->colorSpace_ == YCrCb) || (this->colorSpace_ == CrCb) )
        {
          IplImage* imgYCrCb = cvCreateImage( cvGetSize(frame) , IPL_DEPTH_8U , 3);
          cvCvtColor(frame, imgYCrCb, CV_BGR2YCrCb);
          
          if (isPositive)
            positiveImages.push_back(imgYCrCb);	
          else
            negativeImages.push_back(imgYCrCb);
            
          cvReleaseImage(&frame);
        }
        
        j++;
      }
      
      ///change folder
      if ( folderExists )
        i++;
      else
      {
        folderExists = true;
        break;
      }
    }		
  }

  /**
   * Loads wall patterns as images.
   * @param type
   */
  void TextureFilter::loadPatterns(int type)
  {
    bool isPositive = (type == POSITIVE);
    
    std::string path;
    if (isPositive)
      path = positivePatternPath;
    else
      path = negativePatternPath;
      
    std::stringstream strStream;
    std::stringstream folderStream;
    bool folderExists = true;
      
    /// Read Patterns recursively
    int i=0;
    int j=0;
    while(true)
    {
      ///get in i positive or negative folder 
      strStream.str("");
      strStream << path << i << "/"; 
      
      j=0;
      while(true)
      { 
        ///load file
        folderStream.str("");
        folderStream << strStream.str().c_str() << j << EXTENSION;
        
        ROS_INFO_STREAM(folderStream.str()); 
      
        IplImage* frame = 0;
        frame = cvLoadImage( folderStream.str().c_str() );
            
        if (!frame)
        {
          /// if we can't find 0.png in current folder,
          /// assume that there is no such folder and break
          if (j == 0)
            folderExists = false;
            
          cvReleaseImage(&frame);
          break;
        }
              
        if (isPositive)
        {
          Pattern* pattern = new Pattern();
          pattern->createPattern(frame,POSITIVE);
          positivePatterns.push_back(pattern);
          patterns.push_back(pattern);
        }		
        else
        {
          Pattern* pattern = new Pattern();
          pattern->createPattern(frame,NEGATIVE);
          negativePatterns.push_back(pattern);
          patterns.push_back(pattern);
        }
          
        cvReleaseImage(&frame);
        
        j++;
      }
      
      ///change folder
      if ( folderExists )
        i++;
      else
      {
        folderExists = true;
        break;
      }
    }
  }

  /**
   * Dislays the filtered output. The output includes regions which
   * possibly represent true walls.
   * @param input
   */
  void TextureFilter::showSegmentedImage(IplImage* input)
  {
    
    
    if (this->dimension_ == 3)
    {
      CvPoint3D32f* colors = new CvPoint3D32f[this->nClusters_];
      for (int i=0; i<this->nClusters_; i++)
      {
        colors[i].x = centroids3d(i,0)[0];
        colors[i].y = centroids3d(i,0)[1];
        colors[i].z = centroids3d(i,0)[2];
      }
      
      IplImage* imgSegmented = (IplImage*)cvClone(input);
      
      int height     			= input->height;
      int width     			= input->width;
      int step      			= input->widthStep/sizeof(uchar);
      int channels  			= input->nChannels;
      uchar* data   			= (uchar*)input->imageData;
      uchar* dataSegm			= (uchar*)imgSegmented->imageData;
                        
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
          for (int k=0; k<nClusters_; k++)
          {
            CvPoint3D32f pt2 = colors3d[k];
            distance = calculatePointDistance(pt1, pt2, EUCLIDEAN);
            
            if ( (minDistance == -1) || (minDistance > distance) )
            {
              minDistance = distance;
              index = k;
            }
          }
          
          dataSegm[i*step+j*channels+0] = (uchar)colors3d[index].x;
          dataSegm[i*step+j*channels+1] = (uchar)colors3d[index].y;
          dataSegm[i*step+j*channels+2] = (uchar)colors3d[index].z;
        }
      }
      
      if (this->colorSpace_ == BGR)
      {
        cvShowImage("Segmented" , imgSegmented);
        cvWaitKey(0);
      }
      else if (this->colorSpace_ == YCrCb)
      {
        IplImage* imgBGR = cvCreateImage( cvGetSize(imgSegmented) , IPL_DEPTH_8U , 3);
        cvCvtColor(imgSegmented, imgBGR, CV_YCrCb2BGR);
        
        cvShowImage("Segmented" , imgSegmented);
        cvShowImage("Segmented_BGR" , imgBGR);
        cvWaitKey(0);
      }
      
      delete colors;
      cvReleaseImage(&imgSegmented);
    }
    else if (this->dimension_ == 2)
    {
      CvPoint2D32f* colors = new CvPoint2D32f[this->nClusters_];
      for (int i=0; i<this->nClusters_; i++)
      {
        colors[i].x = (*centroids2d)(i,0)[0];
        colors[i].y = (*centroids2d)(i,0)[1];
      }
      
      IplImage* imgSegmented = (IplImage*)cvClone(input);
      
      int height     			= input->height;
      int width     			= input->width;
      int step      			= input->widthStep/sizeof(uchar);
      int channels  			= input->nChannels;
      uchar* data   			= (uchar*)input->imageData;
      uchar* dataSegm			= (uchar*)imgSegmented->imageData;
                        
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          CvPoint2D32f pt1;
          pt1.x = data[i*step+j*channels+1];
          pt1.y = data[i*step+j*channels+2];
          
          double distance = 0;
          double minDistance = -1;
          int index = 0;
          for (int k=0; k<nClusters_; k++)
          {
            CvPoint2D32f pt2 = colors[k];					
            distance = calculatePointDistance(pt1, pt2, EUCLIDEAN_SIMPLE);
            
            if ( (minDistance == -1) || (minDistance > distance) )
            {
              minDistance = distance;
              index = k;
            }
          }
          
          dataSegm[i*step+j*channels+0] = 127;
          dataSegm[i*step+j*channels+1] = (uchar)colors[index].x;
          dataSegm[i*step+j*channels+2] = (uchar)colors[index].y;
        }
      }
      
      if (this->colorSpace_ == BGR)
      {
        cvShowImage("Segmented" , imgSegmented);
        //cvWaitKey(0);
      }
      else if ( (this->colorSpace_ == YCrCb) || (this->colorSpace_ == CrCb) )
      {
        IplImage* imgBGR = cvCreateImage( cvGetSize(imgSegmented) , IPL_DEPTH_8U , 3);
        cvCvtColor(imgSegmented, imgBGR, CV_YCrCb2BGR);
        
        cvShowImage("Segmented" , imgSegmented);
        cvShowImage("Segmented_BGR" , imgBGR);
        //cvWaitKey(0);
      }
      
      cvReleaseImage(&imgSegmented);
    }
  }

  /**
   * Fragments positive input images to be used as patterns.
   * @param width
   * @param height
   * @param color
   */
  void TextureFilter::fragmentPositiveImages(int width, int height, CvScalar color)
  {
    ROS_INFO("fragmenting positive images");
    std::stringstream strStream;
    for (int i=0; i<positiveImages.size(); i++)
    {		
      strStream.str("");
      strStream << i;
      fragmentImage(positiveImages.at(i), width, height, POSITIVE_FRAGMENT_PATH, strStream.str(), color);
    }	
  }

  /**
   * Fragments negative input images to be used as patterns.
   * @param width
   * @param height
   * @param color
   */
  void TextureFilter::fragmentNegativeImages(int width, int height, CvScalar color)
  {
    ROS_INFO("fragmenting negative images"); 
    std::stringstream strStream;
    for (int i=0; i<negativeImages.size(); i++)
    {
      strStream.str("");
      strStream << i;
      IplImage* currentImage = negativeImages.at(i);
      fragmentImage(currentImage, width, height, NEGATIVE_FRAGMENT_PATH, strStream.str(), color);
    }
  }

  /**
   * Fragments a given image.
   * @param imgInput
   * @param width
   * @param height
   * @param path
   * @param id
   * @param color
   */
  void TextureFilter::fragmentImage(IplImage* imgInput, int width, int height, std::string path, std::string id, CvScalar color)
  {	
    int counter = 0;
    IplImage* img = NULL;
    if (this->colorSpace_ != BGR)
    {
      img = cvCreateImage( cvGetSize(imgInput) , IPL_DEPTH_8U , 3);
      cvCvtColor(imgInput, img, CV_YCrCb2BGR);
    }
    
    /// take non-overlapping blocks
    int widthImg = img->width;
    int heightImg = img->height;

    for (int i=0; i<(heightImg-height); i+=height)
    {
      for (int j=0; j<(widthImg-width); j+=width)
      {			
        CvRect currentRect = cvRect( j, i, width, height );
        
        /// sets the Region of Interest
        /// the rectangle area has to be INSIDE the image 
        cvSetImageROI(img, currentRect);
        
        /// create destination image
        /// cvGetSize will return the width and the height of ROI
        IplImage *imgNew = cvCreateImage(cvGetSize(img), img->depth, img->nChannels);
        
        /// copy subimage
        cvCopy(img, imgNew, NULL);
        
        /// always reset the Region of Interest
        cvResetImageROI(img);
        
        /// save the new image to the specified path
        std::stringstream strStream;
        strStream.str("");
        strStream << path << id << "_" << counter << EXTENSION;
        
        /// check if the current image contains the
        /// invalid color given as argument		
        bool isValid	= false;		
        int step		= imgNew->widthStep/sizeof(uchar);
        int channels	= imgNew->nChannels;
        uchar* data = (uchar*)imgNew->imageData;
        for (int i=0; i<height; i++)
        {
          for (int j=0; j<width; j++)
          {
            isValid = false;
            isValid = (data[i*step + j*channels + 0] != color.val[0]);
            isValid = isValid || (data[i*step + j*channels + 1] != color.val[1]);
            isValid = isValid || (data[i*step + j*channels + 2] != color.val[2]);
            
            if (!isValid)
              break;
          }
          if (!isValid)
            break;
        }
        cvShowImage("11!" , imgNew);
        cvWaitKey(0);
        if (isValid)
          cvSaveImage( strStream.str().c_str() , imgNew );
        
        cvReleaseImage(&imgNew);
        
        counter++;
        
        /// show fragment of image
        //cvShowImage("fragment" , imgNew);
        //cvWaitKey(0);
        
        /// draw a rectangel around the fragment
        /// on the original image
        //cvRectangle(img, cvPoint(j+1,i+1) , cvPoint(j+width-1,i+height-1), cvScalarAll(1), 1, 8, 0);
      }
    }
    /// show original image
    //cvShowImage("original" , img);
    //cvWaitKey(0);
    if (img != NULL)
      cvReleaseImage(&img);
  }

  /**
   * Writes histogram centers to a file, for later node initialization
   */
  void TextureFilter::writeCentersToFile()
  {
    std::stringstream path;
    path << CENTERS_PATH << "CrCb.txt";
    
    std::ofstream outfile(path.str().c_str());
    
    if ( outfile.is_open() )
    {
    
      for (int i=0; i<nClusters_; i++)
      {
        outfile << (*centroids2d)(i,0)[0] << " " << (*centroids2d)(i,0)[1];
        if (i != nClusters_ - 1)
          outfile << std::endl;
      }	 
      
      outfile.close();
      
      ROS_INFO("centers written to file"); 
    }
    else
      ROS_ERROR("Could not open file to write centers"); 
  }

  /**
   * Reads histogram centers from the file.
   */
  void TextureFilter::readCentersFromFile()
  {
    //open file
    std::ifstream infile;
    
    std::stringstream strStream;
    strStream.str("");
    strStream << centersPath;
    
    if (this->colorSpace_ == BGR)
      strStream << "BGR.txt";
    else if (this->colorSpace_ == YCrCb)
      strStream << "YCrCb.txt";
    else
      strStream << "CrCb.txt";

    vector<CvPoint3D32f> colorVec3D;
    vector<CvPoint2D32f> colorVec2D;
    
    infile.open(strStream.str().c_str());
    if ( infile.is_open() )
    {		
      while(true)
      {
        if ( !infile.eof() )
        {
          std::string str;
          getline(infile, str); //Read lines from file and create std::string array
          
          if (str.length() == 0)
            break;
          
          if (this->dimension_ == 3)
          {
            CvPoint3D32f pt;
            size_t found1 = str.find_first_of(" ");
            size_t found2 = str.find(" ", found1+1);
            
            if ( (found1 == std::string::npos) || (found2 == std::string::npos ) )
              break;
            
            std::string str1 = str.substr(0,found1);
            std::string str2 = str.substr(found1+1,found2-found1-1);
            std::string str3 = str.substr(found2+1,str.length()-found2);
              
            pt.x = atof(str1.c_str());
            pt.y = atof(str2.c_str());
            pt.z = atof(str3.c_str());
            
            colorVec3D.push_back(pt);
          }
          else
          {
            CvPoint2D32f pt;
            size_t found1 = str.find(" ");
            
            if ( found1 == std::string::npos )
              break;
            
            std::string str1 = str.substr(0,found1);
            std::string str2 = str.substr(found1+1,str.length()-found1);
              
            pt.x = atof(str1.c_str());
            pt.y = atof(str2.c_str());
                      
            colorVec2D.push_back(pt);
          }
        }
        else
          break;
      }
        
      infile.close();
    }
    else
    {
      ROS_ERROR("could not load file to read centers ");
      ROS_ERROR_STREAM(strStream.str());
    }
      
    
      
    bool haveAllChannels = (this->colorSpace_ == BGR) || (this->colorSpace_ == YCrCb);
    int totalColors;
    if ( haveAllChannels ) 
    {
      if ( this->colors3d != NULL )
      {
        delete this->colors3d;
        this->colors3d = NULL;
      }
      
      totalColors = colorVec3D.size();
      this->colors3d = new CvPoint3D32f[totalColors];
      for (int i=0; i<totalColors; i++)
        colors3d[i] = colorVec3D.at(i);
        
      this->nClusters_ = totalColors;
    }
    else  //colorSpace CrCb
    {			
      totalColors = colorVec2D.size();		
      this->centroids2d = new Mat_<cv::Vec2f>(totalColors, 1, CV_32FC2);
      
      for (int i=0; i<totalColors; i++)
      {
        (*centroids2d)(i,0)[0] = colorVec2D.at(i).x;
        (*centroids2d)(i,0)[1] = colorVec2D.at(i).y;
      }
    
      this->nClusters_ = totalColors;
    }	
    
    printCenters();
  }

  /**
   * Calculates histogram centers.
   * @param positiveClusters
   * @param negativeClusters
   * @param method
   */
  void TextureFilter::findCenters(int positiveClusters, int negativeClusters, int method)
  {
    if (positiveClusters > 0)
      findPatternCenters(positiveClusters, POSITIVE, method);
    
    //printCenters(POSITIVE);
    //cout << "---------" << endl;
    //cin.get();
    
    if (negativeClusters > 0)
      findPatternCenters(negativeClusters, NEGATIVE, method);
      
    //printCenters(NEGATIVE);
    //cout << "---------" << endl;
    //cin.get();
      
    ///Step 4 - Keep unique centers	
    ROS_INFO("merging"); 
    mergeCenters();	
            
    printCenters();
    //cout << "---------" << endl;
    //cin.get();
  }

  /**
   * Calculates histogram centers of wall segments.
   * @param nClusters
   * @param type
   * @param method
   */
  void TextureFilter::findPatternCenters(int nClusters, int type, int method)
  {
    switch (this->dimension_)
    {		
      case 2 :
        if (method == ONE_BY_ONE)
          findPatternCenters2d_1(nClusters, type);
        else //ALL_TOGETHER
          findPatternCenters2d_2(nClusters, type);
        break;
        
      case 3 :
      default:
        if (method == ONE_BY_ONE)
          findPatternCenters3d_1(nClusters, type);
        else //ALL_TOEGTHER
          findPatternCenters3d_2(nClusters, type);
    }
  }

  /**
   * Same thing as above, but for 2D centers.
   * @param nClustersPerImage
   * @param type
   */
  void TextureFilter::findPatternCenters2d_1(int nClustersPerImage, int type)
  {	
    bool isPositive = (type == POSITIVE);
    
    vector<IplImage*>* imageVector;
    if (isPositive)
      imageVector = &this->positiveImages;
    else
      imageVector = &this->negativeImages;
    
    int totalImages = imageVector->size();
    
    if (totalImages == 0)
    {
      ROS_ERROR("No images found"); 
      return;
    }
    
    /// find total number of points of either positive or negative pattern images
    int sampleCount = 0;
    for (int i=0; i<totalImages; i++)
      sampleCount += imageVector->at(i)->width * imageVector->at(i)->height;
      
    ///see if we have more clusters than points
    int nClusters = MIN(nClustersPerImage*totalImages, sampleCount);
        
    ///create Mat that holds the positive or negative K-means centroids
    Mat_<Vec2f>* centroids;
    if (isPositive)
    {
      this->centroidsPositive2d = new Mat_<cv::Vec2f>(nClusters, 1, CV_32FC2);
      centroids = this->centroidsPositive2d;
    }
    else
    {
      this->centroidsNegative2d = new Mat_<cv::Vec2f>(nClusters, 1, CV_32FC2);
      centroids = this->centroidsNegative2d;
    }
    
    for (int i=0; i<nClusters; i++)
      (*centroids)(i,0) = Vec2f(-1.0,-1.0);
      
    /// find centers of every image seperately
    int counter=0;
    for (int q=0; q<totalImages; q++)
    {
      /// find total number of points of either positive or negative pattern images
      int totalPoints = imageVector->at(q)->width * imageVector->at(q)->height;
    
      ///
      /// Create Mats
      ///
      cv::Mat_<cv::Vec2f> pointsTemp(totalPoints, 1, CV_32FC2);
      cv::Mat labels;
      cv::Mat_<cv::Vec2f> centroidsTemp2d(nClustersPerImage, 1, CV_32FC2);
          
      IplImage* currentImage  = imageVector->at(q);	
      int height     			= currentImage->height;
      int width     			= currentImage->width;
      int step      			= currentImage->widthStep/sizeof(uchar);
      int channels  			= currentImage->nChannels;
      uchar* data   			= (uchar*)currentImage->imageData;
            
      int inc=0;						
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          float x = (float)data[i*step+j*channels+1];
          float y = (float)data[i*step+j*channels+2];
              
          pointsTemp(inc,0) = cv::Vec2f(x,y);
                        
          inc++;
        }
      }	
      
      ///
      /// Execute Kmeans algorithm 
      ///
      ROS_INFO("time for kmeans"); 
      kmeans( pointsTemp, nClustersPerImage, labels, 
          TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, MAX_ITERATIONS, ACCURACY ),
          ATTEMPTS, 
          KMEANS_PP_CENTERS, 
          centroidsTemp2d );
      ROS_INFO("finished kmeans"); 
      
      for (int i=0; i<centroidsTemp2d.rows; i++)
      {
        float x = centroidsTemp2d(i,0)[0];
        float y = centroidsTemp2d(i,0)[1];
        
        (*centroids)(counter,0) = Vec2f(x,y);		
        
        {/*
        if (isPositive)
        {
          cout << centroidsTemp2d(i,0)[0] << " " << centroidsTemp2d(i,0)[1] << endl;
          cout << (*centroids)(counter,0)[0] << " " << (*centroids)(counter,0)[1] << endl;
          cout << (*this->centroidsPositive2d)(counter,0)[0] << " " << (*this->centroidsPositive2d)(counter,0)[1] << endl;
        }
        */}
        
        counter++;
      }
    }
    
    if (isPositive)
      this->nPositiveClusters_ = nClusters;
    else
      this->nNegativeClusters_ = nClusters;
  }


  /**
   * I guess the same? I dont know, refactoring! :D
   * @param nClusters
   * @param type
   */
  void TextureFilter::findPatternCenters2d_2(int nClusters, int type)
  {
    bool isPositive = (type == POSITIVE);
    
    vector<IplImage*>* imageVector;
    if (isPositive)
      imageVector = &this->positiveImages;
    else
      imageVector = &this->negativeImages;
    
    int totalImages = imageVector->size();
    
    if (totalImages == 0)
    {
      ROS_ERROR ("No images found"); 
      return;
    }
    
    /// find total number of points of either positive or negative pattern images
    int sampleCount = 0;
    for (int i=0; i<totalImages; i++)
      sampleCount += imageVector->at(i)->width * imageVector->at(i)->height;
      
    ///see if we have more clusters than points
    nClusters = MIN(nClusters, sampleCount);
    
    if (isPositive)
      ROS_INFO("Total positive clusters %d ", nClusters );
    else
      ROS_INFO("Total positive clusters %d ", nClusters );
          
    ///
    /// Create Mats
    ///
    cv::Mat_<cv::Vec2f> points(sampleCount, 1, CV_32FC2);
    cv::Mat labels;
    
    ///create Mat that holds the positive or negative K-means centroids
    Mat_<Vec2f>* centroids;
    if (isPositive)
    {
      this->centroidsPositive2d = new Mat_<cv::Vec2f>(nClusters, 1, CV_32FC2);
      centroids = this->centroidsPositive2d;
    }
    else
    {
      this->centroidsNegative2d = new Mat_<cv::Vec2f>(nClusters, 1, CV_32FC2);
      centroids = this->centroidsNegative2d;
    }
        
    ///initialize points * centroids
    for(int i = 0; i < sampleCount; i++)
      points(i,0) = cv::Vec2f(0.0,0.0);
      
    for (int i=0; i<nClusters; i++)
      (*centroids)(i,0) = Vec2f(-1.0,-1.0);
                
    /// fill Mat points with image data
    /// iterate through all images	
    int inc=0;		
    for (int q=0; q<imageVector->size(); q++)
    { 
      IplImage* currentImage  = imageVector->at(q);			
      int height     			= currentImage->height;
      int width     			= currentImage->width;
      int step      			= currentImage->widthStep/sizeof(uchar);
      int channels  			= currentImage->nChannels;
      uchar* data   			= (uchar*)currentImage->imageData;
                        
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          float x = (float)data[i*step+j*channels+1];
          float y = (float)data[i*step+j*channels+2];
              
          points(inc,0) = Vec2f(x,y);
                        
          inc++;
        }
      }
    }
      
    ///
    /// Execute Kmeans algorithm 
    ///
    ROS_INFO("time for kmeans"); 
    kmeans( points, nClusters, labels, 
        TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, MAX_ITERATIONS, ACCURACY ),
        ATTEMPTS, 
        KMEANS_PP_CENTERS, 
        *centroids );
    ROS_INFO("finished kmeans");
        
    if (isPositive)
      this->nPositiveClusters_ = nClusters;
    else
      this->nNegativeClusters_ = nClusters;
  }


  /**
   * Same in 3D
   * @param numberOfCenters
   * @param type
   */
  void TextureFilter::findPatternCenters3d_1(int numberOfCenters, int type)
  {
    bool isPositive = (type == POSITIVE);
    int K = numberOfCenters;
    
    vector<CvPoint3D32f> colors;
    vector<IplImage*>* imageVector;
    if (isPositive)
      imageVector = &this->positiveImages;
    else
      imageVector = &this->negativeImages;
    
    int totalImages = imageVector->size();
    
    if (totalImages == 0)
      return;
    
    /// find centers of every image seperately
    for (int m=0; m<totalImages; m++)
    {
      /// find total number of points of either positive or negative pattern images
      int totalPoints = imageVector->at(m)->width * imageVector->at(m)->height;
    
      ///
      /// Create Mats
      ///
      CvMat* matPoints = cvCreateMat( totalPoints, 1, CV_32FC3 );
      CvMat* matClusters = cvCreateMat( totalPoints, 1, CV_32SC1 );
      
      IplImage* currentImage  = imageVector->at(m);	
      int height     			= currentImage->height;
      int width     			= currentImage->width;
      int step      			= currentImage->widthStep/sizeof(uchar);
      int channels  			= currentImage->nChannels;
      uchar* data   			= (uchar*)currentImage->imageData;
            
      int pos=0;						
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          CvPoint3D32f pt;
          pt.x = (float)data[i*step+j*channels+0];
          pt.y = (float)data[i*step+j*channels+1];
          pt.z = (float)data[i*step+j*channels+2];
          
          matPoints->data.fl[pos*channels + 0] = pt.x;
          matPoints->data.fl[pos*channels + 1] = pt.y;
          matPoints->data.fl[pos*channels + 2] = pt.z;
          pos++;
        }
      }
        
      ///
      /// Execute Kmeans algorithm 
      ///
      cvKMeans2( matPoints, K, matClusters, 
            cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, MAX_ITERATIONS, ACCURACY ), 
            ATTEMPTS, 0, cv::KMEANS_PP_CENTERS);
    
      ///			
      /// Find centers of clusters
      /// Because opencv refuses to give me results
      ///
      CvPoint3D32f* centers = new CvPoint3D32f[K];	
      int* counter = new int[K];
      for (int i=0; i<K; i++)
      {
        centers[i].x = 0;
        centers[i].y = 0;
        centers[i].z = 0;
        counter[i]=0;
      }
    
      int chnl = 3;
      for ( int i=0; i<totalPoints; i++ )
      {
        int cluster_idx = matClusters->data.i[i];
        centers[cluster_idx].x += matPoints->data.fl[i*chnl+0];
        centers[cluster_idx].y += matPoints->data.fl[i*chnl+1];
        centers[cluster_idx].z += matPoints->data.fl[i*chnl+2];
        counter[cluster_idx]++;
      }
    
      /// Sometimes kmeans returns false [0 0 0] as center
      /// Count how many pixels belong to each cluster
      int trueCenters = 0;		
      for (int i=0; i<K; i++)
      {		
        if ( (float)counter[i] != 0 )
        {
          centers[i].x = (int) (centers[i].x / (float)counter[i]);
          centers[i].y = (int) (centers[i].y / (float)counter[i]);
          centers[i].z = (int) (centers[i].z / (float)counter[i]);	
          trueCenters++;
        }	
        else
        {
          centers[i].x = -1;
          centers[i].y = -1;
          centers[i].z = -1;
        }
      }	
    
      /// Now filter out the invalid points
      for (int i=0; i<K; i++)
      {
        if (centers[i].x != -1)
          colors.push_back(centers[i]);				
      }
      
      {/*
      cout << trueCenters << endl;
      cout << index << endl;
      cout << nPositiveClusters_ << endl;
      */}
    
      delete counter;
      delete centers;
      cvReleaseMat( &matPoints );
      cvReleaseMat( &matClusters );
    }
    
    //for (int i=0; i<colors.size(); i++)
    //	cout << colors[i].x << "||" << colors[i].y << "||" << colors[i].z << endl;
        
    int trueCenters = colors.size();
    CvPoint3D32f* newCenters;
    if (isPositive)
    {
      if ( this->positiveColors3d != NULL )
      {
        delete this->positiveColors3d;
        this->positiveColors3d = NULL;
      }
      
      if (trueCenters > 0)
      {
        this->positiveColors3d = new CvPoint3D32f[trueCenters];
        newCenters = this->positiveColors3d;
        this->nPositiveClusters_ = trueCenters;
      }
    }
    else
    {
      if ( this->negativeColors3d != NULL )
      {
        delete this->negativeColors3d;
        this->negativeColors3d = NULL;
      }
      
      if (trueCenters > 0)
      {
        this->negativeColors3d = new CvPoint3D32f[trueCenters];
        newCenters = this->negativeColors3d;
        this->nNegativeClusters_ = trueCenters;
      }
    }
    
    for (int i=0; i<trueCenters; i++)
      newCenters[i] = colors.at(i);
  }


  /**
   * ???
   * @param numberOfCenters
   * @param type
   */
  void TextureFilter::findPatternCenters3d_2(int numberOfCenters, int type)
  {
    bool isPositive = (type == POSITIVE);
    int K = numberOfCenters;
    
    vector<IplImage*>* imageVector;
    if (isPositive)
      imageVector = &this->positiveImages;
    else
      imageVector = &this->negativeImages;
    
    int totalImages = imageVector->size();
    int totalPoints = 0;
    
    if (totalImages == 0)
      return;
    
    /// find total number of points of either positive or negative pattern images
    for (int i=0; i<totalImages; i++)
      totalPoints += imageVector->at(i)->width * imageVector->at(i)->height;
    
    ///
    /// Create Mats
    ///
    CvMat* matPoints = cvCreateMat( totalPoints, 1, CV_32FC3 );
    CvMat* matClusters = cvCreateMat( totalPoints, 1, CV_32SC1 );
      
    /// fill Mat points with image data
    /// iterate through all images	
    for (int i=0; i<imageVector->size(); i++)
    { 
      IplImage* currentImage = imageVector->at(i);	
      int height     			= currentImage->height;
      int width     			= currentImage->width;
      int step      			= currentImage->widthStep/sizeof(uchar);
      int channels  			= currentImage->nChannels;
      uchar* data   			= (uchar*)currentImage->imageData;
            
      int counter=0;						
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          CvPoint3D32f pt;
          pt.x = (float)data[i*step+j*channels+0];
          pt.y = (float)data[i*step+j*channels+1];
          pt.z = (float)data[i*step+j*channels+2];
          
          matPoints->data.fl[counter*channels + 0] = pt.x;
          matPoints->data.fl[counter*channels + 1] = pt.y;
          matPoints->data.fl[counter*channels + 2] = pt.z;
          counter++;
        }
      }
      //cout << counter << endl;
      {/*
      IplImage* test = cvCreateImage( cvGetSize(currentImage) , 8 , 3);
      height     			= test->height;
      width     			= test->width;
      step      			= test->widthStep/sizeof(uchar);
      channels  			= test->nChannels;
      data   				= (uchar*)test->imageData;
      
      cvZero(test);
      counter = 0;
      for ( int i=0; i<height; i++ )
      {
        for ( int j=0; j<width; j++ )
        {
          data[i*step+j*channels+0] = matPoints->data.fl[counter*channels + 0];
          data[i*step+j*channels+1] = matPoints->data.fl[counter*channels + 1];
          data[i*step+j*channels+2] = matPoints->data.fl[counter*channels + 2];
          counter++;
        }
      }
      cvShowImage("test" , test);
      cvWaitKey(0);
      */} 
    }
    
    ///
    /// Execute Kmeans algorithm 
    ///
    cvKMeans2( matPoints, K, matClusters, 
          cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, MAX_ITERATIONS, ACCURACY ), 
          ATTEMPTS, 0, cv::KMEANS_PP_CENTERS);
    
    ///			
    /// Find centers of clusters
    /// Because opencv refuses to give me results
    ///
    CvPoint3D32f* centers = new CvPoint3D32f[K];	
    int* counter = new int[K];
    for (int i=0; i<K; i++)
    {
      centers[i].x = 0;
      centers[i].y = 0;
      centers[i].z = 0;
      counter[i]=0;
    }
    
    int channels = 3;
    for ( int i=0; i<totalPoints; i++ )
    {
      int cluster_idx = matClusters->data.i[i];
      centers[cluster_idx].x += matPoints->data.fl[i*channels+0];
      centers[cluster_idx].y += matPoints->data.fl[i*channels+1];
      centers[cluster_idx].z += matPoints->data.fl[i*channels+2];
      counter[cluster_idx]++;
    }
    
    int trueCenters = 0;		
    for (int i=0; i<K; i++)
    {		
      if ( (float)counter[i] != 0 )
      {
        centers[i].x = (int) (centers[i].x / (float)counter[i]);
        centers[i].y = (int) (centers[i].y / (float)counter[i]);
        centers[i].z = (int) (centers[i].z / (float)counter[i]);	
        trueCenters++;
      }	
      else
      {
        centers[i].x = -1;
        centers[i].y = -1;
        centers[i].z = -1;
      }
    }	

    /// Now filter out the invalid centers
    CvPoint3D32f* newCenters;
    if (isPositive)
    {
      if ( this->positiveColors3d != NULL )
      {
        delete this->positiveColors3d;
        this->positiveColors3d = NULL;
      }
      
      if (trueCenters > 0)
      {
        this->positiveColors3d = new CvPoint3D32f[trueCenters];
        newCenters = this->positiveColors3d;
        this->nPositiveClusters_ = trueCenters;
      }
    }
    else
    {
      if ( this->negativeColors3d != NULL )
      {
        delete this->negativeColors3d;
        this->negativeColors3d = NULL;
      }
      
      if (trueCenters > 0)
      {
        this->negativeColors3d = new CvPoint3D32f[trueCenters];
        newCenters = this->negativeColors3d;
        this->nNegativeClusters_ = trueCenters;
      }
    }	
    
    {/*
    for (int i=0; i<K; i++)
      cout << "All " << centers[i].x << " " << centers[i].y << " " << centers[i].z << endl;
    cout << "aaa" << endl;
    */}
    
    int index=0;
    for (int i=0; i<K; i++)
    {
      if (centers[i].x != -1)
      {
        newCenters[index].x = centers[i].x;
        newCenters[index].y = centers[i].y;
        newCenters[index].z = centers[i].z;
        index++;
      }	
    }
    {/*
    cout << trueCenters << endl;
    cout << index << endl;
    cout << nPositiveClusters_ << endl;
    */}
    
    delete counter;
    delete centers;
    cvReleaseMat( &matPoints );
    cvReleaseMat( &matClusters );
  }


  /**
   * Joins close histogram centers together.
   */
  void TextureFilter::mergeCenters()
  {
    bool haveAllChannels = (this->colorSpace_ == BGR) || (this->colorSpace_ == YCrCb);
    
    int totalInitialColors = 0;
    if (this->nPositiveClusters_ > 0)
      totalInitialColors = this->nPositiveClusters_;
    if (this->nNegativeClusters_ > 0)
      totalInitialColors += this->nNegativeClusters_;
          
    if (totalInitialColors == 0)
    {
      ROS_ERROR("[TextureFilter] : Something is wrong with the centers!"); 
      return;
    }
        
    if ( haveAllChannels ) 
    {
      vector<CvPoint3D32f> colorVec;
      for (int i=0; i<this->nPositiveClusters_; i++)
        colorVec.push_back(positiveColors3d[i]);
        
      for (int i=0; i<this->nNegativeClusters_; i++)
      {
        CvPoint3D32f pt1 = negativeColors3d[i];
        
        bool areSame = false;
        for (int j=0; j<colorVec.size(); j++)
        {
          CvPoint3D32f pt2 = colorVec.at(j);
          areSame = areSame || comparePoints(pt1,pt2);
          if (comparePoints(pt1,pt2))
            ROS_INFO("%f  %f|%f %f ",pt1.x,pt1.y, pt2.x, pt2.y);
        }
        if (areSame)
          continue;				
      
        colorVec.push_back(pt1);
      }
      
      if ( this->colors3d != NULL )
      {
        delete this->colors3d;
        this->colors3d = NULL;
      }
      
      int totalColors = colorVec.size();
      this->colors3d = new CvPoint3D32f[totalColors];
      for (int i=0; i<totalColors; i++)
        colors3d[i] = colorVec.at(i);
        
      this->nClusters_ = totalColors;
    }
    else  //colorSpace CrCb
    {
      vector<Vec2f> allCentroids;
      
      for (int i=0; i<nPositiveClusters_; i++)
      {
        bool areSame = false;
        Vec2f pt1 = (*centroidsPositive2d)(i,0);
        
        ///check if a point is in valid range
        if ( (pt1[0] < 15) || (pt1[0] > 235) || (pt1[1] < 15) || (pt1[1] > 235) )
        {		
          ROS_INFO("found that %f %f is not valid",pt1[0],pt1[1]); 
          continue;
        }
            
        ///check if a point already exists
        for (int j=0; j<allCentroids.size(); j++)
        {
          Vec2f pt2 = allCentroids.at(j);
          areSame = areSame || ( ( pow((pt1[0] - pt2[0]),2) +
                      pow((pt1[1] - pt2[1]),2) ) < 16 );
        }
        
        if (areSame)
        {
          ROS_INFO("found that  %f %f exists",pt1[0],pt1[1]);
          continue;
        }
          
        allCentroids.push_back(pt1);
      }
      
      for (int i=0; i<nNegativeClusters_; i++)
      {
        bool areSame = false;
        Vec2f pt1 = (*centroidsNegative2d)(i,0);
        
        ///check if a point is in valid range
        if ( (pt1[0] < 15) || (pt1[0] > 235) || (pt1[1] < 15) || (pt1[1] > 235) )
        {		
          ROS_INFO("found that %f %f is not valid",pt1[0],pt1[1]); 
          continue;
        }
            
        ///check if a point already exists
        for (int j=0; j<allCentroids.size(); j++)
        {
          Vec2f pt2 = allCentroids.at(j);
          areSame = areSame || ( ( pow((pt1[0] - pt2[0]),2) +
                      pow((pt1[1] - pt2[1]),2) ) < 32 );
        }
        
        if (areSame)
        {
          ROS_INFO("found that  %f %f exists",pt1[0],pt1[1]);
          continue;
        }
          
        allCentroids.push_back(pt1);
      }
          
      this->nClusters_ = allCentroids.size();
      ROS_INFO("%d ",nClusters_); 
      
      this->centroids2d = new Mat_<cv::Vec2f>(this->nClusters_, 1, CV_32FC2);
      
      for (int i=0; i<allCentroids.size(); i++)
      {
        (*centroids2d)(i,0) = allCentroids.at(i);
        //cout << "merged " << (*centroids2d)(i,0)[0] << " " << (*centroids2d)(i,0)[1] << endl; 
      }
      
    }
    
  }
   

  /**
   * Calculates texture given an input image.
   * @param imgInput
   * @return
   */
  IplImage* TextureFilter::calculateTexture(IplImage* imgInput)
  {	
    IplImage* imgResult = cvCreateImage( cvGetSize(imgInput) , 32 , 1);
    if (nClusters_ < 1)
    {
      cvZero(imgResult);
      return imgResult;
    }
    
    IplImage* img_ycrcb = NULL;
    IplImage* imgToUse = imgInput;
    if ( (this->colorSpace_ == YCrCb) || (this->colorSpace_ == CrCb) )
    {
      //cout << "Converting to YCrCb" << endl;
      img_ycrcb = cvCreateImage( cvGetSize(imgInput) , IPL_DEPTH_8U , 3);
      cvCvtColor(imgInput, img_ycrcb, CV_BGR2YCrCb);
      imgToUse = img_ycrcb;
    }
      
    ///scan every block area and calculate
    ///their kernel histograms
      
    ///compare with wall histograms and 
    ///keep the best scores
            
    cvZero(imgResult);
    int height     		= imgResult->height;
    int width     		= imgResult->width;
    int step      		= imgResult->widthStep/sizeof(float);
    float* data   		= (float*)imgResult->imageData;
                    
    /// for every block calculate histogram	
    int jStart = (int)((BLOCK_WIDTH-1)/2);
    int jEnd;
    if (BLOCK_WIDTH > STEP_X)
      jEnd = width - (int)((BLOCK_WIDTH-1)/2);
    else
      jEnd = width - (int)((STEP_X-1)/2);
      
    int iStart = (int)((BLOCK_HEIGHT-1)/2);
    int iEnd;
    if (BLOCK_HEIGHT > STEP_Y)
      iEnd = height - (int)((BLOCK_HEIGHT-1)/2);
    else
      iEnd = height - (int)((STEP_Y-1)/2);
      
    /// pass the coordinates of the center of 
    /// the area rectangle, to calculate texture
    for (int j=jStart; j<jEnd; j+=STEP_X )
    {
      for (int i=iStart; i<iEnd; i+=STEP_Y )
      {
        //double textureValue = calculateTextureArea(imgToUse, j, i, CHI_SQUARE);
        //double textureValue = calculateTextureArea(imgToUse, j, i, BHATTACHARYYA);
        //double textureValue = calculateTextureArea(imgToUse, j, i, COSINE_DISTANCE);
        double textureValue = calculateTextureArea(imgToUse, j, i, 3);

        //write score to result image
        data[i*step+j] = textureValue;
              
        int xUl = j - (int)(STEP_X-1)/2;
        int yUl = i - (int)(STEP_Y-1)/2;
        int xBr = j + (int)(STEP_X-1)/2;
        int yBr = i + (int)(STEP_Y-1)/2;
        cvRectangle(imgResult, cvPoint(xUl,yUl) , cvPoint(xBr,yBr), cvScalarAll(textureValue) ,-1,8,0);
        
      }
    } 
            
    if (img_ycrcb != NULL)
      cvReleaseImage(&img_ycrcb);
    
    return imgResult;
    
  }


  /**
   * Calculates the texture's area.
   * @param imgInput
   * @param pos_j
   * @param pos_i
   * @param method
   * @return
   */
  double TextureFilter::calculateTextureArea(IplImage* imgInput, int pos_j, int pos_i, int method)
  {	
    int totalCenters = nClusters_;
    
    int startX = pos_j - (int)((BLOCK_WIDTH-1)/2);
    int endX;
    if (BLOCK_WIDTH > STEP_X)
      endX = pos_j + (int)((BLOCK_WIDTH-1)/2);
    else
      endX = pos_j + (int)((STEP_X-1)/2);
      
    int startY = pos_i - (int)((BLOCK_HEIGHT-1)/2);
    int endY;
    if (BLOCK_HEIGHT > STEP_Y)
      endY = pos_i + (int)((BLOCK_HEIGHT-1)/2);
    else
      endY = pos_i + (int)((STEP_Y-1)/2);
                  
    /// create area histogram
    int numPatterns = patterns.size();
    double* histA = new double[totalCenters];
    memset(histA, 0, sizeof(double)*totalCenters);
    
    //for (int i=0; i<totalCenters; i++)
    //	cout << histA[i] << " ";
    //cout << endl;
      
    /// for a given list of centers
    /// find minimum distance and classify
    /// the current pixel. Vote for that center

    /// iterate through all block pixels
    /// and calculate area histogram
    int step      			= imgInput->widthStep/sizeof(uchar);
    int channels  			= imgInput->nChannels;
    uchar* data   			= (uchar*)imgInput->imageData;
      
    //cout << endY - startY << endl;
    //cout << endX - startX << endl;
    for (int i=startY; i<=endY; i++)
    {
      for (int j=startX; j<=endX; j++)
      {
        //Vec2f pt1;
        //pt1[0] = data[i*step+j*channels+1];
        //pt1[1] = data[i*step+j*channels+2];
            
        int cr = (int)data[i*step+j*channels+1];
        int cb = (int)data[i*step+j*channels+2];	
        
        for (int w=0; w<VOTE_NUM; w++)	
          histA[ colorLabels[cr][cb][w] ]++;
      }	
    }
    
    /// normalize area's histogram ---> seems OK!!!
    double sum = 0;
    for (int c=0; c<totalCenters; c++)
      sum += histA[c];
    
    for (int c=0; c<totalCenters; c++)
      histA[c] = (double)(histA[c] / sum);
      
        
    /// iterate through all patterns
    /// and find best similarity score		
    double score = 0;
    double scoreMin = -1;
    int index=0;
    for (int c=0; c<numPatterns; c++)
    {		
      /// current wall pattern
      Pattern* pattern = patterns.at(c);
      
      ///calculate similarity with current wall histogram
      double* histB = pattern->m_hist.valuesNorm;
            
      score = 0;		
      switch ( method ) 
      {
        case 0 :
          for (int k=0; k<totalCenters; k++)
          {
            if ( (histA[k] + histB[k]) != 0)
              score += pow( (histA[k] - histB[k]),2 ) / (histA[k] + histB[k]);
          }
          score = score/2;
          break;
          
        case 1 : 	
          for (int k=0; k<totalCenters; k++)
          {
            score += sqrt( abs(histA[k] * histB[k]) );
          }
          score = sqrt( abs(1 - score) );
          break;
          
        case 2 : 	
          for (int k=0; k<totalCenters; k++)	
            score += histA[k] * histB[k];
          score = 1 - score;
          break;
          
        case 3 : 
          for (int k=0; k<totalCenters; k++)	
          {
            score += pow( (histA[k] - histB[k]) , 2) / (histA[k] + 0.01);
            score += pow( (histA[k] - histB[k]) , 2) / (histB[k] + 0.01);
          }
          score = score/200;
          //score = 1 - score;
          break;
      
        default:
          for (int k=0; k<totalCenters; k++)
          {
            if ( (histA[k] + histB[k]) != 0)
              score += pow( (histA[k] - histB[k]),2 ) / (histA[k] + histB[k]);
          }
          score = score/2;
          break;
      }
      
      //double lambda = 5;
      //if (pattern->isPositive)
      //	scoreSum += exp(-lambda*score)*negativePatterns.size();
      //else
      //	scoreSum -= exp(-lambda*score)*positivePatterns.size();
            
      if ( (scoreMin == -1) || (scoreMin > score) )
      {
        index = c;
        scoreMin = score;
      }
              
    }
    score = scoreMin/2;
    
    if ( patterns.at(index)->isPositive == false )
      score = 1-score;	
            
    delete histA;
    histA = NULL;
    
    /// return score value
    return abs(score);		//sometimes returns -0
  }


  /**
   * Creates histogram regions.
   */
  void TextureFilter::createRegions()
  {
    /// Create Regions and save to regionContainer	
    int jStart = (int)((BLOCK_WIDTH-1)/2);
    int jEnd;
    if (BLOCK_WIDTH > STEP_X)
      jEnd = WIDTH - (int)((BLOCK_WIDTH-1)/2);
    else
      jEnd = WIDTH - (int)((STEP_X-1)/2);
      
    int iStart = (int)((BLOCK_HEIGHT-1)/2);
    int iEnd;
    if (BLOCK_HEIGHT > STEP_Y)
      iEnd = HEIGHT - (int)((BLOCK_HEIGHT-1)/2);
    else
      iEnd = HEIGHT - (int)((STEP_Y-1)/2);
      
    ///find number of regions
    nRegions_ = (int)((jEnd-jStart)/STEP_X) * (int)((iEnd-iStart)/STEP_Y);
    nRegionsRows_ = (int)((iEnd-iStart)/STEP_Y);
    nRegionsCols_ = (int)((jEnd-jStart)/STEP_X);
    
    ///initialize regions
    regionContainer = new region[nRegions_];
    for (int q=0; q<nRegions_; q++)
    {
      regionContainer[q].hist.values = new int[nClusters_];
      regionContainer[q].hist.valuesNorm = new double[nClusters_];
      regionContainer[q].hist.values = new int[nClusters_];
      regionContainer[q].hist.valuesNorm = new double[nClusters_];
      regionContainer[q].hist.values = new int[nClusters_];
      regionContainer[q].hist.valuesNorm = new double[nClusters_];
      
      regionContainer[q].id = (uintptr_t)&regionContainer[q];
    }
    
    /// fill regions with data 
    /// set id and neighbours
    int counter;
    for (int j=jStart; j<jEnd; j+=STEP_X )
    {
      for (int i=iStart; i<iEnd; i+=STEP_Y )
      {
        regionContainer[counter].j = j;
        regionContainer[counter].i = i;
        
        if ( ( i == iStart ) && ( j == jStart ) )	//first region of first row
        {
          regionContainer[counter].parent_id = 0;
          regionContainer[counter].child_id = (uintptr_t)(&regionContainer[counter+1]);
        }
        else if ( ( i == iStart ) && ( j != jStart ) ) //first region of a column. the first region of the first col excluded
        {
          regionContainer[counter].parent_id = (uintptr_t)(&regionContainer[counter-nRegionsRows_]);	
          regionContainer[counter].child_id = (uintptr_t)(&regionContainer[counter+1]);
        }
        else if ( ( i + STEP_Y > iEnd ) && ( j + STEP_X > jEnd ) )	//last element
        {
          regionContainer[counter].parent_id = (uintptr_t)(&regionContainer[counter-1]);
          regionContainer[counter].child_id = 0;
        }
        else // all other elements
        {
          regionContainer[counter].parent_id = (uintptr_t)&regionContainer[counter-1];
          regionContainer[counter].child_id = (uintptr_t)&regionContainer[counter+1];
        }
        
      }
    }
     
  }


  /**
   * Executes pixel clustering, to decide whether they belong to a wall or not.
   */
  void TextureFilter::clusterPixels()
  {
    int totalCenters = centroids2d->rows;
    memset(this->colorLabels, 0, sizeof(int)*256*256);
    
    if (totalCenters > 0)
    {
      for (int cr=0; cr<256; cr++)
      {
        for (int cb=0; cb<256; cb++)
        {
          Vec2f pt1(cr,cb);
          
          double distance = 0;
          int* indexArr = new int[totalCenters];
          double* distArr = new double[totalCenters];
          
          for (int k=0; k<totalCenters; k++)
          {	
            Vec2f pt2 = (*centroids2d)(k,0);
            distance = pow((pt1[0] - pt2[0]),2) + pow((pt1[1] - pt2[1]),2);
                                                              
            indexArr[k] = k;
            distArr[k] = distance;		
          }
          
          int i;
          int j;
          for (i=0; i<totalCenters; i++)
          {
            for (j=0; j<i; j++)
            {
              if (distArr[i] < distArr[j])
              {
                double temp = distArr[i];
                distArr[i] = distArr[j];
                distArr[j] = temp;
                
                temp = indexArr[i];
                indexArr[i] = indexArr[j];
                indexArr[j] = temp;
              }
            }
          }
          
          //for (int s=0; s<totalCenters; s++)
          //{
          //	cout << distArr[s] << "  " << indexArr[s] << endl;
          //}
                  
          for (int w=0; w<VOTE_NUM; w++)
            this->colorLabels[cr][cb][w] = indexArr[w];
          
          delete indexArr;
          delete distArr;
        }
        //cout << endl;
      }	
    }
    
    
  }

  void TextureFilter::drawLabelsImage( IplImage* dst )
  {
    int step = dst->widthStep/sizeof(uchar);
    uchar* data = (uchar*)dst->imageData;
    
    for (int i=0; i<256; i++)
    {
      for (int j=0; j<256; j++)
      {
        data[i*step+j] = colorLabels[i][j][0];
      }
    }
    
    cvEqualizeHist(dst,dst);
  }

  /**
   * Calculates patterns's histogram on every dimension, using calculateHistogram.
   */
  void TextureFilter::calculatePatternHistograms()
  {
    
    //for (int i=0; i<nClusters_; i++)
    //	cout << (*centroids2d)(i,0)[0] << " " << (*centroids2d)(i,0)[1];
    //cout << endl;
    
    for (int i=0; i<patterns.size(); i++)
    {
      Pattern* pattern = patterns.at(i);
    
      if ( this->dimension_ == 3)
      {
        pattern->calculateHistogram( colors3d, nClusters_, colorSpace_);
      }
      else if ( this->dimension_ == 2 )
      {
        
        pattern->calculateHistogram( centroids2d, nClusters_, colorSpace_);
        
        //cout << "oo ";
        //for (int i=0; i<nClusters_; i++)
        //	cout << pattern->m_hist.valuesNorm[i] << " ";
        //cout << endl;
      }
      else
        ROS_ERROR( "[TextureFilter] : Wrong dimension in calculatePatternHistograms");
    }
  }

  /**
   * Prints histogram centers to the console.
   */
  void TextureFilter::printCenters()
  {
    ROS_INFO("Printing centers");
    
    if (this->dimension_ == 3)
    {
      for (int i=0; i<this->nClusters_; i++)
          ROS_INFO("%f %f %f ",colors3d[i].x,colors3d[i].y,colors3d[i].z); 
    }
    else if (this->dimension_ == 2)
    {
      for (int i=0; i<this->nClusters_; i++)
          ROS_INFO("%f %f ",(*centroids2d)(i,0)[0],(*centroids2d)(i,0)[1]); 
    }
    else
      ROS_ERROR("Invalid dimension or NULL color-center array!");
  }

  /**
   * Prints histogram centers to the console.
   * @param type
   */
  void TextureFilter::printCenters(int type)
  {
    bool isPositive = (type == POSITIVE);
    
      if ( (dimension_ == 3) && (isPositive) && (this->nPositiveClusters_ != -1) && (positiveColors3d != NULL) )
      {
        for (int i=0; i<this->nPositiveClusters_; i++)
          ROS_INFO("%f %f %f",positiveColors3d[i].x,positiveColors3d[i].y,positiveColors3d[i].z);  
      }
      else if ( (dimension_ == 3) && (!isPositive) && (this->nNegativeClusters_ != -1) && (negativeColors3d != NULL) )
      {
        for (int i=0; i<this->nNegativeClusters_; i++)
          ROS_INFO("%f %f %f", negativeColors3d[i].x ,negativeColors3d[i].y, negativeColors3d[i].z);
      }
      else if ( (dimension_ == 2) && (isPositive) && (this->nPositiveClusters_ != -1) )
      {
        for (int i=0; i<this->nPositiveClusters_; i++)
          ROS_INFO("%f %f", (*centroidsPositive2d)(i,0)[0],(*centroidsPositive2d)(i,0)[1]);
      }
      else if ( (dimension_ == 2) && (!isPositive) && (this->nNegativeClusters_ != -1) )
      {
        for (int i=0; i<this->nNegativeClusters_; i++)
          ROS_INFO("%f %f",(*centroidsNegative2d)(i,0)[0],(*centroidsNegative2d)(i,0)[1]);
      }
      else
        ROS_ERROR("Invalid dimension or NULL color-center array!");
    
  }

  /**
   * Displays patterns for debugging purposes.
   * @param type
   */
  void TextureFilter::showPatterns(int type)
  {
    bool isPositive = (type == POSITIVE);
      
    if (isPositive)
    {
      for(int i=0; i<positivePatterns.size(); i++)
      {
        cvShowImage("Positive Patterns" , positivePatterns.at(i)->m_imgPatternBGR );
        cvWaitKey(0);
      }
    }
    else
    {
      for(int i=0; i<negativePatterns.size(); i++)
      {
        cvShowImage("Negative Patterns" , negativePatterns.at(i)->m_imgPatternBGR );
        cvWaitKey(0);
      }	
    }
  }

  /**
   * Displays images for debugging purposes.
   * @param type
   */
  void TextureFilter::showImages(int type)
  {
    bool isPositive = (type == POSITIVE);
      
    if (isPositive)
    {
      for(int i=0; i<positiveImages.size(); i++)
      {
        cvShowImage("Positive Images" , positiveImages.at(i) );
        cvWaitKey(0);
      }
    }
    else
    {
      for(int i=0; i<negativeImages.size(); i++)
      {
        cvShowImage("Negative Images" , negativeImages.at(i) );
        cvWaitKey(0);
      }	
    }
  }

  /**
   * Compares two 3D points.
   * @param pt1
   * @param pt2
   * @return
   */
  bool TextureFilter::comparePoints(CvPoint3D32f pt1, CvPoint3D32f pt2)
  {
    bool res = (pt1.x == pt2.x) && (pt1.y == pt2.y) && (pt1.z == pt2.z);
    
    return res;
  }

  /**
   * Compares two 2D points.
   * @param pt1
   * @param pt2
   * @return
   */
  bool TextureFilter::comparePoints(CvPoint2D32f pt1, CvPoint2D32f pt2)
  {
    bool res = (pt1.x == pt2.x) && (pt1.y == pt2.y);
    
    return res;
  }

  /**
   * Calculates the distance of two 3D points using various methods.
   * @param pt1
   * @param pt2
   * @param type
   * @return
   */
  double TextureFilter::calculatePointDistance(CvPoint3D32f pt1, CvPoint3D32f pt2, int type)
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
   * Calculates the distance of two 2D points using various methods.
   * @param pt1
   * @param pt2
   * @param type
   * @return
   */
  double TextureFilter::calculatePointDistance(CvPoint2D32f pt1, CvPoint2D32f pt2, int type)
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
   * Converts a scalar from BGR to YCrCb colorspace.
   * @param bgr
   * @return
   */
  CvScalar TextureFilter::BGR2YCrCb(CvScalar bgr)
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
   * Converts a scalar from YCrCb to BGR colorspace.
   * @param ycrcb
   * @return
   */
  CvScalar TextureFilter::YCrCb2BGR(CvScalar ycrcb)
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
}
