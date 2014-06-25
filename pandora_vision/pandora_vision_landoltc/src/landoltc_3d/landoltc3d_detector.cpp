/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Victor Daropoulos
*********************************************************************/

#include "pandora_vision_landoltc/landoltc_3d/landoltc3d_detector.h"


namespace pandora_vision
{
//!< Constructor
LandoltC3dDetector::LandoltC3dDetector()
{
  _minDiff = 60;

  _threshold = 90;
  
  _edges = 0;
  
}

//!< Destructor
LandoltC3dDetector::~LandoltC3dDetector()
{
  ROS_INFO("[landoltc3d_node]: landoltc3d_detector instance destroyed");
}

/**
  @brief Function for the initialization of the reference image
  @param void
  @return void
**/
void LandoltC3dDetector::initializeReferenceImage(std::string path)
{
  //!<Loading reference image passed as argument to main
  cv::Mat ref;
  //std::cout << path << std::endl;
  ROS_DEBUG_STREAM("path: " << path);
  ref = cv::imread(path);
  if (!ref.data)
    //std::cout << "Pattern image not loaded" << std::endl;
    ROS_DEBUG("Pattern image not loaded");

  //!< Turning to gray and binarizing ref image

  cv::cvtColor(ref, ref, CV_BGR2GRAY);
  cv::threshold(ref, ref, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);

  cv::findContours(ref, _refContours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  
  clahe = cv::createCLAHE(4, cv::Size(8, 8));
}

/**
  @brief Mask for separating a LandoltC3D Contour to its components
  @return void
**/
void LandoltC3dDetector::applyMask()
{
  for(int i = 0; i < _landoltc3d.size(); i++)
  {    
    for (int j = 0; j < _landoltc3d.at(i).color.size(); j++)
    {
      _mask = cv::Mat(_coloredContours.rows, _coloredContours.cols, CV_8UC1);
    
      cv::inRange(_coloredContours, _landoltc3d.at(i).color[j], _landoltc3d.at(i).color[j], _mask);
    
      cv::Mat out = getWarpPerspectiveTransform(_mask, _landoltc3d.at(i).bbox[j]);
    
      cv::Mat cropped = _mask(_landoltc3d.at(i).bbox[j]).clone(); 
    
      cv::Mat padded;
    
      cv::copyMakeBorder(out, padded, 8, 8, 8, 8, cv::BORDER_CONSTANT, cv::Scalar(0));
    
      findRotation(padded, &(_landoltc3d.at(i)));
    
      if(Landoltc3DParameters::visualization)
      {
        cv::imshow("padded", padded);
        cv::waitKey(20);
      }
    }
  }
}

/**
  @brief Thinning algorith using the Zhang-Suen method
  @param in [cv::Mat*] Matrix containing the frame to thin
  @return void
**/
void LandoltC3dDetector::thinning(cv::Mat* in)
{
  *in = *in / 255;
  static cv::Mat thinned;
  static cv::Mat dif;
  in->copyTo(thinned);

  do {
    thinningIter(in, 1);
    thinningIter(in, 2);
    cv::absdiff(*in, thinned, dif);
    in->copyTo(thinned);;
  } while(cv::countNonZero(dif) > 0);

  *in = (*in) * 255;
}

/**
  @brief Thinning iteration call from the thinning function
  @param in [cv::Mat*] Matrix containing the frame to thin
  @param iter [int] Number of iteration with values 1-2
  @return void
  **/
void LandoltC3dDetector::thinningIter(cv::Mat* in, int iter)
{
  static cv::Mat temp = cv::Mat(in->size(), CV_8UC1);

  for(int y = 0; y < (in->rows - 1); y++)
  {
    for(int x = 0; x < (in->cols - 1); x++)
    {
      int ind = in->cols * y + x;
      temp.data[ind] = 1;
      
      unsigned char p2 = in->data[ind - in->cols];   
      unsigned char p4 = in->data[ind + 1];
      unsigned char p6 = in->data[ind + in->cols];
      unsigned char p8 = in->data[ind - 1];
      int c1 = p2 * p4 * (iter == 1 ? p6 : p8);
      int c2 = (iter == 1 ? p4 : p2) * p6 * p8;
      if(!(c1 == 0 && c2 == 0))
        continue;
      
      unsigned char p3 = in->data[ind - in->cols + 1];
      unsigned char p5 = in->data[ind + in->cols + 1];
      unsigned char p7 = in->data[ind + in->cols - 1];
      unsigned char p9 = in->data[ind - in->cols - 1];

      int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
      if( B < 2 || B > 6)
      {
        continue;
      }

      int A = 
        (p2 == 0 && p3 == 1) + 
        (p3 == 0 && p4 == 1) + 
        (p4 == 0 && p5 == 1) + 
        (p5 == 0 && p6 == 1) + 
        (p6 == 0 && p7 == 1) + 
        (p7 == 0 && p8 == 1) + 
        (p8 == 0 && p9 == 1) + 
        (p9 == 0 && p2 == 1);
      
      if(A != 1)
        continue;

      temp.data[ind] = 0;

    }
  }

  cv::bitwise_and(*in, temp, *in);
}

/**
  @brief Function for calculating the neighbours of pixels considering
  8-connectivity
  @param index [unsigned int] Index of pixel in matrix
  @param in [cv::Mat&] Input Image
  @return void
**/    
void LandoltC3dDetector::find8Neights(unsigned int index, const cv::Mat& in)
{
  unsigned int y = index / in.cols;
  unsigned int x = index % in.cols;

  unsigned char p1 = in.at<unsigned char>(y-1, x);
  unsigned char p2 = in.at<unsigned char>(y-1, x+1);
  unsigned char p3 = in.at<unsigned char>(y, x+1);
  unsigned char p4 = in.at<unsigned char>(y+1, x+1);
  unsigned char p5 = in.at<unsigned char>(y+1, x);
  unsigned char p6 = in.at<unsigned char>(y+1, x-1);
  unsigned char p7 = in.at<unsigned char>(y, x-1);
  unsigned char p8 = in.at<unsigned char>(y-1, x-1);
     
  if(p1+p2+p3+p4+p5+p6+p7+p8 == 255) 
  {
    _edges++;
    _edgePoints.push_back(cv::Point(x, y));    
  }
}

/**
  @brief Function for calculating perspective transform, in
  order to get better angle calculation precision
  @param rec [cv::rec] Rectangle enclosing a 'C'
  @param in [cv::Mat&] Input Image
  @return [cv::Mat] Output Image 
**/    
cv::Mat LandoltC3dDetector::getWarpPerspectiveTransform(const cv::Mat& in, cv::Rect rec)
{
  std::vector<cv::Point2f> corners;
  static cv::Mat quad = cv::Mat::zeros(100, 100, CV_8UC1);
  
  corners.clear();
  corners.push_back(rec.tl());
  
  cv::Point2f tr = cv::Point2f(rec.tl().x+rec.width, rec.tl().y);
  corners.push_back(tr);
  
  corners.push_back(rec.br());
  cv::Point2f bl = cv::Point2f(rec.br().x-rec.width, rec.br().y);
  corners.push_back(bl);
  
  
  // Corners of the destination image
  
  static cv::Point2f q_p[] = {
    cv::Point2f(0, 0),
    cv::Point2f(quad.cols, 0),
    cv::Point2f(quad.cols, quad.rows),
    cv::Point2f(0, quad.rows)
  };
  static std::vector<cv::Point2f> 
    quad_pts ( q_p, q_p + sizeof(q_p) / sizeof(cv::Point2f) );
  
  // Get transformation matrix
  cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);
  
  // Apply perspective transformation
  cv::warpPerspective(in, quad, transmtx, quad.size());
  
  return quad;
}

/**
  @brief Calculation of rotation based on thinning.Precision is good for a
  distance up to 50cm from the camera, gives more accurate results than the first
  method but it's slower.
  @param in [const cv::Mat&] Matrix containing the padded frame
  @param temp [LandoltC3D*] Struct of LandoltC3D
  @return void
**/  
void LandoltC3dDetector::findRotation(const cv::Mat&in, LandoltC3D* temp)
{
  static cv::Mat paddedptr;
        
  paddedptr = in.clone();
        
  thinning(&paddedptr);
  
  static unsigned int *pts = new unsigned int[paddedptr.cols * paddedptr.rows];
        
  int limit = 0;

  for (unsigned int rows = 1; rows < paddedptr.rows - 1; rows++)
  {
    for (unsigned int cols = 1; cols < paddedptr.cols - 1; cols++)
    {
      if(paddedptr.data[rows * paddedptr.cols + cols] !=0)
      {
          pts[limit]= rows * paddedptr.cols + cols;
          limit++;          
      }
    }
  }
  
  for(int j = 0; j < limit; j++) 
  {
    find8Neights(pts[j], paddedptr);    
  }
  
  for(int k = 0; k < _edgePoints.size(); k++)  
  {
    cv::circle(paddedptr, _edgePoints.at(k), 2, 255, -1, 8, 0);
  }
  
  if(_edgePoints.size() == 2)
  {
    int yc=(_edgePoints.at(0).y+_edgePoints.at(1).y)/2;
    int xc=(_edgePoints.at(0).x+_edgePoints.at(1).x)/2;
    
    cv::Point gapCenter(xc, yc);
    
    cv::circle(paddedptr, gapCenter, 1, 255, -1, 8, 0);
        
    cv::Point center(paddedptr.cols/2, paddedptr.rows/2);
    
    cv::circle(paddedptr, center, 1, 255, -1, 8, 0);
  
    double angle = atan2(gapCenter.y-center.y, gapCenter.x-center.x);
    
    if(angle < 0) angle+=2*3.14159265359;
    
    //std::cout << "Angle of " << i <<" is : " << angle*(180/3.14159265359) << std::endl;
    //ROS_INFO("Angle of %d is %lf \n", i, angle*(180/3.14159265359));
    (*temp).angles.push_back(angle);

    
  }
  
  if(Landoltc3DParameters::visualization)
  {
    cv::imshow("paddedptr", paddedptr); 
    cv::waitKey(5); 
  }
    
  _edges = 0;
  
  _edgePoints.clear();
  
  //~ delete[] pts;
  
}

/**
  @brief Function for applying BradleyThresholding on Image
  @param in [cv::Mat&] Input Image to be thresholded
  @param out [cv::Mat*] Output, thresholded image
  @return void
**/

void LandoltC3dDetector::applyBradleyThresholding(const cv::Mat& input, cv::Mat* output)
{
  int64_t sum = 0;
  
  int count = 0;
      
  int index;
      
  static uint64_t* integralImg =new uint64_t[input.cols*input.rows];
          
  unsigned char* in=(unsigned char*)input.data;
  unsigned char* out=(unsigned char*)output->data;
  
  int x1, y1, x2, y2;
      
  int s2 = input.cols/16;
      
        
  for(int i = 0; i < input.cols; i++)
  {
    sum = 0;
        
    for(int j = 0; j < input.rows; j++)
    {
      index = j*input.cols+i;
      sum+=in[index];
      if(i == 0) 
        integralImg[index]=sum;
      else
        integralImg[index]=integralImg[index-1]+sum;
    }
  }

  for(int i = 0; i < input.cols; i++)
  {
    for(int j = 0; j < input.rows; j++)
    {
      index = j*input.cols+i;
      
      x1 = i-s2;
      x2 = i+s2;
      y1 = j-s2;
      y2 = j+s2;
          
      if (x1 < 0) x1 = 0;
      if (x2 >= input.cols) x2 = input.cols-1;
      if (y1 < 0) y1 = 0;
      if (y2 >= input.rows) y2 = input.rows-1;
          
      count=(x2-x1)*(y2-y1);
          
      sum = integralImg[y2*input.cols+x2]-integralImg[y1*input.cols+x2]-
      integralImg[y2*input.cols+x1]+integralImg[y1*input.cols+x1];
          
      if((int64_t)(in[index]*count) < (int64_t)(sum*(1.0-Landoltc3DParameters::bradleyPerc)))
      {
        out[index]=0;
      }
      else
      {
        out[index]=255;
      }
    }
  }
  
  //~ delete[] integralImg;
}
      

/**
  @brief Rasterize line between two points
  @param A [cv::Point] The start point of a line
  @param B [cv::Point] The end point of a line
  @return void
**/

void LandoltC3dDetector::rasterizeLine(cv::Point A, cv::Point B)
{
  uint16_t* votingData = reinterpret_cast<uint16_t*>(_voting.data);
  cv::Rect r(0, 0, _voting.cols, _voting.rows);

  //!< if line is out of frame return
  if(!cv::clipLine(r, A, B)) return;

  float rounding = 0.49999;

  int dx = B.x - A.x;
  int dy = B.y - A.y;

  //!<X major line

  if (abs(dx) >= abs(dy))
  {
    if (B.x < A.x) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.y - A.y) / static_cast<float>(B.x - A.x);
    float yIntercept = A.y - gradient * A.x + rounding;
    for (int i = A.x; i < B.x; i++)
    {
      float y = gradient * i + yIntercept;
      votingData[(static_cast<int>(y) * _voting.cols) + i] += 1.0;
    }
  }

  //!<Y major line

  if (abs(dx) < abs(dy))
  {
    if (B.y < A.y) std::swap(A, B);

    //!<calculation of values of line, gradient and intercept
    float gradient = (B.x - A.x) / static_cast<float>(B.y - A.y);
    float xIntercept = A.x - gradient * A.y + rounding;
    for (int i = A.y; i < B.y; i++)
    {
      float x = gradient * i + xIntercept;
      votingData[i * _voting.cols + static_cast<int>(x)] += 1.0;
    }
  }
}

/**
  @brief Finds Centers based on gradient
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param grX [float*] X gradient component
  @param grY [float*] Y gradient component
  @return void
**/

void LandoltC3dDetector::findCenters(int rows, int cols, float* grX, float* grY)
{
  cv::Point center;

  //!< Rasterization of lines between thresholded points

  for (int x = 0; x < cols; x++)
  {
    for (int y = 0; y < rows; y++)
    {
      int i = y * cols + x;
      float dx = grX[i];
      float dy = grY[i];
      float mag = dx * dx + dy * dy;
      if (mag > (Landoltc3DParameters::gradientThreshold 
      * Landoltc3DParameters::gradientThreshold))
      {
        mag = sqrt(mag);
        float s = 20 / mag;
        rasterizeLine(cv::Point(x + dx * s, y + dy * s), cv::Point(x - dx * s, y - dy * s));
      }
    }
  }

  const uint16_t* readvoting = (const uint16_t*)_voting.data;
  int de = 2;
  int bullcount = 0;

  //!<Searching for landoltC centers

  for (int y = de; y < rows - de ; y++)
  {
    for (int x = de; x < cols - de; x++)
    {
      int i = y * cols + x;
      int cur = readvoting[i];
      if (cur >= Landoltc3DParameters::centerThreshold)
      {
        bool biggest = true;

        //!<Search if there's a bigger center in a smaller area
        for (int dy = -de; dy < de && biggest; dy++)
        {
          for(int dx = -de; dx < de; dx++)
          {
            float her = readvoting[(y + dy) * cols + (x + dx)];
            if (cur < her)
            {
              biggest = false;
              break;
            }
          }
        }

        if (biggest)
        {
          center.x = x;
          center.y = y;
          _centers.push_back(center);
         // std::cout << "Bullseye " << bullcount++ << " xy " << center.x << "," << center.y << std::endl;
        }
      }
    }
  }

  return;
}

/**
  @brief Finds LandoltC Contours on RGB Frames
  @param inImage [const cv::Mat&] Input Image
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param ref [std::vector<cv::Point>] Vector containing contour points of reference image
  @return void
**/

void LandoltC3dDetector::findLandoltContours(const cv::Mat& inImage, int rows, int cols, std::vector<cv::Point> ref)
{
  cv::RNG rng(12345);

  //!<find contours and moments in frame used for shape matching later

  static std::vector<std::vector<cv::Point> > contours;
  cv::findContours(inImage, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
  std::vector<cv::Moments> mu(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mu[i] = moments(contours[i], false);
  }

  std::vector<cv::Point2f> mc(contours.size());
  for(int i = 0; i < contours.size(); i++)
  {
    mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
  }

  std::vector<cv::Point> approx;

  //!<Shape matching using Hu Moments, and contour center proximity

  LandoltC3D temp;
  
  for(std::vector<cv::Point>::iterator it = _centers.begin(); it != _centers.end(); ++it)
  {
    int counter = 0;
    bool flag = false;
    _fillColors.clear();
    _rectangles.clear();
    
    for(int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true) * 0.02, true);
      std::vector<cv::Point> cnt = contours[i];
      double prec = cv::matchShapes(cv::Mat(ref), cv::Mat(cnt), CV_CONTOURS_MATCH_I3, 0);
      if (!isContourConvex(cv::Mat(cnt)) && fabs(mc[i].x - (*it).x) < 7 && fabs(mc[i].y - (*it).y) < 7 
      && prec < Landoltc3DParameters::huMomentsPrec)
      {
        flag = true;
        cv::Rect bounding_rect = boundingRect((contours[i]));
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(_coloredContours, contours, i, color, CV_FILLED, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        _fillColors.push_back(color);
        _newCenters.push_back(mc[i]);
        _rectangles.push_back(bounding_rect);
        counter++;

      }
    }
    
    if(flag)
    {
      temp.center = *it;
      temp.count = counter;
      temp.color = _fillColors;
      temp.bbox = _rectangles;
      _landoltc3d.push_back(temp);      
      flag = false;
    } 
    
  }
}


/**
  @brief Function called from ImageCallBack that Initiates LandoltC search in the frame
  @param input [cv::Mat*] Matrix containing the frame received from the camera
  @return void
**/

void LandoltC3dDetector::begin(cv::Mat* input)
{
  static cv::Mat gray, gradX, gradY, dst, abs_grad_x, abs_grad_y, thresholded, grad_x, grad_y;
  static  cv::Mat erodeKernel(cv::Size(1, 1), CV_8UC1, cv::Scalar(1));
  
  cv::cvtColor(*input, gray, CV_BGR2GRAY);

  _voting = cv::Mat::zeros(input->rows, input->cols, CV_16U);
  _coloredContours = cv::Mat::zeros(input->rows, input->cols, input->type());
  thresholded = cv::Mat::zeros(input->rows, input->cols, CV_8UC1);
  
  bilateralFilter(gray, dst, 2, 4, 1);
    
  gray = dst.clone();
  
  clahe->apply(gray, dst);
  
  cv::Sobel(dst, gradX, CV_32F, 1, 0, 3);
  cv::Sobel(dst, gradY, CV_32F, 0, 1, 3);

  float* gradXF = reinterpret_cast<float*>(gradX.data);
  float* gradYF = reinterpret_cast<float*>(gradY.data);

  findCenters(dst.rows, dst.cols, gradXF, gradYF);
  
  convertScaleAbs( gradX, abs_grad_x );
  
  convertScaleAbs( gradY, abs_grad_y );
      
  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst ); 
  
  //~ cv::adaptiveThreshold(dst, thresholded, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
  //~ cv::THRESH_BINARY_INV, 7, Landoltc3DParameters::adaptiveThresholdSubtractSize);
  
  applyBradleyThresholding(dst, &thresholded);

  cv::erode(thresholded, thresholded, erodeKernel);
  
  if(Landoltc3DParameters::visualization)
  {
    cv::imshow("thresholded", thresholded);
    cv::waitKey(10);
  }
  
  findLandoltContours(thresholded, thresholded.rows, thresholded.cols, _refContours[0]);

  for(std::size_t i = 0; i < _rectangles.size(); i++)
  {
    cv::rectangle(*input, _rectangles.at(i), cv::Scalar(0, 0, 255), 1, 8, 0);
  }
  
  applyMask();
  
  if(Landoltc3DParameters::visualization)
  {
    cv::imshow("Raw", *input);
    cv::waitKey(10);
  }
  
  fusion();

}

/**
  @brief Function for fusing results from both LandoltC3D and
  Predator
  @return void
**/
void LandoltC3dDetector::fusion()
{
  if(!PredatorOn)
  {
    
    for(int i = 0; i < _landoltc3d.size(); i++)
    {
      if(_landoltc3d.at(i).angles.size() == 1)
      {
        _landoltc3d.at(i).probability = 0.2;
      }
      else if(_landoltc3d.at(i).angles.size() == 2)
      {
        _landoltc3d.at(i).probability = 0.4;
      }
      else if(_landoltc3d.at(i).angles.size() == 3)
      {
        _landoltc3d.at(i).probability = 0.6;
      }
      else if(_landoltc3d.at(i).angles.size() == 4)
      {
        _landoltc3d.at(i).probability = 0.8;
      }
      else if(_landoltc3d.at(i).angles.size() == 5)
      {
        _landoltc3d.at(i).probability = 1;
      }
      ROS_INFO("Probability is %f", _landoltc3d.at(i).probability);    
   
    }
    
  }
  else
  {
    for(int i = 0; i< _landoltc3d.size(); i++)
    {
      if(predator_bbox.contains(_landoltc3d.at(i).center))
      {
        
        _landoltc3d.at(i).probability = confidence;
        //ROS_INFO("Probability is %f", confidence);
      }
    }
  
  }
}

/**
  @brief Function for storing bounding box and probability sent
  from Predator
  @param bbox [cv::Rect] Predator bounding box
  @param posterior [float] Predator Probability
  @return void
**/
void LandoltC3dDetector::setPredatorValues(cv::Rect bbox, float posterior)
{
  predator_bbox = bbox;
  confidence = posterior;
}

/**
  @brief Function used for fusion, in order to decide whether Predator
  is ON or OFF
  @return void
**/
void LandoltC3dDetector::setPredatorOn(bool flag)
{
  PredatorOn = flag;
}
  

/**
  @brief Clearing vector values
  @param void
  @return void
**/
void LandoltC3dDetector::clear()
{
  _centers.clear();
  _newCenters.clear();
  _rectangles.clear();
  _fillColors.clear();
  _landoltc3d.clear();
}

/**
  @brief Returns detected landoltc3d, for publishing them later
  @param void
  @return [std::vector<LandoltC3D>] Vector of detected Landolts
**/

std::vector<LandoltC3D> LandoltC3dDetector::getDetectedLandolt()
{
  return _landoltc3d;
}


} // namespace pandora_vision
