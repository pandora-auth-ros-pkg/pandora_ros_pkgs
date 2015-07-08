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
#ifndef PANDORA_VISION_LANDOLTC_LANDOLTC_3D_LANDOLTC3D_DETECTOR_H
#define PANDORA_VISION_LANDOLTC_LANDOLTC_3D_LANDOLTC3D_DETECTOR_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "pandora_vision_landoltc/landoltc_3d/landoltc3d_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_landoltc
{

struct LandoltC3D
{
  cv::Point center;
  int count;
  std::vector<float> angles;
  std::vector<cv::Scalar> color;
  std::vector<cv::Rect> bbox;
  float probability;
  LandoltC3D() {}
};

class LandoltC3dDetector
{
  private:
  /// Value for threshholding gradients
  int _minDiff;
  /// Value for thresholding values in voting array
  int _threshold;
  /// Vector containing centers of possible landolts
  std::vector <cv::Point> _centers;
  /// Vector containing colors of landolts,used for seperating them later
  std::vector<cv::Scalar> _fillColors;
  /// Vector containing bounding rectangles of each landolt
  std::vector<cv::Rect> _rectangles;
  /// Vector containing centers of verified landolts
  std::vector<cv::Point> _newCenters;
  /// Vector containing contour points of reference C
  std::vector<std::vector<cv::Point> > _refContours;
  /// 2D Matrix containing "votes" of each pixel, used for finding the centers
  cv::Mat _voting;
  /// 2D Matrix containing landoltsC's, each colored with a unique color
  cv::Mat _coloredContours;
  /// Pointer used for Contrast Limited Adaptive Histogram Equalization
  cv::Ptr<cv::CLAHE> clahe;
  /// 2D Matric used for separating each LandoltC3D to each parts
  cv::Mat _mask;
  /// Vector containing edge points of C, found using findRotationB function
  std::vector<cv::Point> _edgePoints;
  /// Value representing the number of edges found, again using finRotationB function
  int _edges;
  /// Vector containing LandoltC3D structs
  std::vector<LandoltC3D> _landoltc3d;
  /// Predator Bounding Box
  cv::Rect predator_bbox;
  /// Predator Probability
  float confidence;
  /// Boolean value used for fusion decision
  bool PredatorOn;

  public:
  /// Constructor
  LandoltC3dDetector();

  /// Destructor
  ~LandoltC3dDetector();

  /**
  @brief Function for the initialization of the reference image
  @param path [std::string] The path of the reference image
  @return void
  **/
  void initializeReferenceImage(std::string path);

  /**
  @brief Rasterize line between two points
  @param A [cv::Point] The start point of a line
  @param B [cv::Point] The end point of a line
  @return void
  **/
  void rasterizeLine(cv::Point A, cv::Point B);

  /**
  @brief Finds Centers based on gradient
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param grX [float*] X gradient component
  @param grY [float*] Y gradient component
  @return void
  **/
  void findCenters(int rows, int cols, float* grX, float* grY);

  /**
  @brief Finds LandoltC Contours on RGB Frames
  @param inImage [const cv::Mat&] Input Image
  @param rows [int] Number of rows of matrix
  @param cols [int] Number of columns of matrix
  @param ref [std::vector<cv::Point>] Vector containing contour points of reference image
  @return void
  **/
  void findLandoltContours(const cv::Mat& inImage, int rows, int cols, std::vector<cv::Point> ref);

  /**
  @brief Mask for separating a LandoltC3D Contour to its components
  @return void
  **/
  void applyMask();

  /**
  @brief Thinning algorith using the Zhang-Suen method
  @param in [cv::Mat*] Matrix containing the frame to thin
  @return void
  **/
  void thinning(cv::Mat* in);

  /**
  @brief Thinning iteration call from the thinning function
  @param in [cv::Mat*] Matrix containing the frame to thin
  @param iter [int] Number of iteration with values 1-2
  @return void
  **/
  void thinningIter(cv::Mat* in, int iter);

  /**
  @brief Function called for the initiation of LandoltC search in the frame
  @param input [cv::Mat*] Matrix containing the frame received from the camera
  @return void
  **/
  void begin(cv::Mat* input);

  /**
  @brief Function for applying BradleyThresholding on Image
  @param in [cv::Mat&] Input Image to be thresholded
  @param out [cv::Mat*] Output, thresholded image
  @return void
  **/

  void applyBradleyThresholding(const cv::Mat& in, cv::Mat* out);

  /**
  @brief Calculation of rotation based on thinning.Precision is good for a
  distance up to 50cm from the camera, gives more accurate results than the first
  method but it's slower.
  @param in [const cv::Mat&] Matrix containing the padded frame
  @param temp [LandoltC3D*] Struct of LandoltC3D
  @return void
  **/
  void findRotation(const cv::Mat&in, LandoltC3D* temp);

  /**
  @brief Function for calculating the neighbours of pixels considering
  8-connectivity
  @param index [unsigned int] Index of pixel in matrix
  @param in [cv::Mat&] Input Image
  @return void
  **/
  void find8Neights(unsigned int index, const cv::Mat& in);

  /**
  @brief Function for calculating perspective transform, in
  order to get better angle calculation precision
  @param rec [cv::Rect] Rectangle enclosing a 'C'
  @param in [cv::Mat&] Input Image
  @return [cv::Mat] Output Image
  **/
  cv::Mat getWarpPerspectiveTransform(const cv::Mat& in, cv::Rect rec);

  /**
  @brief Clearing vector values
  @param void
  @return void
  **/
  void clear();

  /**
  @brief Function for fusing results from both LandoltC3D and
  Predator
  @return void
  **/

  void fusion();

  /**
  @brief Function for storing bounding box and probability sent
  from Predator
  @param bbox [cv::Rect] Predator bounding box
  @param posterior [float] Predator Probability
  @return void
  **/
  void setPredatorValues(cv::Rect bbox, float posterior);

  /**
  @brief Function used for fusion, in order to decide whether Predator
  is ON or OFF
  @return void
  **/
  void setPredatorOn(bool flag);

  /**
  @brief Returns detected landoltc3d, for publishing them later
  @param void
  @return [std::vector<LandoltC3D>] Vector of detected Landolts
  **/
  std::vector<LandoltC3D> getDetectedLandolt();
};
}  // namespace pandora_vision_landoltc
}  // namespace pandora_vision
#endif  // PANDORA_VISION_LANDOLTC_LANDOLTC_3D_LANDOLTC3D_DETECTOR_H
