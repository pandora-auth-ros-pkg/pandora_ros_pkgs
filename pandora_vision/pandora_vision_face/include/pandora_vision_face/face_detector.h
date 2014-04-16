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
* Author: Aprilis George
* 		    Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_FACE_FACE_DETECTOR_H 
#define PANDORA_VISION_FACE_FACE_DETECTOR_H 

#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "pandora_vision_face/skin_detector.h"

namespace pandora_vision
{
class FaceDetector
{
private:
  SkinDetector* skinDetector;

  std::vector<std::vector<cv::Rect_<int> > > faces;

  std::vector<cv::Rect_<int> > faces_total;

  //!< If enabled SkinDetector output, is also considered in
  //!< probability value
  bool isSkinDetectorEnabled;

  int _bufferSize;

  //!< Number of rotation angles
  int angleNum;

  int now;

  cv::Mat skinImg;
  //!< Total probability of face found in a frame
  float probability;

  //!< Image buffer used to store frames
  std::vector<cv::Mat> frame_buffer;

  //!< Vector with partial probabilities that are used to
  //!< calculate total probability of face according to
  //!< consistency in last _bufferSize frames
  std::vector<float> probability_buffer;

  //!< Cascade classifier for face detection
  cv::CascadeClassifier cascade;

  //!<Trained model for face detection
  cv::Ptr<cv::FaceRecognizer> model;

  /**
    @brief Initializes frame and probability buffer
    @param image [cv::Mat] The current frame
    @return void
  */
  void initFrameProbBuffers(cv::Mat frame);

  /**
    @brief Rotates the given frame in 5 main angles and
      searches for faces in each rotated frame.
    @param frameIN [cv::Mat] The frame to be scanned for faces
    @return 	integer of the sum of faces found in all rotations
      of the frame.
  */
  int findFaces1Frame(cv::Mat frame);

  /**
    @brief Calls detectMultiscale to scan frame for faces and drawFace
      to create rectangles around the faces found in each frame
    @param frame [cv::Mat] the frame to be scaned.
    @param cascade [cv::CascadeClassifier] the classifier used for
      detection
    @param	angle [float] the rotation angle
    @return [int] the number of faces found in each frame
  */
  int detectFace(cv::Mat img);

  void createRectangles(cv::Mat *tmp);
  void compareWithSkinDetector(float *probability, cv::Mat tmp, int *totalArea);

public:


  //debug switch - webNode changes it externally:
  bool isDebugMode;

  //!< The Constructor
  FaceDetector(std::string cascade_path, std::string model_path,
               int bufferSize, bool skinEnabled, std::string skinHist, std::string wallHist, std::string wall2Hist);

  //!< The Destructor
  ~FaceDetector();

  /**
    @brief Searches for faces in current frame.
    @param image [cv::Mat] The  current frame
    @return number [int] of faces found in current frame
  **/
  int findFaces(cv::Mat img);

  /**
    @brief Creates the continuous table of faces found that contains
    information for each face in every set of 4 values:
    table[i*4]=face #i position x center
    table[i*4+1]=face #i position y center
    table[i*4+2]=face #i rectangle width
    table[i*4+3]=face #i rectangle height
    @return int[] table of face positions and sizes
  */
  int* getFacePositionTable();

  /**
    @brief Returns the size of the table with the positions of the
    faces found
    @return [int] size of table
  */
  int getFaceTableSize();

  /**
    @brief Returns the probability of the faces detected in the frame
    @return [float] probability value
  */
  float getProbability();

  /**
   @brief Rotates input frame according to the given angle
   @param frame [cv::Mat] the frame to be rotated.
   @param thAngle [int] angle in degrees (angle>=0)
     any angle more than 360 degrees is reduced to a primary circle
     angle.
   @param	rotMatData pointer to the data of the rotation
     matrix values produces for this rotation (this function feels the values)
   @return the frame rotated
  */
  cv::Mat frameRotate( cv::Mat frame, float angle);


};
}// namespace pandora_vision
#endif  // PANDORA_VISION_FACE_FACE_DETECTOR_H
