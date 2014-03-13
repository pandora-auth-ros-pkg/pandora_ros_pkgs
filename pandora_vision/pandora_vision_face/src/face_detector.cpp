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
* Author: George Aprilis
* 		    Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_face/face_detector.h"

namespace pandora_vision
{
/**
 @brief Class Constructor
 Initializes cascade_name and constants
 and allocates memory for sequence of elements
 @param cascade_path [std::string] the name of
         the cascade to be loaded
 @param model_path [std::string] the path to the model
         to be loaded
 @param bufferSize [int] number of frames in the frame buffer
 @param skinEnabled [bool] enables the verification
         of face algorithm using the Skin detector
 @param mn parameter used in cvHaarDetect function
 @param minFaceDim parameter used in cvHaarDetect function
 @param skinHist Histogram - parameter to be passed in the Skin Detector
 @param wallHist Histogram - parameter to be passed in the Skin Detector
 @param wall2Hist Histogram - parameter to be passed in the Skin Detector
 @return void
*/
FaceDetector::FaceDetector(std::string cascade_path, std::string model_path,
                           int bufferSize, bool skinEnabled, std::string skinHist,
                           std::string wallHist, std::string wall2Hist )
{
  cascade.load(cascade_path);
  if(cascade.empty())
  {
    ROS_ERROR("[Face Detector]: Cannot load cascade classifier");
    exit(0);
  }

  model = cv::createFisherFaceRecognizer();
  model->load(model_path);

  _bufferSize = bufferSize;
  angleNum = 5;
  isSkinDetectorEnabled = skinEnabled;
  isDebugMode = false;

  now = 0;

  probability = 1.;

  if (isSkinDetectorEnabled == true)
  {
    skinDetector = new SkinDetector( skinHist, wallHist, wall2Hist );
  }

}

/**
 @brief Class Destructor
 Deallocates all memory used for storing sequences of faces,
 matrices and images
*/
FaceDetector::~FaceDetector()
{
  //!< Erase frame and probability buffers
  if (!frame_buffer.empty())
  {
    frame_buffer.erase (frame_buffer.begin(), frame_buffer.begin() +
                        frame_buffer.size());
  }

  if (!probability_buffer.empty())
  {
    probability_buffer.erase (probability_buffer.begin(),
                              probability_buffer.begin() + probability_buffer.size());
  }

  delete skinDetector;
}


/**
  @brief Detects number of faces found in current frame.
  The image buffer contributs to probability.
  @param frameIN [cv::Mat] The frame to be scanned for faces
  @return Integer of the sum of faces found in all
  rotations of the frame
*/
int FaceDetector::findFaces(cv::Mat frame)
{

  cv::Mat tmp;
  tmp = cv::Mat::zeros(frame.size().width, frame.size().height , CV_8UC1);

  initFrameProbBuffers(frame);
  //~ createRectangles(tmp);

  //!< Clear vector of faces before using it for the current frame
  faces_total.erase (faces_total.begin(), faces_total.begin() +
                     faces_total.size());

  int facesNum = detectFace(frame);
  ROS_INFO_STREAM("Number of faces found " << facesNum);

  int totalArea = 0;

  if(facesNum)
  {
    totalArea = round( cv::norm(tmp, cv::NORM_L1, cv::noArray()) / 255.);
  }

  if(totalArea == 0)
  {
    //!< if no face was found, probability for this frame is 0
    probability_buffer[now] = 0.;
  }
  else
  {
    probability_buffer[now] = round( cv::norm(tmp, cv::NORM_L1,
                                     cv::noArray()) / 255.) / (float)totalArea;
  }
  //!< clear value from last scan
  probability = 0.;

  //!< calculate probability
  for(int i = 0 ; i < _bufferSize ; i++)
  {
    probability += (probability_buffer[i]);
  }
  probability = probability / _bufferSize;

  //!< Compare Probability with Skin Output
  if(isSkinDetectorEnabled )
  {
    std::cout << "Skin detector enabled" << std::endl;
    skinDetector->init();

    //!< When there is a problem with detectSkin() it returns 1
    //!< in this case findFaces() returns -2
    if ( skinDetector->detectSkin( frame ) )
    {
      return -2;
    }
    //compareWithSkinDetector(probability, tmp, totalArea);
  }
  now = (now + 1) % _bufferSize; //prepare index for next frame

  return facesNum;
}


/**
  @brief Set probability accordint to skinDetector instance
  @param frameIN [cv::Mat] The frame to be scanned for faces
  @return Integer of the sum of faces found in all
  rotations of the frame
*/
void FaceDetector::compareWithSkinDetector(float &probability, cv::Mat tmp, int &totalArea)
{
  float skinFactor = 0.;
  float skinFaceRatio = 0.;
  int skinPixelNum = 0;

  //skinImg = skinDetector->getImgContoursForFace();
  skinImg = skinDetector->imgThresholdFiltered;
  skinPixelNum = round( cv::norm(skinImg, cv::NORM_L1, cv::noArray()) / 255.);
  bitwise_and( frame_buffer[now] , skinImg , tmp); //tmp now stores common skin-face pixels

  if(totalArea == 0)
  {
    skinFaceRatio = 0.; // if no face was found, skinFaceRatio for this frame is 0
  }
  else
  {
    skinFaceRatio = round( cv::norm(tmp, cv::NORM_L1, cv::noArray()) / 255.) / (float)totalArea;
  }

  if (skinFaceRatio >= 0.01)
  {
    skinFactor = 1.;
  }
  else if(skinFaceRatio >= 0.005 && skinFaceRatio < 0.01)
  {
    skinFactor = 0.8;
  }
  else if(skinFaceRatio >= 0.001 && skinFaceRatio < 0.005)
  {
    skinFactor = 0.7;
  }
  else if(skinFaceRatio >= 0.0005 && skinFaceRatio < 0.001)
  {
    skinFactor = 0.4;
  }
  else
  {
    skinFactor = 0.;
  }

  probability = 0.7 * probability + 0.3 * skinFactor;
  std::cout << "skinFaceRatio: " << skinFaceRatio << std::endl;
  std::cout << "skinFaceFactor: " << skinFactor << std::endl;

  skinDetector->deallocateMemory();
}

/**
  @brief Initializes frame and probability buffer
  @param image [cv::Mat] The current frame
  @return void
*/
void FaceDetector::initFrameProbBuffers(cv::Mat frame)
{
  if(frame_buffer.empty())
  {
    for(int ii = 0; ii < _bufferSize; ii++)
    {
      cv::Mat tmp = cv::Mat::zeros(frame.size().width, frame.size().height, CV_8UC1);
      frame_buffer.push_back(tmp);
    }
  }
  if(probability_buffer.empty())
  {
    for(int ii = 0; ii < _bufferSize; ii++)
    {
      probability_buffer.push_back(0);
    }
  }
}

/**
  @brief Crate rectangles to current frame according to the positions
    of faces found in previous frames
  @param frameIN [cv::Mat] The frame to be scanned for faces
  @return void
*/
void FaceDetector::createRectangles(cv::Mat tmp)
{
  cv::Rect faceRect;
  cv::Point start;
  cv::Point end;
  for(int i = 0; i < ( faces_total.size() ? (faces_total.size()) : 0) ; i++)
  {
    faceRect = faces_total.at(i);
    start = cv::Point( faceRect.x , faceRect.y );
    end = cv::Point( faceRect.x + faceRect.width, faceRect.y + faceRect.height );
    cv::rectangle(tmp, start, end, cv::Scalar(255, 255, 255, 0), CV_FILLED);
  }
}

/**
  @brief Rotates the given frame in 5 main angles and
    searches for faces in each rotated frame.
  @param frameIN [cv::Mat] The frame to be scanned for faces
  @return 	integer of the sum of faces found in all rotations
  of the frame.
*/
int FaceDetector::findFaces1Frame(cv::Mat frame)
{
  float angle[] = { 0, 45, 315, 90, 270 };

  int facesNum_total = 0;
  int faceNum = 0;
  for(int i = 0 ; i < angleNum ; i++)
  {
    int facesNum = 0;
    cv::Mat src(frame.size().width ,
                frame.size().height, CV_8UC3);
    src = frame.clone();
    if(!src.data)
    {
      ROS_ERROR("No image data in current thread");
    }
    cv::Mat rotated_frame;
    rotated_frame = frameRotate(src , angle[i]);
    if(!rotated_frame.data)
    {
      ROS_INFO("Rotated frame is empty");
    }
    facesNum = detectFace(rotated_frame);

    facesNum_total += facesNum;
  }

  //!< Number of Faces is the sum of all faces found in
  //!< each rotated frame
  return facesNum_total;
}

/**
  @brief Creates the continuous table of faces found that contains
  information for each face in every set of 4 values:
  table[i*4]		=	face #i position x center
  table[i*4+1]	=	face #i position y center
  table[i*4+2]	=	face #i rectangle width
  table[i*4+3]	=	face #i rectangle height
  @return int[] table of face positions and sizes
*/
int* FaceDetector::getFacePositionTable()
{
  cv::Rect faceRect;
  int* table = new int[ 4 * faces_total.size() ];
  for(int ii = 0; ii < faces_total.size(); ii++)
  {
    faceRect = faces_total.at(ii);

    //!< Face center_x
    table[ii * 4]   = round( faceRect.x + faceRect.width * 0.5 );

    //!< Face center_y
    table[ii * 4 + 1] = round( faceRect.y + faceRect.height * 0.5 );

    //!< Face width
    table[ii * 4 + 2] = faceRect.width;

    //!< Face height
    table[ii * 4 + 3] = faceRect.height;
  }
  return table;
}

/**
  @brief Returns the size of the table with the positions of the
  faces found
  @return [int] size of table
*/
int FaceDetector::getFaceTableSize()
{
  return 4 * faces_total.size();
}

/**
  @brief Returns the probability of the faces detected in the frame
  @return [float] probability value
*/
float FaceDetector::getProbability()
{
  return probability;
}

/**
  @brief Calls detectMultiscale to scan frame for faces and drawFace
    to create rectangles around the faces found in each frame
  @param frame [cv::Mat] the frame to be scaned.
  @param cascade [cv::CascadeClassifier] the classifier used for
    detection
  @param	angle [float] the rotation angle
  @return [int] the number of faces found in each frame
*/
int FaceDetector::detectFace(cv::Mat img)
{
  cv::Mat original(img.size().width, img.size().height, CV_8UC1);
  original = img.clone();
  cv::Mat gray(img.size().width, img.size().height, CV_8UC1);
  cvtColor(original, gray, CV_BGR2GRAY);
  std::vector< cv::Rect_<int> > thrfaces;

  int im_width = 92;
  int im_height = 112;

  if(!cascade.empty())
  {
    //!< Find the faces in the frame:
    cascade.detectMultiScale(gray, thrfaces);
    for(int i = 0; i < thrfaces.size(); i++)
    {
      //!< Process face by face:
      cv::Rect face_i = thrfaces[i];
      cv::Mat face = gray(face_i);
      cv::Mat face_resized;
      cv::resize(face, face_resized, cv::Size(im_width, im_height), 1.0, 1.0, cv::INTER_CUBIC);
      int prediction = model->predict(face_resized);
      ROS_INFO_STREAM("Prediction " << prediction);
      rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
      //!< Add every element created for each frame, to the total amount of faces
      faces_total.push_back (thrfaces.at(i));
    }
  }
  int res = thrfaces.size();
  thrfaces.erase (thrfaces.begin(), thrfaces.begin() + thrfaces.size());
  return res;

}

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
cv::Mat FaceDetector::frameRotate(cv::Mat frame, float thAngle)
{
  float iImageCenterY = frame.rows / 2;
  float iImageCenterX = frame.cols / 2;

  //!< Calculate rotation matrix
  cv::Mat matRotation = getRotationMatrix2D(
                          cv::Point( iImageCenterX, iImageCenterY ), (thAngle - 180), 1.1 );

  cv::Mat rotated_frame;
  cv::warpAffine( frame, rotated_frame, matRotation, rotated_frame.size() );

  return rotated_frame;
}

}
