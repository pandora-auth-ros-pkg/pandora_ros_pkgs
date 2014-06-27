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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/face_detector.h"

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
   @return void
  */
  FaceDetector::FaceDetector(std::string cascade_path, std::string model_path,
                           int bufferSize)
  {
    trained_cascade.load(cascade_path);
    //~ ROS_ERROR("%s", cascade_path.c_str());
    if(trained_cascade.empty())
    {
      ROS_ERROR("[Face Detector]: Cannot load cascade classifier");
      exit(0);
    }

    trained_model = cv::createFisherFaceRecognizer();
    trained_model->load(model_path);

    _bufferSize = 1;
   
    now = 0;

    probability = 1.;
    probability_buffer = std::vector<float>(_bufferSize);
  }

  /**
   @brief Class Destructor
   Deallocates all memory used for storing sequences of faces,
   matrices and images
  */
  FaceDetector::~FaceDetector()
  {
    //! Erase frame and probability buffers
    if (!frame_buffer.empty())
    {
      frame_buffer.erase (frame_buffer.begin(), frame_buffer.begin() +
                          frame_buffer.size());
    }

    if (!probability_buffer.empty())
    {
      probability_buffer.clear();
    }
  }


  /**
    @brief Detects number of faces found in current frame.
    The image buffer contributs to probability.
    @param frame [cv::Mat] The frame to be scanned for faces
    @return Integer of the sum of faces found in all
    rotations of the frame
  */
  int FaceDetector::findFaces(cv::Mat frame)
  {
    int facesNum = 0;
    cv::Mat tmp;
    tmp = cv::Mat::zeros(frame.size().width, frame.size().height , CV_8UC1);

    initFrameProbBuffers(frame);
    createRectangles(&tmp);

    //! Clear vector of faces before using it for the current frame
    faces_total.clear();

    facesNum = detectFace(frame);
    ROS_INFO_STREAM("Number of faces: "<< facesNum);

    int totalArea = 0;
    if(facesNum)
    {
      totalArea = round( cv::norm(tmp, cv::NORM_L1, cv::noArray()) / 255.);
    }
   
    if(totalArea == 0)
    {
      //! if no face was found, probability for this frame is 0
      probability_buffer[now] = 0.;
    }
    else
    {
      probability_buffer[now] = round( 
        cv::norm(tmp, cv::NORM_L1, 
          cv::noArray()) / 255.) / static_cast<float>(totalArea);
    }
    //! Clear value from last scan and calculate probability
    probability = 0.0;
    for(int i = 0 ; i < _bufferSize ; i++){
      probability += (probability_buffer[i]);
    }
    probability = probability / _bufferSize;
    
    //~ ROS_INFO_STREAM("probability"<< probability);
    now = (now + 1) % _bufferSize; //prepare index for next frame
        
    return facesNum;
  }


  /**
    @brief Initializes frame and probability buffer
    @param frame [cv::Mat] The current frame
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
    @param tmp [cv::Mat] The frame to be scanned for faces
    @return void
  */
  void FaceDetector::createRectangles(cv::Mat *tmp)
  {
    cv::Rect faceRect;
    cv::Point start;
    cv::Point end;
    for(int i = 0; i < faces_total.size(); i++)
    {
      faceRect = faces_total.at(i);
      start = cv::Point( faceRect.x , faceRect.y );
      end = cv::Point( faceRect.x + faceRect.width, faceRect.y + faceRect.height);
      cv::rectangle(*tmp, start, end, cv::Scalar(255, 255, 255, 0), CV_FILLED);
    }
  }


  /**
    @brief Creates the continuous table of faces found that contains
    information for each face in every set of 4 values:
    @return int[] table of face positions and sizes
  */
  int* FaceDetector::getFacePositionTable()
  {
    int* table = new int[ 4 * faces_total.size() ];
    for(int ii = 0; ii < faces_total.size(); ii++)
    {
      cv::Rect faceRect = faces_total.at(ii);

      //! Face center_x
      table[ii * 4]   = round( faceRect.x + faceRect.width * 0.5 );

      //! Face center_y
      table[ii * 4 + 1] = round( faceRect.y + faceRect.height * 0.5 );

      //! Face width (rectangle width)
      table[ii * 4 + 2] = faceRect.width;

      //! Face height (rectangle height)
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
    @param img [cv::Mat] the frame to be scaned.
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
    
    if(!trained_cascade.empty())
    {
      //! Find the faces in the frame:
      trained_cascade.detectMultiScale(gray, thrfaces);
      ROS_INFO_STREAM("thrfaces.size()" <<thrfaces.size());
      for(int i = 0; i < thrfaces.size(); i++)
      {
        //! Process face by face:
        cv::Rect face_i = thrfaces[i];
        cv::Mat face = gray(face_i);
        cv::Mat face_resized;
        cv::resize(face, face_resized, cv::Size(im_width, im_height), 
          1.0, 1.0, cv::INTER_CUBIC);
        int prediction = trained_model->predict(face_resized);
        ROS_INFO_STREAM("Prediction " << prediction);
        rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
        //! Add every element created for each frame, to the total amount of faces
        faces_total.push_back (thrfaces.at(i));
      }
    }
    
    //~ cv::imshow("face_detector", original);
    //~ cv::waitKey(30);
    int res = thrfaces.size();
 
    thrfaces.clear();
    return res;
  }

}// namespace pandora_vision
