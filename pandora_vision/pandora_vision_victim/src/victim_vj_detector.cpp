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

#include "pandora_vision_victim/victim_vj_detector.h"

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
  VictimVJDetector::VictimVJDetector(
    std::string cascade_path, 
    std::string model_path)
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
  }

  /**
   @brief Class Destructor
   Deallocates all memory used for storing sequences of faces,
   matrices and images
  */
  VictimVJDetector::~VictimVJDetector()
  {
  }


  /**
    @brief Detects number of faces found in current frame.
    The image buffer contributs to probability.
    @param frame [cv::Mat] The frame to be scanned for faces
    @return Integer of the sum of faces found in all
    rotations of the frame
  */
  std::vector<DetectedVictim> VictimVJDetector::findFaces(cv::Mat frame)
  {
    cv::Mat tmp;
    tmp = cv::Mat::zeros(frame.size().width, frame.size().height , CV_8UC1);

    //! Clear vector of faces before using it for the current frame
    faces_total.clear();

    std::vector<DetectedVictim> candidateVictim;
    std::vector<float> preds = detectFace(frame);
    std::vector<float> probs = predictionToProbability(preds);
    std::vector<BoundingBox> keypoints = getAlertKeypoints();
    
    if(probs.size() != keypoints.size())
    {
      ROS_FATAL("[PANDORA_VICTIM] Something went terribly wrong");
    }
    
    for(unsigned int i = 0 ; i < probs.size() ; i++)
    {
      DetectedVictim dv;
      dv.probability = probs[i];
      dv.keypoint = keypoints[i].keypoint;
      dv.boundingBox = keypoints[i].bounding_box;
      candidateVictim.push_back(dv);
    }
    
    return candidateVictim;
  }

  /**
    @brief Creates the continuous table of faces found that contains
    information for each face in every set of 4 values:
    @return int[] table of face positions and sizes
  */
  std::vector<BoundingBox> VictimVJDetector::getAlertKeypoints()
  {
    std::vector<BoundingBox> table;
    for(int ii = 0; ii < faces_total.size(); ii++)
    {
      BoundingBox tbb;
      cv::Rect faceRect = faces_total.at(ii);
      cv::Point2f p (
        round( faceRect.x + faceRect.width * 0.5 ),
        round( faceRect.y + faceRect.height * 0.5 )
      );
      tbb.keypoint = p;
      tbb.bounding_box = faceRect;
      table.push_back(tbb);
    }
    return table;
  }

  /**
    @brief Returns the probability of the faces detected in the frame
    @return [float] probability value
  */  
  std::vector<float> VictimVJDetector::predictionToProbability
    (std::vector<float> prediction)
  {
    std::vector<float> p;
    for(unsigned int i = 0 ; i < prediction.size() ; i++)
    {
      //~ Normalize probability to [-1,1]
      float temp_prob = tanh(0.5 * (prediction[i] - 20.0) );
      //~ Normalize probability to [0,1]
      temp_prob = (1 + temp_prob) / 2.0;
      ROS_INFO_STREAM("Viola pred/prob :" << prediction[i] <<" "<<temp_prob);
      p.push_back(temp_prob);
    }
    return p;
  }

  /**
    @brief Calls detectMultiscale to scan frame for faces and drawFace
      to create rectangles around the faces found in each frame
    @param img [cv::Mat] the frame to be scaned.
    @return [int] the number of faces found in each frame
  */
  std::vector<float> VictimVJDetector::detectFace(cv::Mat img)
  {
    std::vector<float> predictions;
    cv::Mat original(img.size().width, img.size().height, CV_8UC1);
    original = img.clone();
    cv::Mat gray(img.size().width, img.size().height, CV_8UC1);
    if(original.channels() != 1)
    {
      cvtColor(original, gray, CV_BGR2GRAY);
    }
    std::vector< cv::Rect_<int> > thrfaces;

    int im_width = VictimParameters::modelImageWidth;
    int im_height = VictimParameters::modelImageHeight;
    
    if(!trained_cascade.empty())
    {
      //! Find the faces in the frame:
      trained_cascade.detectMultiScale(gray, thrfaces);
      for(int i = 0; i < thrfaces.size(); i++)
      {
        //! Process face by face:
        cv::Rect face_i = thrfaces[i];
        cv::Mat face = gray(face_i);
        cv::Mat face_resized;
        cv::resize(face, face_resized, cv::Size(im_width, im_height), 
          1.0, 1.0, cv::INTER_CUBIC);
        double local_conf = 0.0;
        int pred_label = -1;
        trained_model->predict(face_resized, pred_label, local_conf);
        predictions.push_back(local_conf);
        rectangle(original, face_i, CV_RGB(0, 255, 0), 1);
        //! Add every element created for each frame, to the total amount of faces
        faces_total.push_back (thrfaces.at(i));
      }
    }
 
    thrfaces.clear();
    return predictions;
  }

}// namespace pandora_vision
