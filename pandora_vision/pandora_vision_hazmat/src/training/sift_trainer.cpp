/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/
#include "pandora_vision_hazmat/training/sift_trainer.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param images[const cv::Mat&] : The images that will be processed to 
     * extract features and keypoints.
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used
     * to store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     **/  
    bool SiftTrainer::getFeatures(const cv::Mat& image, cv::Mat* descriptors,
        std::vector<cv::KeyPoint>* keyPoints)
    {
      if (!image.data)
        return false;
      // Calculate image keypoints.
      featureDetector_->detect(image, *keyPoints);

      // Extract Descriptors from image
      featureExtractor_->compute( image , *keyPoints , *descriptors );
      return true;
    }
    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param images[const std::vector<cv::Mat&>] : The image we want 
     * to process.
     * @param descriptors[cv::Mat*]: A pointer to the vector that will be 
     * used to store the descriptors for each image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in every image.
     **/
    bool SiftTrainer::getFeatures(const std::vector<cv::Mat>& images,
        std::vector<cv::Mat>* descriptors,
        std::vector<std::vector<cv::KeyPoint> >* keyPoints)
    {
      // Calculate image keypoints.
      featureDetector_->detect( images , *keyPoints );  

      cv::Mat tempDescriptorMat;
      // Extract Descriptors from the images.
      for (int i = 0 ; i < keyPoints->size(); i++)
      {
        if (!images[i].data)
          continue;
        featureExtractor_->compute( images[i] , (*keyPoints)[i] ,
            tempDescriptorMat);
        descriptors->push_back(tempDescriptorMat);
      }
      return descriptors->size() > 0;
    }

    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param images[const cv::Mat&] : The images that will be processed to 
     * extract features and keypoints.
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used
     * to store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     * @param boundingBox[std::vector<cv::Point2f>*] : A pointer to the vector
     * containing the bounding box for the pattern in the current image.
     **/  

    bool SiftTrainer::getFeatures(const cv::Mat& image, cv::Mat* descriptors, 
        std::vector<cv::KeyPoint>* keyPoints , 
        std::vector<cv::Point2f>* boundingBox )
    {
      if (!image.data)
        return false;
      bool featureFlag = this->getFeatures(image, descriptors, keyPoints);

      // Calculate the bounding box for the current pattern 
      (*boundingBox).push_back( cv::Point2f( 0.0f , 0.0f  )); 
      (*boundingBox).push_back(  cv::Point2f( image.cols , 0));
      (*boundingBox).push_back ( cv::Point2f( image.cols  , 
            image.rows)); 
      (*boundingBox).push_back( cv::Point2f( 0.0f , 
            image.rows));

      (*boundingBox).push_back( cv::Point2f( image.cols / 2.0f ,
            image.rows / 2.0f));
      return featureFlag;
    }


} // namespace pandora_vision_hazmat
} // namespace pandora_vision

