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
#ifndef PANDORA_VISION_HAZMAT_TRAINING_SURF_TRAINER_H
#define PANDORA_VISION_HAZMAT_TRAINING_SURF_TRAINER_H

#include <string>
#include <vector>

#include "pandora_vision_hazmat/training/planar_pattern_trainer.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @class SurfTrainer
     * @brief Class that uses the SURF algorithm for HAZMAT pattern training.
     */
    class SurfTrainer : public PlanarPatternTrainer
    {
      public:
        /**
         * @brief Constructor for a Hazmat trainer module that uses SURF.
         */
        SurfTrainer() : featureType_("SURF")
        {
          featureDetector_ = cv::FeatureDetector::create(featureType_);
          // Feature Extractor Initialization.
          featureExtractor_ = cv::DescriptorExtractor::create(featureType_);
        }

        virtual ~SurfTrainer()
        {
        }

        /**
         * @brief: Function used to produce the necessary keypoints and their
         * corresponding descriptors for an image.
         * @param image[const cv::Mat&] : The images that will be processed to
         * extract features and keypoints.
         * @param descriptors[cv::Mat*]: A pointer to the array that will be
         * used to store the descriptors of the current image.
         * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the
         * vector containing the Keypoints detected in the current image.
         */
        bool getFeatures(const cv::Mat& image, cv::Mat* descriptors,
            std::vector<cv::KeyPoint>* keyPoints);

        /**
         * @brief: Function used to produce the necessary keypoints and their
         * corresponding descriptors for an image.
         * @param images[const std::vector<cv::Mat&>] : The image we want
         * to process.
         * @param descriptors[cv::Mat*]: A pointer to the vector that will be
         * used to store the descriptors for each image.
         * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the
         * vector containing the Keypoints detected in every image.
         */
        virtual bool getFeatures(const std::vector<cv::Mat>& images,
            std::vector<cv::Mat>* descriptors,
            std::vector<std::vector<cv::KeyPoint> >* keyPoints);

        /**
         * @brief: Function used to produce the necessary keypoints and their
         * corresponding descriptors for an image.
         * @param image[const cv::Mat&] : The images that will be processed to
         * extract features and keypoints.
         * @param descriptors[cv::Mat*]: A pointer to the array that will be
         * used  to store the descriptors of the current image.
         * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the
         * vector containing the Keypoints detected in the current image.
         * @param boundingBox[std::vector<cv::Point2f>*] : A pointer to the
         * vector containing the bounding box for the pattern in the current
         * image.
         */
        virtual bool getFeatures(const cv::Mat& image, cv::Mat *descriptors ,
            std::vector<cv::KeyPoint> *keyPoints ,
            std::vector<cv::Point2f> *boundingBox);

        /**
         *  @brief: Returns the type of features that were used to describe
         *  the pattern.
         *  @return std::string : The name of the feature.
         */
        std::string getFeatureType()
        {
          return featureType_;
        }
      private:
        cv::Ptr<cv::FeatureDetector> featureDetector_;
        cv::Ptr<cv::DescriptorExtractor> featureExtractor_;
        const std::string featureType_;  ///< ID of the algorithm used.
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_TRAINING_SURF_TRAINER_H
