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


#ifndef PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H
#define PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H
 
#include "pandora_vision_hazmat/detection/feature_matching_detector.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /*
     * @class SurfDetector
     * @brief : A planar object detector that uses SURF features to match
     * the pattern with any candidates on the frame.
    */
    class SurfDetector : public FeatureMatchingDetector 
    {
      public:

        /*
         * @brief : The constructor for the SURF detector objects
         */
        SurfDetector();

        /*
         * @brief : The destructor for the SURF detector objects that deletes
         *          the memory allocated for the matcher object.
         */
        ~SurfDetector()
        {
          delete[] matchers_;
        };

        /*
         * @brief: Function used to produce the necessary keypoints and their
         *          corresponding descriptors for an image. 
         * @param frame[const cv::Mat&] : The images that will be processed to 
         * extract features and keypoints.
         * @param mask[const cv::Mat&] : A mask defines the image regions that
         * will be processed.
         * @param descriptors[cv::Mat*]: A pointer to the array that will be
         * used to store the descriptors of the current image.
         * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the
         * vector containing the Keypoints detected in the current image.
         */
        void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
            , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints );

      private:

        cv::SURF s_; //<! The object that calculates SURF keypoints 
        //<! and descriptors.

    };

} // namespace pandora_vision_hazmat
} // namespace pandora_vision

#endif  // PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H_
