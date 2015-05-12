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

#include "pandora_vision_hazmat/detection/surf_detector.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /*
     * @brief : The constructor for the SURF detector objects
     */
    SurfDetector::SurfDetector() :
      FeatureMatchingDetector("SURF")
    {
      int patternNum = this->getPatternsNumber();

      // Initialize the matchers that will be used for the 
      // detection of the pattern.
      matchers_ = new cv::Ptr<cv::DescriptorMatcher>[patternNum];
      // A temporary container for the descriptors of each pattern.
      std::vector<cv::Mat> descriptors;

      for (int i = 0 ; i < patternNum ; i++ )
      {
        matchers_[i] = cv::DescriptorMatcher::create("FlannBased");
        // Add the descriptors of the i-th pattern to the 
        // container.
        descriptors.push_back( (*patterns_ )[i].descriptors );
        // Add the descriptors to the matcher and train it.
        matchers_[i]->add( descriptors );
        matchers_[i]->train();

        // Clear the container for the next iteration.
        descriptors.clear();
      }

      // Initialize the keypoint detector and the feature extractor
      // that will be used.
      s_ = cv::SURF();
    }

    /*
     * @brief: Function used to produce the necessary keypoints and their
     *          corresponding descriptors for an image. 
     * @param frame[const cv::Mat&] : The images that will be processed to 
     * extract features and keypoints.
     * @param mask[const cv::Mat&] : A mask defines the image regions that
     * will be processed.
     * @param descriptors[cv::Mat*]: A pointer to the array that will be used
     * to store the descriptors of the current image.
     * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the vector
     * containing the Keypoints detected in the current image.
     */
    void SurfDetector::getFeatures( const cv::Mat &frame , 
        const cv::Mat &mask , cv::Mat *descriptors , 
        std::vector<cv::KeyPoint> *keyPoints ) 
    {
#ifdef FEATURES_CHRONO
      gettimeofday( &startwtime , NULL );
#endif

      // Detect the Keypoints on the image.
      s_.detect( frame ,  *keyPoints ,  mask );

#ifdef FEATURES_CHRONO
      gettimeofday( &endwtime, NULL );
      double keyPointTime = static_cast<double>((endwtime.tv_usec -
            startwtime.tv_usec) / 1.0e6 + endwtime.tv_sec -
          startwtime.tv_sec);
#endif
#ifdef FEATURES_CHRONO
      gettimeofday( &startwtime , NULL );
#endif

      // Extract descriptors for the the detected keypoints.
      s_.compute( frame, *keyPoints , *descriptors);


#ifdef FEATURES_CHRONO
      gettimeofday( &endwtime, NULL );
      double descriptorsTime = static_cast<double>((endwtime.tv_usec -
            startwtime.tv_usec) / 1.0e6 + endwtime.tv_sec -
          startwtime.tv_sec);  
      if (featureTimerFlag_)
      {  
        ROS_DEBUG_STREAM_NAMED("detection", "[Hazmat Detection]: ["
            << featuresName_ << "] : Descriptors Computation time : "
            << descriptorsTime);
        ROS_DEBUG_STREAM_NAMED("detection", "[Hazmat Detection]: ["
            << featuresName_ << "] : Keypoint Extraction time : "
            << descriptorsTime);
      }
#endif
    }

} // namespace pandora_vision_hazmat
} // namespace pandora_vision
