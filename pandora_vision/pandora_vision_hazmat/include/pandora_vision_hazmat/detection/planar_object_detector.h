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

#ifndef PANDORA_VISION_HAZMAT_DETECTION_PLANAR_OBJECT_DETECTOR_H
#define PANDORA_VISION_HAZMAT_DETECTION_PLANAR_OBJECT_DETECTOR_H

#include <vector>
#include <string>

#include "pandora_vision_hazmat/detection/detector_interface.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    /**
     * @class PlanarObjectDetector
     * @brief : Class interface for planar object detector modules.
     */
    class PlanarObjectDetector : public Detector
    {
      public:
        /**
         * @brief : This function is used to detect the desired
         * objects in the current frame.
         * @param frame[const cv::Mat&] : The frame that will be processed.
         * @param detectedObjects[std::vector<Object>*] : A vector containing
         * all the objects detected in the scene.
         * @return bool : True if an object has been detected,false otherwise.
         */
        bool virtual detect(const cv::Mat &frame, std::vector<POIPtr>*
            detectedObjects);

        /**
         * @brief Function used to read the necessary training data for
         * the detector to function.
         * @return [bool] : A flag that tells us whether we succeeded in
         * reading the data.
         */
        bool virtual readData(void) = 0;

        /**
         * @brief: Function used to produce the necessary keypoints and their
         * corresponding descriptors for an image.
         * @param frame[const cv::Mat&] : The images that will be processed to
         * extract features and keypoints.
         * @param mask[const cv::Mat&] : A mask defines the image regions that
         * will be processed.
         * @param descriptors[cv::Mat*]: A pointer to the array that will be
         * used to store the descriptors of the current image.
         * @param keyPoints[std::vector<cv::KeyPoint>*] : A pointer to the
         * vector containing the Keypoints detected in the current image.
         */
        void virtual getFeatures(const cv::Mat &frame, const cv::Mat &mask,
            cv::Mat *descriptors, std::vector<cv::KeyPoint> *keyPoints) = 0;

        /**
         * @brief Function used to detect matches between a pattern and the
         * input frame.
         * @param frameDescriptors [const cv::Mat & ] : Descriptors of the
         * current frame.
         * @param patternDescriptors [const cv::Mat &] : Descriptors of the
         * pattern.
         * @param patternKeyPoints [std::vector<cv::KeyPoint> *] : Vector of
         * detected keypoints in the pattern.
         * @param sceneKeyPoints [std::vector<cv::KeyPoint> *] : Vector of
         * detected keypoints in the frame.
         */
        bool virtual findKeypointMatches(const cv::Mat& frameDescriptors,
            const cv::Mat& patternDescriptors,
            const std::vector<cv::Point2f>& patternKeyPoints,
            const std::vector<cv::KeyPoint>& sceneKeyPoints,
            std::vector<cv::Point2f>* matchedPatternKeyPoints,
            std::vector<cv::Point2f>* matchedSceneKeyPoints,
            const int &patternID = 0) = 0;

        /**
         * @brief Find the homography between the scene and the pattern
         * keypoints, check if it is valid and return the bounding box of
         * the detected pattern .
         * @param patternKeyPoints [std::vector<cv::KeyPoint> &] : Input
         * keypoints from detected descriptor matches on the pattern.
         * @param sceneKeyPoints [std::vector<cv::KeyPoint> &] : Input
         * keypoints from detected descriptor matches in the scene.
         * @param patternBB [std::vector<cv::Point2f *] : Vector of 2D float
         * Points that containes the bounding box and the center of the
         * pattern.
         */
        bool virtual findBoundingBox(const std::vector<cv::Point2f>
            &patternKeyPoints,
            const std::vector<cv::Point2f>& sceneKeyPoints,
            const std::vector<cv::Point2f>& patternBB,
            std::vector<cv::Point2f>* sceneBB);

        /**
         * @brief : Returns the name of the features used.
         * @return const std::string : The feature name
         */
        const std::string getFeaturesName(void)
        {
          return this->featuresName_;
        }

        /**
         * @brief : Changes the default pattern file path
         * @param fileName[const std::string &] : The path to the directory
         * containing the training data.
         */
        static void setFileName(const std::string& fileName)
        {
          fileName_ = fileName;
        }

        static void setDims(const cv::Mat & frame)
        {
          width_ = frame.cols;
          height_ = frame.rows;
        }

        /**
         * @brief : Default empty constructor
         */
        PlanarObjectDetector()
        {
          maskCreatorPtr_.reset(new ImageSignature);
        }

        /**
         * @brief : Constructor that initiliazes the vector containing
         * the patterns.
         * @param featureName[const std::string&] : The name of the features.
         */
        explicit PlanarObjectDetector(const std::string &featureName)
          : featuresName_(featureName), patterns_(new std::vector<Pattern>)
        {
          displayResultsFlag_ = false;
          maskDisplayFlag_ = false;
          maskCreatorPtr_.reset(new ImageSignature);
        }

        /**
         * @brief : Constructor that initiliazes the vector containing
         * the patterns and reads the training data from the specified
         * directory.
         * @param featureName[const std::string&] : The name of the features.
         * @param fileName[const std::string&] : The name of the file that
         * contains the training data.
         */
        PlanarObjectDetector(const std::string& featureName,
            const std::string& fileName): featuresName_(featureName),
            patterns_(new std::vector<Pattern>)
        {
          fileName_ = fileName;
          displayResultsFlag_ = false;
          maskCreatorPtr_.reset(new ImageSignature);
        }

        /**
         * @brief : The default destructor of the class.
         */
        virtual ~PlanarObjectDetector()
        {
        }

        /**
         * @brief : Returns the number of patterns we want to detect.
         * @return : The number of patterns.
         */
        int getPatternsNumber(void)
        {
          return patterns_->size();
        }

        /**
         * @brief : Sets the next value of the display flag
         * @param displayResultsFlag[bool] : The new value of the flag
         */
        void setDisplayResultsFlag(bool displayResultsFlag)
        {
          displayResultsFlag_ = displayResultsFlag;
        }

        /**
         * @brief : Sets the next value of the mask display flag
         * @param maskDisplayFlag[bool] : The new value of the flag
         */
        void setMaskDisplayFlag(bool maskDisplayFlag)
        {
          maskDisplayFlag_ = maskDisplayFlag;
        }

        /**
         * @brief : Sets the next value of the flag that decides whether to
         * print or not the execution time of the feature computation
         * @param featureTimerFlag[bool] : The new value of the flag
         */
        void setFeatureTimerFlag(bool featureTimerFlag)
        {
          featureTimerFlag_ = featureTimerFlag;
        }

      protected:
        bool displayResultsFlag_;  //!< Flag used to toggle the display of the
        //!< detected patterns.

        bool maskDisplayFlag_;  //!< Flag used to toggle the display of the
        //!< display mask.

#if defined(CHRONO) || defined(FEATURES_CHRONO)

        bool featureTimerFlag_;  //!< Flag used to toggle the display of the
        //!< execution times of the  detector submodules.
#endif

        boost::shared_ptr< std::vector<Pattern> > patterns_;  //!< The vector
        //!< that contains the patterns we want to detect.

        static std::string fileName_;  //!< Name of the file from which the
        //!< training data is read.

        const std::string featuresName_;  //!< Name of features used.

#if defined(CHRONO) || defined(FEATURES_CHRONO)
        struct timeval startwtime, endwtime;  //!< Structs used for calculating
        //!< the execution time.
#endif
      private :
        static int width_;  //!< Width of the input frame.

        static int height_;  //!< Height of the input frame.

        boost::shared_ptr<ImageSignature> maskCreatorPtr_;
    };
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
#endif  // PANDORA_VISION_HAZMAT_DETECTION_PLANAR_OBJECT_DETECTOR_H
