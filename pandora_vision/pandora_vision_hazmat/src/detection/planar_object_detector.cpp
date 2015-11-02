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

#include <string>
#include <vector>

#include "pandora_vision_hazmat/detection/planar_object_detector.h"

namespace pandora_vision
{
  namespace pandora_vision_hazmat
  {
    int PlanarObjectDetector::width_ = 640;
    int PlanarObjectDetector::height_ = 480;
    std::string PlanarObjectDetector::fileName_ = std::string();

    /**
     * @brief : This function is used to detect the desired
     * objects in the current frame .
     * @param frame[const cv::Mat&] : The frame that will be processed.
     * @param detectedObjects[std::vector<Object>*] : A vector containing
     * all the objects detected in the scene.
     * @return bool : True if an object has been detected,false otherwise.
     */
    bool PlanarObjectDetector::detect(const cv::Mat& frame,
        std::vector<POIPtr>* detectedObjects)
    {
      // Check if the frame is not an empty matrix.
      if (!frame.data)
      {
        ROS_ERROR_STREAM_NAMED("detection", "[Hazmat Detection]:" <<
            " The provided frame is empty!");
        return false;
      }
      if (patterns_ == NULL)
      {
        ROS_FATAL_STREAM("[Hazmat Detection]: Pointer to pattern array" <<
            " is NULL!");
        ROS_BREAK();
      }
      // Check if that patterns have been read succesfully.
      if (patterns_->size() < 1)
      {
        ROS_FATAL_STREAM("[Hazmat Detection]: No patterns read." <<
            " Detection cannot continue ");
        return false;
      }

      bool foundPattern = false;
      // The matrix that contains the descriptors of the frame.
      cv::Mat frameDescriptors;
      // The mask that will be applied to the frame so as to limit the
      // the search space.
      cv::Mat mask;
      // The detected keypoints of the frame that will be matched to the
      // input pattern so as to find the correspondence from the training
      // set to the query frame.
      std::vector<cv::KeyPoint> frameKeyPoints;

#ifdef CHRONO
      gettimeofday(&startwtime , NULL);
#endif

      // Create the mask that will be used to extract regions of interest
      // on the image based on the Image Signature Saliency Map.
      maskCreatorPtr_->createSaliencyMapMask(frame, &mask);
      // Calculate the keypoints and extract the descriptors from the
      // frame.

      if (maskDisplayFlag_)
      {
        cv::Mat maskedFrame;
        frame.copyTo(maskedFrame, mask);
        imshow("Saliency Map Mask", maskedFrame);
        cv::waitKey(5);
      }

#ifdef CHRONO
      gettimeofday(&startwtime, NULL);
#endif

      getFeatures(frame, mask, &frameDescriptors, &frameKeyPoints);

#ifdef CHRONO
      gettimeofday(&endwtime , NULL);
      double featuresTime = static_cast<double>((endwtime.tv_usec -
            startwtime.tv_usec) / 1.0e6
            + endwtime.tv_sec - startwtime.tv_sec);
      ROS_DEBUG_STREAM_NAMED("detection", "Calculation time for the"
          << " features in the frame is " << featuresTime);
#endif

      bool matchesFound;
      bool boundingBoxFound;

      std::vector<cv::Point2f> patternKeyPoints;
      std::vector<cv::Point2f> sceneKeyPoints;
      std::vector<cv::Point2f> sceneBB;

      // Temporary variables used to store the detected center of a pattern.
      float x , y;

      cv::Mat trackingFrame;
      if (displayResultsFlag_)
        frame.copyTo(trackingFrame, cv::Mat());

      // For every pattern in the list :
      for (int i = 0; i < patterns_->size(); i++)
      {
#ifdef CHRONO
        gettimeofday(&startwtime, NULL);
#endif

        // Try to find key point matches between the i-th pattern
        // and the descriptors and the keypoints extracted from the frame.
        matchesFound = findKeypointMatches(frameDescriptors,
            (*patterns_)[i].descriptors, (*patterns_)[i].keyPoints,
            frameKeyPoints,
            &patternKeyPoints,
            &sceneKeyPoints, i);

#ifdef CHRONO
        gettimeofday(&endwtime, NULL);
        double keyPointTime = static_cast<double>((endwtime.tv_usec -
              startwtime.tv_usec) / 1.0e6 +
              endwtime.tv_sec - startwtime.tv_sec);
        ROS_DEBUG_STREAM_NAMED("detection", "Calculation time for the"
            " keypoints in the frame is " << featuresTime);
#endif

        // If we have succesfully found the matches.
        if (matchesFound)
        {
#ifdef CHRONO
          gettimeofday(&startwtime , NULL);
#endif

          // Find the bounding box for this query pattern.
          boundingBoxFound = findBoundingBox(patternKeyPoints,
              sceneKeyPoints, (*patterns_)[i].boundingBox, &sceneBB);

#ifdef CHRONO
          gettimeofday(&endwtime, NULL);
          double boundingBoxTime = static_cast<double>((endwtime.tv_usec -
                startwtime.tv_usec) / 1.0e6
              + endwtime.tv_sec - startwtime.tv_sec);
          ROS_DEBUG_STREAM_NAMED("detection", "Calculation time for the"
              << " bounding box for the pattern " << i << "  is : "
              << boundingBoxTime);
#endif

          // If this flag is true then a valid match has been found and
          // we have detected the pattern.
          if (boundingBoxFound)
          {
            if (displayResultsFlag_)
            {
              cv::line(trackingFrame, sceneBB[0], sceneBB[1],
                  cv::Scalar(0, 255, 0), 4);
              cv::line(trackingFrame, sceneBB[1], sceneBB[2],
                  cv::Scalar(0, 255, 0), 4);
              cv::line(trackingFrame, sceneBB[3], sceneBB[0],
                  cv::Scalar(0, 255, 0), 4);
              cv::line(trackingFrame, sceneBB[2], sceneBB[3],
                  cv::Scalar(0, 255, 0), 4);
              cv::circle(trackingFrame, cv::Point2f(sceneBB[4].x,
                    sceneBB[4].y), 4.0, cv::Scalar(0, 0, 255), -1, 8);
            }

            cv::Point2f base = sceneBB[0] - sceneBB[1];
            cv::Point2f h = sceneBB[1] - sceneBB[2];
            double sideA = cv::norm(base);
            double sideB = cv::norm(h);
            double surface = sideA * sideB;

            foundPattern |= boundingBoxFound;

            // Get the center of the detected pattern.
            x = sceneBB[sceneBB.size() - 1].x;
            y = sceneBB[sceneBB.size() - 1].y;

            boost::shared_ptr<HazmatPOI> hazmatPOIPtr(new HazmatPOI);
            hazmatPOIPtr->setName((*patterns_)[i].name);
            hazmatPOIPtr->setPoint(cv::Point2f(x, y));
            hazmatPOIPtr->setPatternId((*patterns_)[i].hazmatNum);
            hazmatPOIPtr->setName((*patterns_)[i].hazmatLabelName);

            detectedObjects->push_back(hazmatPOIPtr);
          }
        }
        patternKeyPoints.clear();
        sceneBB.clear();
        sceneKeyPoints.clear();
      }

      if (displayResultsFlag_)
      {
        cv::imshow("Tracking Frame", trackingFrame);
        cv::waitKey(5);
      }

      patternKeyPoints.clear();
      sceneBB.clear();
      sceneKeyPoints.clear();

      return foundPattern;
    }

    /**
     * @brief Find the homography between the scene and the pattern keypoints,
     * check if it is valid and return the bounding box of the detected
     * pattern.
     * @param patternKeyPoints [std::vector<cv::KeyPoint> &] : Input
     * keypoints from detected descriptor matches on the pattern.
     * @param sceneKeyPoints [std::vector<cv::KeyPoint> &] : Input
     * keypoints from detected descriptor matches in the scene.
     * @param patternBB [std::vector<cv::Point2f *] : Vector of 2D float
     * Points that containes the bounding box and the center of the
     * pattern.
     */
    bool PlanarObjectDetector::findBoundingBox(
        const std::vector<cv::Point2f>& patternKeyPoints,
        const std::vector<cv::Point2f>& sceneKeyPoints,
        const std::vector<cv::Point2f>& patternBB,
        std::vector<cv::Point2f>* sceneBB)
    {
      bool boundingBoxFound = false;

      if (patternBB.size() <= 0)
      {
        ROS_DEBUG_STREAM_NAMED("detection", "No bounding box has been"
                               << " read for the current pattern!");
        return boundingBoxFound;
      }

      // Check if we have enough points to find the homography between
      // the pattern and the scene.
      if (patternKeyPoints.size() > 4 && sceneKeyPoints.size() > 4)
      {
        // Calculate the homography matrix using RANSAC algorithm to
        // eliminate outliers.
        cv::Mat H = cv::findHomography(patternKeyPoints, sceneKeyPoints,
            CV_RANSAC, 5);
        // Transform the bounding box to the frame coordinates.
        cv::perspectiveTransform(patternBB, *sceneBB, H);

        // Check if every point of the bounding box is inside the image.
        // If not then the correspondences are invalid and these keypoints
        // are rejected.
        for (int i = 0; i < sceneBB->size(); i++)
        {
          if (((*sceneBB)[i].x < 0)
              || ((*sceneBB)[i].x > PlanarObjectDetector::width_)
              || ((*sceneBB)[i].y < 0)
              || ((*sceneBB)[i].y > PlanarObjectDetector::height_))
          {
            sceneBB->clear();
            // The bounding box has a point out of the screen so it is
            // rejected.
            return boundingBoxFound;
          }
        }

        // Check if the Bounding box is Convex
        // If not the resulting homography is incorrect due to false
        // matching between the descriptors.
        std::vector<cv::Point2f> boundingBox = *sceneBB;
        boundingBox.pop_back();
        if (!cv::isContourConvex(boundingBox))
        {
          return boundingBoxFound;
        }
        // A convex bounding box has been found inside the frame.
        boundingBoxFound = true;

        // Clear the bounding box vector.
        return boundingBoxFound;
      }
      else
        // Not enough points for the homography.
        return boundingBoxFound;
    }
}  // namespace pandora_vision_hazmat
}  // namespace pandora_vision
