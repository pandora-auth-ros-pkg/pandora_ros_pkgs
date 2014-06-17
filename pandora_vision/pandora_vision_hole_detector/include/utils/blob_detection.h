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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef UTILS_BLOB_DETECTION_H
#define UTILS_BLOB_DETECTION_H

#include "utils/morphological_operators.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class BlobDetection
    @brief Provides methods for blob detection
   **/
  class BlobDetection
  {
    public:

      /**
        @brief Implements the brushfire algorithm for one blob keypoint
        in order to find its outline points
        @param[in] inKeyPoint [const cv::KeyPoint&] The keypoint
        @param[in] edgesImage [cv::Mat*] The input image
        @param[out] blobOutlineVector [std::vector<cv::Point2f>*]
        The output vector containing the blob's outline
        @param[out] blobArea [float*] The area of the blob
        @return void
       **/
      static void brushfireKeypoint(
        const cv::KeyPoint& inKeyPoint,
        cv::Mat* edgesImage,
        std::vector<cv::Point2f>* blobOutlineVector,
        float* blobArea);

      /**
        @brief Implements the brushfire algorithm for all blob keypoints
        in order to find blobs' outlines
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] edgesImage [cv::Mat*] The input image
        @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point2f> >*]
        The output vector containing the blobs' outline
        @param[out] blobsArea [std::vector<float>*] The area of each blob
        @return void
       **/
      static void brushfireKeypoints(
        const std::vector<cv::KeyPoint>& inKeyPoints,
        cv::Mat* edgesImage,
        std::vector<std::vector<cv::Point2f> >* blobsOutlineVector,
        std::vector<float>* blobsArea);

      /**
        @brief Implements the brushfire algorithm. Its specific purpose is
        to find the points between a blob's outline and its bounding box
        (not necessarily one of least area).
        @param[in] inPoint [const cv::Point2f&] The input point
        @param[in] inImage [cv::Mat*] The input image
        @param[out] visited [std::set<unsigned int>&] The points between
        areas of non-zero value pixels.
        @return void
       **/
      static void brushfirePoint(const cv::Point2f& inPoint,
        cv::Mat* inImage,
        std::set<unsigned int>* visited);

      /**
        @brief Detects blobs in an image
        @param[in] inImage [const cv::Mat&] The input image
        @param[out] keyPointsOut [std::vector<cv::KeyPoint>*] The ouput
        @return void
       **/
      static void detectBlobs(const cv::Mat& inImage,
        std::vector<cv::KeyPoint>* keyPointsOut);

      /**
        @brief Implements a raycast algorithm for a blob keypoint in order
        to find its outline points. The output is a vector of connected points.
        @param[in] inKeyPoint [const cv::KeyPoint&] The keypoint
        @param[in] edgesImage [cv::Mat*] The input image
        @param[in] partitions [const int&] The number of directions
        towards which the outline of the blob will be sought,
        or the number of partitions in which the blob will be divided by
        the rays. Same deal.
        @param[in] findArea [const bool&] Indicates whether to calculate
        the area of the blob
        @param[out] blobOutlineVector [std::vector<cv::Point2f>*]
        The output vector containing the blobs' (rough approximate) outline
        @param[out] blobArea [float*] The blob's area. Non-zero if @param
        findArea true
        @return void
       **/
      static void raycastKeypoint(
        const cv::KeyPoint& inKeyPoint,
        cv::Mat* edgesImage,
        const int& partitions,
        const bool& findArea,
        std::vector<cv::Point2f>* blobOutlineVector,
        float* blobArea);

      /**
        @brief Implements a raycast algorithm for all blob keypoints in order
        to find blobs' outlines. The output is a vector containing a coherent
        vector of points.
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] edgesImage [cv::Mat*] The input image
        @param[in] partitions [const int&] The number of directions
        towards which the outline of the blob will be sought,
        or the number of partitions in which the blob will be divided by
        the rays.  Same deal.
        @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point2f> >*]
        The output vector containing the blobs' (rough approximate) outline
        @param[out] blobsArea [std::vector<float>*] The area of each blob
        @return void
       **/
      static void raycastKeypoints(
        const std::vector<cv::KeyPoint>& inKeyPoints,
        cv::Mat* edgesImage,
        const int& partitions,
        std::vector<std::vector<cv::Point2f> >* blobsOutlineVector,
        std::vector<float>* blobsArea);

  };

} // namespace pandora_vision

#endif  // UTILS_BLOB_DETECTION_H
