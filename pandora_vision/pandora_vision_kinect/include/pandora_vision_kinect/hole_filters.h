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
 * Authors: Alexandros Filotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef KINECT_HOLE_FILTERS
#define KINECT_HOLE_FILTERS

#include "pandora_vision_kinect/noise_elimination.h"
#include "pandora_vision_kinect/edge_detection.h"
#include "pandora_vision_kinect/blob_detection.h"
#include "pandora_vision_kinect/bounding_box_detection.h"
#include "pandora_vision_kinect/planes_detection.h"
#include <math.h>

/**
@namespace vision
@brief The main namespace for PANDORA vision
**/
namespace vision
{
  /**
  @class HoleDetector
  @brief Provides the functionalities for detecting holes [functional]
  **/
  class HoleFilters
  {
    public:

      /**
        @brief The structure that represents the overall holes found.
        @param keyPoints [std::vector<cv::KeyPoint>] The vector of the
        holes' keypoints
        @param rectangles [std::vector< std::vector<cv::Point2f> >] The
        vector of the holes' rotated bounding boxes vertices
        @param outlines [std::vector<std::vector<cv::Point> >] The
        vector of the holes' outlines
       **/
      struct HolesConveyor
      {
        std::vector<cv::KeyPoint> keyPoints;
        std::vector< std::vector<cv::Point2f> > rectangles;
        std::vector<std::vector<cv::Point> > outlines;
      };

      /**
        @brief Checks for valid holes just by depth difference between the
        center of the blob and the edges of the bounding box
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The bounding boxes' vertices
        @param[in][out] msgs [std::vector<std::string>&] Messages for debug
        reasons
        @param[in] inflationSize [const int] The number of pixels by which the
        bounding rectange will be inflated
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesDepthDiff(
          const cv::Mat& depthImage,
          const std::vector<cv::KeyPoint>& inKeyPoints,
          const std::vector<std::vector<cv::Point2f> >& inRectangles,
          std::vector<std::string>& msgs,
          const int inflationSize);

      /**
        @brief Checks for valid holes by area / depth comparison
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The bounding boxes' vertices
        @param[in][out] msgs [std::vector<std::string>&] Messages for debug
        reasons
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesDepthArea(
          const cv::Mat& depthImage,
          const std::vector<cv::KeyPoint>& inKeyPoints,
          const std::vector<std::vector<cv::Point2f> >& inRectangles,
          std::vector<std::string>& msgs);


      /**
        @brief Brushfire from a blobs's outline to its bounding box
        with an inflation size (inflates the rectangle by inflationSize pixels).
        If the points between the blob's outline and the inflated rectangle
        lie on one plane, this blob is a hole.
        @param[in] inImage [const cv::Mat] The input depth image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
        The original point cloud acquired from the depth sensor
        @param[in] keyPoints [const std::vector<cv::KeyPoint>] The keypoints of
        blobs
        @param[in] outlines [const std::vector<std::vector<cv::point> >&] The
        points the outline consists of
        @param[in] rectangles [const std::vector<std::vector<cv::point2f> >&]
        The bounding boxes' vertices
        @param inflationsize [const int] Grow the rectangle by inflationsize as
        to acquire more points to check for plane existence.
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesBrushfireOutlineToRectangle(
          const cv::Mat inImage,
          const PointCloudXYZPtr initialPointCloud,
          const std::vector<cv::KeyPoint> keyPoints,
          const std::vector<std::vector<cv::Point> > outlines,
          const std::vector<std::vector<cv::Point2f> > rectangles,
          int inflationSize);

      /**
        @brief  Given the bounding box of a blob, inflate it.
        All the points that lie on the (edges of the) rectangle should
        also lie on exactly one plane for the blob to be a hole.
        @param[in] inImage [const cv::Mat] The input depth image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
        The original point cloud,  uninterpolated, undistorted.
        @param[in] keyPoints [const std::vector<cv::KeyPoint>] The keypoints of
        blobs
        @param[in] rectangles [const std::vector<std::vector<cv::point2f> >] The
        bounding boxes' vertices
        @param[in] inflationsize [cosnt int] grow the rectangle by inflationsize
        as to acquire more points to check for plane existence.
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesRectangleOutline(
          const cv::Mat inImage,
          const PointCloudXYZPtr initialPointCloud,
          const std::vector<cv::KeyPoint> keyPoints,
          const std::vector<std::vector<cv::Point2f> > rectangles,
          const int inflationSize);

      /**
        @brief Apply a cascade-like hole checker. Each filter applied is attached
        to an order which relates to the sequence of the overall filter execution.
        @param[in] interpolatedDepthImage [cv::Mat] The denoised depth image
        @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
        The undistorted input point cloud
        @param[in][out] conveyor [HolesConveyor&] A struct that contains the
        final valid holes
        @return void
       **/
      static void checkHoles(
          const cv::Mat& interpolatedDepthImage,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr initialPointCloud,
          HolesConveyor& conveyor);

      /**
        @brief Apply a cascade-like hole checker. Each filter applied is
        attached to an order which relates to the sequence of the overall filter
        execution.
        @param[in] method [const unsigned int] The filter identifier to execute
        @param[in] img [const cv::Mat&] The input depth image
        @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr] The
        original point cloud that corresponds to the input depth image
        @param[in][out] conveyor [HolesConveyor&] The structure that holds the
        final holes' data
        @param[in] inflationSize [const int] The amount of pixels by which each
        bounding box is inflated
        @param[in][out] imgs [std::vector<cv::Mat>&] A vector of images which
        shows the holes that are considered valid by each filter
        @param[in][out] msgs [std::vector<std::string>&] Debug messages
        @return void
       **/
      static void applyFilter(
          const unsigned int method,
          const cv::Mat& img,
          const PointCloudXYZPtr pointCloud,
          HolesConveyor& conveyor,
          const int inflationSize,
          std::vector<cv::Mat>& imgs,
          std::vector<std::string>& msgs);

      /**
        @brief Given a set of keypoints and an edges image, this function
        returns the valid keypoints and for each one, its respective, least
        area, rotated bounding box and the points of its outline.
        @param[in] keyPoints [const std::vector<cv::KeyPoint>]
        The original keypoints found.
        @param[in] denoisedDepthImageEdges [const cv::Mat] The original denoised
        depth edges image
        @param[in] detectionMethod [const int] The method by which the outline
        of a blob is obtained. 0 means by means of brushfire, 1 by means of
        raycasting
        @param[in][out] conveyor [HolesConveyor&] A struct that contains the
        final valid holes
        @return void
       **/
      static void validateBlobs(
          const std::vector<cv::KeyPoint> keyPoints,
          const cv::Mat denoisedDepthImageEdges,
          const int detectionMethod,
          HolesConveyor& conveyor);

      /**
        @brief This functions takes as input arguments a keypoints vector of
        size N, a rectangles vector of size M (the rectangle is represented
        by its 4 vertices so that the input can be either a Rectangle or a
        Rotated Rectangle) and a vector with the area of each rectangle
        with purpose to identify the least area rectangle in which a
        keypoint resides. It outputs a vector of keypoints (each keypoint must
        reside in at least one rectangle) and a vector of rectangles (again
        represented by its 4 vertices). There is a one-to-one association
        between the keypoints and the rectangles.
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>] The key points
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >]
        The rectangles found
        @param[in] inRectanglesArea [const std::vector<float>] The area of each
        rectangle
        @param[in] inContours [const std::vector<std::vector<cv::Point> >] The
        outline of each blob found
        @param[out] conveyor [HolesConveyor&] The container of vector of blobs'
        keypoints, outlines and areas
        @return void
       **/
      static void validateKeypointsToRectangles(
          const std::vector<cv::KeyPoint> inKeyPoints,
          const std::vector<std::vector<cv::Point2f> > inRectangles,
          const std::vector<float> inRectanglesArea,
          const std::vector<std::vector<cv::Point> > inContours,
          HolesConveyor& conveyor);
  };

}

#endif
