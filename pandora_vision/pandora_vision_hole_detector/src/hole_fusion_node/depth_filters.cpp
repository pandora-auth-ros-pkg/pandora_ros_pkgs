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

#include "hole_fusion_node/depth_filters.h"

namespace pandora_vision
{
  /**
    @brief Checks for valid holes just by the depth difference between
    the keypoint of the blob and the edges of its bounding box
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[out] msgs [std::vector<std::string>*] Messages for debug reasons
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @return void
   **/
  void DepthFilters::checkHolesDepthDiff(
    const cv::Mat& depthImage,
    const HolesConveyor& conveyor,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthDiff", "applyFilter");
    #endif

    for(unsigned int i = 0 ; i < rectanglesIndices.size() ; i++)
    {
      float mean = 0;

      for(unsigned int j = 0 ; j < 4; j++)
      {
        int x = rectanglesVector[i][j].x;
        int y = rectanglesVector[i][j].y;

        mean += depthImage.at<float>(y, x);
      }

      mean /= rectanglesVector[i].size();

      float value = depthImage.at<float>(
        conveyor.keyPoints[rectanglesIndices[i]].pt.y,
        conveyor.keyPoints[rectanglesIndices[i]].pt.x) - mean;

      // The gaussian mean
      float m = Parameters::HoleFusion::holes_gaussian_mean;

      // The gaussian standard deviation
      float s = Parameters::HoleFusion::holes_gaussian_stddev;

      // The gaussian probability of this hole being valid
      probabilitiesVector->at(rectanglesIndices[i]) =
        exp(-pow((value - m) / s, 2) / 2);

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthDiff");
    #endif
  }



  /**
    @brief Checks for valid holes by area / depth comparison
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points; each point is internal to its
    respective hole
    @param[out] msgs [std::vector<std::string>*] Messages for debug
    reasons
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @return void
   **/
  void DepthFilters::checkHolesDepthArea(
    const HolesConveyor& conveyor,
    const cv::Mat& depthImage,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthArea", "applyFilter");
    #endif

    for(unsigned int i = 0 ; i < conveyor.keyPoints.size() ; i++)
    {
      // The mean depth value of the points inside the i-th hole
      float mean = 0.0;

      for (std::set<unsigned int>::iterator it = holesMasksSetVector[i].begin();
        it != holesMasksSetVector[i].end(); it++)
      {
         int x = static_cast<int>(*it) % depthImage.cols;
         int y = static_cast<int>(*it) / depthImage.cols;

        mean += depthImage.at<float>(y, x);
      }

      // The number of points inside the i-th hole, or else, its area
      float area = holesMasksSetVector[i].size();

      mean /= area;

      // area = f(mean) for one circular hole
      float singleHoleDepthArea = 5.7028988154428989 * pow(10, 4)
        * exp(- 2.2113680011649128 * mean);

      // Upper-most curve, plus an increase in height
      // At most, one complete hole contains three circular ones
      float high = 3 * singleHoleDepthArea - area + 7000;

      // Lower-most curve minus a reduction in height
      // At least, one complete hole is one circular hole
      float low = singleHoleDepthArea - area - 7000;


      if(low < 0 && high > 0)
      {
        probabilitiesVector->at(i) = 1.0;
      }
      else
      {
        probabilitiesVector->at(i) = 0.0;
      }

      msgs->push_back(TOSTR(low) + std::string(" / ") + TOSTR(high));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthArea");
    #endif
  }



  /**
    @brief If the intermediate points (points between a hole's outline
    and its bounding rectangle) for each hole lie on one plane,
    this hole is considered valid. Although plane constitution alone is
    considered valid, the @param probabilitiesVector hint to the
    validity of the candidate hole through this filter
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud
    [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The point cloud acquired from the depth sensor, interpolated
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points;
    these points are the points between the hole's outline and its
    bounding rectangle
    @param[in] rectanglesIndices [const std::vector<int>&] Because
    each hole's bounding rectangle may be inflated, and thus not all holes
    possess a bounding rectangle by this process, in this vector is stored
    the indices of the holes whose inflated bounding box is inside the
    image's bounds
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @param[out] msgs [std::vector<std::string>*] Messages for
    debug reasons
    @return void
   **/
  void DepthFilters::checkHolesOutlineToRectanglePlaneConstitution(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesOutlineToRectanglePlaneConstitution",
      "applyFilter");
    #endif

    for (unsigned int i = 0; i < rectanglesIndices.size(); i++)
    {
      // From each set of intermediate points, construct the point cloud
      // that will be checked for plane constitution
      PointCloudXYZPtr intermediatePointsPointCloud (new PointCloudXYZ);

      intermediatePointsPointCloud->width = intermediatePointsSetVector[i].size();
      intermediatePointsPointCloud->height = 1;
      intermediatePointsPointCloud->points.resize
        (intermediatePointsPointCloud->width
         * intermediatePointsPointCloud->height);

      intermediatePointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      intermediatePointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator
        it = intermediatePointsSetVector[i].begin();
        it != intermediatePointsSetVector[i].end(); it++)
      {
        intermediatePointsPointCloud->points[pointCloudPointsIndex].x =
          initialPointCloud->points[*intermediatePointsSetVector[i].find(*it)].x;

        intermediatePointsPointCloud->points[pointCloudPointsIndex].y =
          initialPointCloud->points[*intermediatePointsSetVector[i].find(*it)].y;

        intermediatePointsPointCloud->points[pointCloudPointsIndex].z =
          initialPointCloud->points[*intermediatePointsSetVector[i].find(*it)].z;

        pointCloudPointsIndex++;
      }

      // Check if the intermediatePointsPointCloud's points are on a plane
      std::vector<pcl::PointIndices::Ptr> inliersVector;
      int numPlanes = PlanesDetection::locatePlanes(
        intermediatePointsPointCloud, false, &inliersVector);

      // The probability (in all probability) of the current hole lying on
      // one plane will be the ratio of the number of intermediate points
      // that lie on one plane over all the intermediate points
      // (max points on one plane) / (all intermediate points)
      int maxPoints = 0;
      for (unsigned int iv = 0; iv < inliersVector.size(); iv++)
      {
        if (inliersVector[iv]->indices.size() > maxPoints)
        {
          maxPoints = inliersVector[iv]->indices.size();
        }
      }

      probabilitiesVector->at(rectanglesIndices[i]) =
        static_cast<float> (maxPoints) / intermediatePointsSetVector[i].size();

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesOutlineToRectanglePlaneConstitution");
    #endif
  }



  /**
    @brief All the points that lie on the (edges of the) rectangle should
    also lie on exactly one plane for the blob to be a hole. Although all
    planes are considered valid, the @param probabilitiesVector hint
    to the validity of the candidate hole through this filter
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud
    [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The point cloud acquired from the depth sensor, interpolated
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @param[out] msgs [std::vector<std::string>*] Messages for
    debug reasons
    @return void
   **/
  void DepthFilters::checkHolesRectangleEdgesPlaneConstitution(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesRectangleEdgesPlaneConstitution", "applyFilter");
    #endif


    // For each inflated rectangle, store in visitedPoints
    // the points that constitute the rectangle.
    // We will test if these points all lie on one plane.
    for (unsigned int i = 0; i < rectanglesVector.size(); i++)
    {
      // The canvas image will hold the rectangles.
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));


      // Draw the rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(canvas, rectanglesVector[i][j],
          rectanglesVector[i][(j + 1) % 4], color, 1, 8);
      }

      std::set<unsigned int> visitedPoints;
      for (unsigned int rows = 0; rows < inImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < inImage.cols; cols++)
        {
          if (canvas.data[rows * inImage.cols + cols] != 0)
          {
            visitedPoints.insert(rows * inImage.cols + cols);
          }
        }
      }

      // From each set of points lying on the edges of the rectangle,
      // construct the point cloud that will be checked for plane constitution
      PointCloudXYZPtr edgePointsPointCloud (new PointCloudXYZ);

      edgePointsPointCloud->width = visitedPoints.size();
      edgePointsPointCloud->height = 1;
      edgePointsPointCloud->points.resize
        (edgePointsPointCloud->width * edgePointsPointCloud->height);

      edgePointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      edgePointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        edgePointsPointCloud->points[pointCloudPointsIndex].x =
          initialPointCloud->points[*visitedPoints.find(*it)].x;

        edgePointsPointCloud->points[pointCloudPointsIndex].y =
          initialPointCloud->points[*visitedPoints.find(*it)].y;

        edgePointsPointCloud->points[pointCloudPointsIndex].z =
          initialPointCloud->points[*visitedPoints.find(*it)].z;

        pointCloudPointsIndex++;
      }

      // Check if the edgePointsPointCloud points are on a plane
      std::vector<pcl::PointIndices::Ptr> inliersVector;
      int numPlanes = PlanesDetection::locatePlanes(edgePointsPointCloud,
        false, &inliersVector);

      // The probability (in all probability) of an inflated rectangle
      // residing on one plane will be the ratio of
      // (max points on one plane) / (all inflated rectangle points)
      int maxPoints = 0;
      for (unsigned int iv = 0; iv < inliersVector.size(); iv++)
      {
        if (inliersVector[iv]->indices.size() > maxPoints)
        {
          maxPoints = inliersVector[iv]->indices.size();
        }
      }

      probabilitiesVector->at(rectanglesIndices[i]) =
        static_cast<float> (maxPoints) / visitedPoints.size();

      msgs->push_back(TOSTR(probabilitiesVector->at(rectanglesIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesRectangleEdgesPlaneConstitution");
    #endif
  }



  /**
    @brief Checks the homogeneity of the gradient of an interpolated
    depth image in areas denoted by the points inside the
    holesMasksSetVector vector
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] interpolatedDepthImage [const cv::Mat&] The input
    interpolated depth image
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points; each point is internal to its
    respective hole
    @param[out] msgs [std::vector<std::string>*] Debug messages
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @return void
   **/
  void DepthFilters::checkHolesDepthHomogeneity(
    const HolesConveyor& conveyor,
    const cv::Mat& interpolatedDepthImage,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthHomogeneity", "applyFilter");
    #endif

    // Facilitate the edge detection by converting the 32FC1 image
    // values to a range of 0-255
    cv::Mat visualizableDenoisedImage;
    visualizableDenoisedImage = Visualization::scaleImageForVisualization
      (interpolatedDepthImage, Parameters::Image::scale_method);

    // from now onwards every image is in the range of 0-255
    cv::Mat interpolatedDepthImageEdges;
    EdgeDetection::applySobel(visualizableDenoisedImage,
      &interpolatedDepthImageEdges);

    // Threshold the interpolatedDepthImageEdges image
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      Parameters::Depth::denoised_edges_threshold, 255, 3);

    // make all non zero pixels have a value of 255
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      0, 255, 0);

    unsigned char* ptr = interpolatedDepthImageEdges.ptr();

    for (unsigned int o = 0; o < conveyor.outlines.size(); o++)
    {
      int numBlacks = 0;
      int numWhites = 0;

      for (std::set<unsigned int>::iterator it = holesMasksSetVector[o].begin();
        it != holesMasksSetVector[o].end(); it++)
      {
        if (ptr[*it] != 0)
        {
          numWhites++;
        }
        else
        {
          numBlacks++;
        }
      }

      probabilitiesVector->at(o) =
        static_cast<float>(numWhites) / (numWhites + numBlacks);

      msgs->push_back(TOSTR(probabilitiesVector->at(o)));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthHomogeneity");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is
    attached to an order which relates to the sequence of the overall
    filter execution.
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] interpolatedDepthImage [const cv::Mat&] The denoised
    depth image
    @param[in] interpolatedPointCloud
    [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
    The interpolated input point cloud
    @param[in] holesMasksSetVector
    [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points's indices;
    each point is internal to its respective hole
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds
    for each hole a set of points;
    these points are the points between the hole's outline and its
    bounding rectangle
    @param[out] probabilitiesVector [std::vector<std::vector<float> >*]
    A 2D vector of probabilities hinting to the certainty degree with
    which each candidate hole is associated for every
    active filter executed.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of active filters by
    the number of keypoints found and published by the rgb node.
    @return void
   **/
  void DepthFilters::checkHoles(
    const HolesConveyor& conveyor,
    const cv::Mat& interpolatedDepthImage,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& interpolatedPointCloud,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    std::vector<std::vector<float> >* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles", "sift");
    #endif

    std::map<int, int> filtersOrder;

    if (Parameters::HoleFusion::run_checker_depth_diff > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_depth_diff] = 1;
    }
    if (Parameters::HoleFusion::run_checker_outline_of_rectangle > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_outline_of_rectangle] = 2;
    }
    if (Parameters::HoleFusion::run_checker_depth_area > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_depth_area] = 3;
    }
    if (Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle] = 4;
    }
    if (Parameters::HoleFusion::run_checker_depth_homogeneity > 0)
    {
      filtersOrder[Parameters::HoleFusion::run_checker_depth_homogeneity] = 5;
    }

    std::vector<cv::Mat> imgs;
    std::vector<std::string> msgs;

    int counter = 0;
    for (std::map<int, int>::iterator o_it = filtersOrder.begin();
      o_it != filtersOrder.end(); ++o_it)
    {
      applyFilter(
        o_it->second,
        interpolatedDepthImage,
        interpolatedPointCloud,
        conveyor,
        holesMasksSetVector,
        rectanglesVector,
        rectanglesIndices,
        intermediatePointsSetVector,
        &probabilitiesVector->at(counter),
        &imgs,
        &msgs);

      counter++;
    } // o_it iterator ends

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_check_holes) // Debug
    {
      Visualization::multipleShow("depth checkHoles functions",
        imgs, msgs, 1200, 1);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("checkHoles");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is
    attached to an order which relates to the sequence of the
    overall filter execution.
    @param[in] method [const int&] The filter identifier to execute
    @param[in] img [const cv::Mat&] The input depth image
    @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&] The
    original point cloud that corresponds to the input depth image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] holesMasksSetVector [const std::vector<std::set<unsigned int> >&]
    A vector that holds sets of points; each point is internal to its
    respective hole
    @param[in] rectanglesVector
    [const std::vector<std::vector<cv::Point2f> >&] A vector that holds
    the vertices of each rectangle that corresponds to a specific hole
    inside the coveyor
    @param[in] rectanglesIndices [const std::vector<int>&] A vector that
    is used to identify a hole's corresponding rectangle. Used primarily
    because the rectangles used are inflated rectangles; not all holes
    possess an inflated rectangle
    @param[in] intermediatePointsSetVector
    [const std::vector<std::set<unsigned int> >& ] A vector that holds for each
    hole a set of points; these points are the points between the hole's
    outline and its bounding rectangle
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree with which each
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node.
    @param[out] imgs [std::vector<cv::Mat>*] A vector of images which
    shows the holes that are considered valid by each filter
    @param[out] msgs [std::vector<std::string>*] Debug messages
    @return void
   **/
  void DepthFilters::applyFilter(
    const int& method,
    const cv::Mat& img,
    const PointCloudXYZPtr& pointCloud,
    const HolesConveyor& conveyor,
    const std::vector<std::set<unsigned int> >& holesMasksSetVector,
    const std::vector<std::vector<cv::Point2f> >& rectanglesVector,
    const std::vector<int>& rectanglesIndices,
    const std::vector<std::set<unsigned int> >& intermediatePointsSetVector,
    std::vector<float>* probabilitiesVector,
    std::vector<cv::Mat>* imgs,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("applyFilter", "checkHoles");
    #endif

    std::string windowMsg;
    std::vector<std::string> finalMsgs;
    std::vector<std::string> msgs_;

    // Initialize structures
    finalMsgs.clear();
    msgs_.clear();

    switch(method)
    {
      // Filter #1 (through difference of depth)------------------------------
      case 1 :
        {
          checkHolesDepthDiff(
            img,
            conveyor,
            rectanglesVector,
            rectanglesIndices,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Depth difference";
          break;
        }
        // Filter #2----------------------------------------------------------
        // Inflate the bounding boxes by an inflation size.
        // For a blob to be at least a potential hole, all the points that
        // constitute the inflated rectangle should lie on exactly one plane.
      case 2 :
        {
          checkHolesRectangleEdgesPlaneConstitution(
            img,
            pointCloud,
            rectanglesVector,
            rectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Outline of rectangle on plane";
          break;
        }
        // Filter #3 (depth & area comparison)--------------------------------
      case 3 :
        {
          checkHolesDepthArea(
            conveyor,
            img,
            holesMasksSetVector,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Area / Depth";
          break;
        }
        // Filter #4----------------------------------------------------------
        // Brushfire from blob outline to blob bounding box
        // with an inflation size (inflates the rectangle by x pixels).
        // If the points between the blob's outline and the inflated rectangle
        // lie on one plane, this blob is a hole.
      case 4 :
        {
          checkHolesOutlineToRectanglePlaneConstitution(
            img,
            pointCloud,
            intermediatePointsSetVector,
            rectanglesIndices,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Points around blob to plane";
          break;
        }
        // Filter #5 (Depth homogenity)---------------------------------------
        // All holes are considered valid except for those that are edgeless
        // inside the area denoted by the conveyor->outlines points
      case 5 :
        {
          checkHolesDepthHomogeneity(
            conveyor,
            img,
            holesMasksSetVector,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Depth homogenity";
          break;
        }
    }

    for(int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      if(msgs_.size() == conveyor.keyPoints.size())
      {
        finalMsgs.push_back(msgs_[i]);
      }
    }

    #ifdef DEBUG_SHOW
    if(Parameters::Debug::show_check_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" ") + windowMsg;
      msgs->push_back(msg);

      cv::Mat tmp;
      tmp = Visualization::showHoles(
        windowMsg.c_str(),
        img,
        conveyor,
        -1,
        finalMsgs);

      imgs->push_back(tmp);
    }
    #endif
    #ifdef DEBUG_TIME
    Timer::tick("applyFilter");
    #endif
  }

} // namespace pandora_vision
