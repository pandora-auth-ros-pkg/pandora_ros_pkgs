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

#include "hole_fusion_node/depth_filters.h"

namespace pandora_vision
{
  /**
    @brief Checks for valid holes just by depth difference between the center
    of the blob and the edges of the bounding box
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @param[in] inflationSize [const in&t] The number of pixels by which the
    bounding rectange will be inflated
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
    std::vector<std::string>* msgs,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthDiff", "applyFilter");
    #endif


    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;

    std::vector<cv::KeyPoint> validKeyPoints;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        // check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < depthImage.cols &&
          vert_x - inflationSize * cos(theta) > 0 &&
          vert_y - inflationSize * sin(theta) < depthImage.rows &&
          vert_y - inflationSize * sin(theta) > 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< if one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        validKeyPoints.push_back(conveyor.keyPoints[i]);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles


    for(unsigned int i = 0 ; i < validKeyPointsIndices.size() ; i++)
    {
      float mean = 0;
      for(unsigned int j = 0 ; j < 4; j++)
      {
        int x = inflatedRectangles[i][j].x;
        int y = inflatedRectangles[i][j].y;
        if(x < 0 || x >= depthImage.cols || y < 0 || y >= depthImage.rows)
        {
          mean = -40000.0;  //!< Invalid value
          break;
        }
        mean += depthImage.at<float>(y, x);
      }
      mean /= 4;

      float value = depthImage.at<float>(
        conveyor.keyPoints[validKeyPointsIndices[i]].pt.y,
        conveyor.keyPoints[validKeyPointsIndices[i]].pt.x) - mean;

      if(value > 0 && value < Parameters::depth_difference)
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) = 1.0;
      }
      else
      {
        probabilitiesVector->at(validKeyPointsIndices[i]) = 0.0;
      }

      msgs->push_back(TOSTR(value));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthDiff");
    #endif
  }



  /**
    @brief Checks for valid holes by area / depth comparison
    @param[in] depthImage [const cv::Mat&] The depth image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @return void
   **/
  void DepthFilters::checkHolesDepthArea(
    const cv::Mat& depthImage,
    const HolesConveyor& conveyor,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthArea", "applyFilter");
    #endif

    for(unsigned int i = 0 ; i < conveyor.keyPoints.size() ; i++)
    {
      float mean = 0;
      float area = sqrt(
        pow(conveyor.rectangles[i][0].x - conveyor.rectangles[i][1].x, 2) +
        pow(conveyor.rectangles[i][0].y - conveyor.rectangles[i][1].y, 2));

      area *= sqrt(
        pow(conveyor.rectangles[i][1].x - conveyor.rectangles[i][2].x, 2) +
        pow(conveyor.rectangles[i][1].y - conveyor.rectangles[i][2].y, 2));


      for(unsigned int j = 0; j < conveyor.rectangles[i].size(); j++)
      {
        int x = conveyor.rectangles[i][j].x;
        int y = conveyor.rectangles[i][j].y;
        mean += depthImage.at<float>(y, x);
      }
      mean /= conveyor.rectangles[i].size();

      //!< Cubic fitting  --- Small hole
      //!< Normal : -18480  * x^3+87260.3 * x^2-136846 * x+74094
      float vlow = -18480.0 * pow(mean, 3) + 87260.3 * pow(mean, 2) -
        136846 * mean + 68500.0 - area;
      //!< Exponential fitting --- Small hole
      //!< Normal : -23279.4  * x^3+112218 * x^2-182162 * x+105500
      float vhigh = -23279.4 * pow(mean, 3) + 112218.0 * pow(mean, 2) -
        182162.0 * mean + 112500 - area;

      if(vlow < 0 && vhigh > 0)
      {
        probabilitiesVector->at(i) = 1.0;
      }
      else
      {
        probabilitiesVector->at(i) = 0.0;
      }

      msgs->push_back(TOSTR(vlow) + std::string(",") + TOSTR(vhigh));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthArea");
    #endif
  }



  /**
    @brief Brushfire from a blobs's outline to its bounding box
    with an inflation size (inflates the rectangle by inflationSize pixels).
    If the points between the blob's outline and the inflated rectangle
    lie on one plane, this blob is a hole. Although all
    planes are considered valid, the @param probabilitiesVector hint
    to the validity of the candidate hole through this filter
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The original point cloud acquired from the depth sensor
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inflationsize [const int&] Grow the rectangle by inflationsize as
    to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void DepthFilters::checkHolesBrushfireOutlineToRectangle(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesBrushfireOutlineToRectangle", "applyFilter");
    #endif

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;

    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;
    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    //!< for every valid inflated rectangle store a point
    //!< from which the brushfire will begin
    std::vector<cv::Point> brushfireBeginPoints;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        // check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) > 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) > 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } // end for rectangle's points

      //!< if inflatedVerticesWithinImageLimits < 4 in the end,
      //!< we won't need this point since the inflated rectangle
      //!< will be discarded.
      cv::Point potentialBrushfireBeginPoint(
        round(vert_x - inflationSize / 2 * cos(theta)),
        round(vert_y - inflationSize / 2 * sin(theta)));

      //!< if one or more vertices are out of bounds
      //!< discard the whole inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        inflatedRectangles.push_back(inflatedVertices);
        brushfireBeginPoints.push_back(potentialBrushfireBeginPoint);
      }
    } //!< end for rectangles


    /**
     * For each inflated rectangle, store in visitedPoints
     * the visited points of brushfire.
     * (visitedPoints = points between the blob's outline
     * and the inflated rectangle)
     * We will test if these points all lie on one plane.
     **/
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {

      //!< The canvas image will hold the blobs' outlines, and their rectangles.
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

      //!< Draw the outline of the i-th valid blob
      for(unsigned int j = 0; j < conveyor.outlines[i].size(); j++)
      {
        canvas.at<uchar>(
          conveyor.outlines[i][j].y, conveyor.outlines[i][j].x) = 255;
      }

      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(canvas, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
      }

      std::set<unsigned int> visitedPoints;
      BlobDetection::brushfirePoint(brushfireBeginPoints[i],
        &canvas, &visitedPoints);

      /**
       * In order to construct a new point cloud
       * we need a priori knowledge of the valid points
       * of the input point cloud.
       **/
      int numValidVisitedPoints = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          numValidVisitedPoints++;
        }
      }

      PointCloudXYZPtr visitedPointsPointCloud (new PointCloudXYZ);

      visitedPointsPointCloud->width = numValidVisitedPoints;
      visitedPointsPointCloud->height = 1;
      visitedPointsPointCloud->points.resize
        (visitedPointsPointCloud->width * visitedPointsPointCloud->height);

      visitedPointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      visitedPointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          visitedPointsPointCloud->points[pointCloudPointsIndex].x =
            initialPointCloud->points[*visitedPoints.find(*it)].x;

          visitedPointsPointCloud->points[pointCloudPointsIndex].y =
            initialPointCloud->points[*visitedPoints.find(*it)].y;

          visitedPointsPointCloud->points[pointCloudPointsIndex].z =
            initialPointCloud->points[*visitedPoints.find(*it)].z;

          pointCloudPointsIndex++;
        }
      }

      //!< Check if the visitedPointsPointCloud points are on a plane
      std::vector<pcl::PointIndices::Ptr> inliersVector;
      int numPlanes = PlanesDetection::locatePlanes(visitedPointsPointCloud,
        false, &inliersVector);

      //!< The probability (in all probability) of an inflated rectangle
      //!< residing on one plane will be the ratio of
      //!< (max points on one plane) / (all inflated rectangle points)
      int maxPoints = 0;
      for (unsigned int iv = 0; iv < inliersVector.size(); iv++)
      {
        if (inliersVector[iv]->indices.size() > maxPoints)
        {
          maxPoints = inliersVector[iv]->indices.size();
        }
      }

      probabilitiesVector->at(validKeyPointsIndices[i]) =
        static_cast<float> (maxPoints) / pointCloudPointsIndex;

      msgs->push_back(TOSTR(probabilitiesVector->at(validKeyPointsIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesBrushfireOutlineToRectangle");
    #endif
  }



  /**
    @brief  Given the bounding box of a blob, inflate it.
    All the points that lie on the (edges of the) rectangle should
    also lie on exactly one plane for the blob to be a hole. Although all
    planes are considered valid, the @param probabilitiesVector hint
    to the validity of the candidate hole through this filter
    @param[in] inImage [const cv::Mat&] The input depth image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
    The original point cloud,  uninterpolated, undistorted.
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[in] inflationSize [cosnt int&] grow the rectangle by inflationSize
    as to acquire more points to check for plane existence.
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @param[in][out] msgs [std::vector<std::string>*] Messages for debug reasons
    @return void
   **/
  void DepthFilters::checkHolesRectangleOutline(
    const cv::Mat& inImage,
    const PointCloudXYZPtr& initialPointCloud,
    const HolesConveyor& conveyor,
    const int& inflationSize,
    std::vector<float>* probabilitiesVector,
    std::vector<std::string>* msgs)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesRectangleOutline", "applyFilter");
    #endif

    //!< Since not all rectangles may make it, store the indices
    //!< of the original keypoints that correspond to valid inflated rectangles
    //!< in the validKeyPointsIndices vector
    std::vector<unsigned int> validKeyPointsIndices;

    //!< Store the vertices of the inside-of-image-bounds inflated bounding
    //!< rectangles in the inflatedRectangles vector
    std::vector<std::vector<cv::Point> > inflatedRectangles;

    float key_y;
    float key_x;
    float vert_y;
    float vert_x;
    double theta;
    double keypointVertDist;

    for (unsigned int i = 0; i < conveyor.rectangles.size(); i++)
    {
      std::vector<cv::Point> inflatedVertices;
      int inflatedVerticesWithinImageLimits = 0;

      for (int j = 0; j < 4; j++)
      {
        key_y = conveyor.keyPoints[i].pt.y;
        key_x = conveyor.keyPoints[i].pt.x;

        vert_y = conveyor.rectangles[i][j].y;
        vert_x = conveyor.rectangles[i][j].x;

        theta = atan2(key_y - vert_y, key_x - vert_x);

        keypointVertDist = sqrt(pow(key_x -vert_x, 2) + pow(key_y -vert_x, 2));

        //!< check if the inflated vertex has gone out of bounds
        if (vert_x - inflationSize * cos(theta) < inImage.cols &&
          vert_x - inflationSize * cos(theta) >= 0 &&
          vert_y - inflationSize * sin(theta) < inImage.rows &&
          vert_y - inflationSize * sin(theta) >= 0)
        {
          inflatedVerticesWithinImageLimits++;
        }

        inflatedVertices.push_back(
          cv::Point(round(vert_x - inflationSize * cos(theta)),
            round(vert_y - inflationSize * sin(theta))));
      } //!< end for rectangle's points

      //!< If one or more vertices are out of bounds discard the whole
      //!< inflated rectangle
      if (inflatedVerticesWithinImageLimits < 4)
      {
        inflatedVertices.clear();
        continue;
      }
      else
      {
        validKeyPointsIndices.push_back(i);
        inflatedRectangles.push_back(inflatedVertices);
      }
    } //!< end for rectangles

    /**
     * For each inflated rectangle, store in visitedPoints
     * the points that constitute the rectangle.
     * We will test if these points all lie on one plane.
     **/
    for (unsigned int i = 0; i < inflatedRectangles.size(); i++)
    {
      //!< The canvas image will hold the blobs' outlines, and their rectangles.
      cv::Mat canvas = cv::Mat::zeros(inImage.size(), CV_8UC1);
      cv::RNG rng(12345);
      cv::Scalar color = cv::Scalar(
        rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));


      //!< Draw the inflated rectangle that corresponds to it
      for(int j = 0; j < 4; j++)
      {
        cv::line(canvas, inflatedRectangles[i][j],
          inflatedRectangles[i][(j + 1) % 4], color, 1, 8);
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


      /**
       * In order to construct a new point cloud
       * we need a priori knowledge of the valid points
       * of the input point cloud.
       * */
      int numValidVisitedPoints = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< If the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          numValidVisitedPoints++;
        }
      }

      PointCloudXYZPtr visitedPointsPointCloud (new PointCloudXYZ);

      visitedPointsPointCloud->width = numValidVisitedPoints;
      visitedPointsPointCloud->height = 1;
      visitedPointsPointCloud->points.resize
        (visitedPointsPointCloud->width * visitedPointsPointCloud->height);

      visitedPointsPointCloud->header.frame_id =
        initialPointCloud->header.frame_id;
      visitedPointsPointCloud->header.stamp =
        initialPointCloud->header.stamp;

      int pointCloudPointsIndex = 0;
      for(std::set<unsigned int>::iterator it = visitedPoints.begin();
        it != visitedPoints.end(); it++)
      {
        //!< if the point in the original point cloud is not missing (noise)
        if (!(initialPointCloud->points[*visitedPoints.find(*it)].x !=
            initialPointCloud->points[*visitedPoints.find(*it)].x))
        {
          visitedPointsPointCloud->points[pointCloudPointsIndex].x =
            initialPointCloud->points[*visitedPoints.find(*it)].x;

          visitedPointsPointCloud->points[pointCloudPointsIndex].y =
            initialPointCloud->points[*visitedPoints.find(*it)].y;

          visitedPointsPointCloud->points[pointCloudPointsIndex].z =
            initialPointCloud->points[*visitedPoints.find(*it)].z;

          pointCloudPointsIndex++;
        }
      }

      //!< Check if the visitedPointsPointCloud points are on a plane
      std::vector<pcl::PointIndices::Ptr> inliersVector;
      int numPlanes = PlanesDetection::locatePlanes(visitedPointsPointCloud,
        false, &inliersVector);

      //!< The probability (in all probability) of an inflated rectangle
      //!< residing on one plane will be the ratio of
      //!< (max points on one plane) / (all inflated rectangle points)
      int maxPoints = 0;
      for (unsigned int iv = 0; iv < inliersVector.size(); iv++)
      {
        if (inliersVector[iv]->indices.size() > maxPoints)
        {
          maxPoints = inliersVector[iv]->indices.size();
        }
      }

      probabilitiesVector->at(validKeyPointsIndices[i]) =
        static_cast<float> (maxPoints) / pointCloudPointsIndex;

      msgs->push_back(TOSTR(probabilitiesVector->at(validKeyPointsIndices[i])));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesRectangleOutline");
    #endif
  }



  /**
    @brief Checks the homogenity of the gradient of depth in an area
    enclosed by @param inOutlines
    @param[in] interpolatedDepthImage [const cv::Mat&] The input depth image
    @param[in] conveyor [const HolesConveyor&] The candidate holes
    @param[out] msgs [std::vector<std::string>*] Debug messages
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities, each position of which hints to the certainty degree
    with which the associated candidate hole is associated.
    While the returned set may be reduced in size, the size of this vector
    is the same throughout and equal to the number of keypoints found and
    published by the rgb node
    @return void
   **/
  void DepthFilters::checkHolesDepthHomogenity(
    const cv::Mat& interpolatedDepthImage,
    const HolesConveyor& conveyor,
    std::vector<std::string>* msgs,
    std::vector<float>* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHolesDepthHomogenity", "applyFilter");
    #endif

    //!< Facilitate the edge detection by converting the 32FC1 image
    //!< values to a range of 0-255
    cv::Mat visualizableDenoisedImage;
    visualizableDenoisedImage = Visualization::scaleImageForVisualization
      (interpolatedDepthImage, Parameters::scale_method);

    //!< from now onwards every image is in the range of 0-255
    cv::Mat interpolatedDepthImageEdges;
    EdgeDetection::applySobel(visualizableDenoisedImage,
      &interpolatedDepthImageEdges);

    //!< Threshold the interpolatedDepthImageEdges image
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      Parameters::threshold_lower_value, 255, 3);

    //!< make all non zero pixels have a value of 255
    cv::threshold(interpolatedDepthImageEdges, interpolatedDepthImageEdges,
      0, 255, 0);

    for (unsigned int o = 0; o < conveyor.outlines.size(); o++)
    {
      int numBlacks = 0;
      int numWhites = 0;
      for (unsigned int rows = 0; rows < interpolatedDepthImage.rows; rows++)
      {
        for (unsigned int cols = 0; cols < interpolatedDepthImage.cols; cols++)
        {
          //!< Check whether the current point resides inside the given outline
          if (cv::pointPolygonTest(
              conveyor.outlines[o], cv::Point(cols, rows), false) > 0)
          {
            if (interpolatedDepthImageEdges.at<unsigned char>(rows, cols) != 0)
            {
              numWhites++;
            }
            else
            {
              numBlacks++;
            }
          }
        }
      }

      probabilitiesVector->at(o) =
        static_cast<float>(numWhites) / (numWhites + numBlacks);

      msgs->push_back(TOSTR(probabilitiesVector->at(o)));
    }

    #ifdef DEBUG_TIME
    Timer::tick("checkHolesDepthHomogenity");
    #endif
  }



  /**
    @brief Apply a cascade-like hole checker. Each filter applied is
    attached to an order which relates to the sequence of the overall
    filter execution.
    @param[in] interpolatedDepthImage [const cv::Mat&] The denoised depth
    image
    @param[in] initialPointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr]
    The undistorted input point cloud
    @param[in] conveyor [const HolesConveyor&] A struct that
    contains the final valid holes
    @param[out] probabilitiesVector [std::vector<std::vector<float> >*]
    A 2D vector of probabilities hinting to the certainty degree with
    which each candidate hole is associated for every
    active filter executed.
    While the returned set may be reduced in size,
    the size of this vector is the same throughout and equal to the number
    of active filters by the number of keypoints found and
    published by the rgb node.
    @return void
   **/
  void DepthFilters::checkHoles(
    const cv::Mat& interpolatedDepthImage,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& initialPointCloud,
    const HolesConveyor& conveyor,
    std::vector<std::vector<float> >* probabilitiesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("checkHoles", "sift");
    #endif

    std::map<int, int> filtersOrder;

    if (Parameters::run_checker_depth_diff > 0)
    {
      filtersOrder[Parameters::run_checker_depth_diff] = 1;
    }
    if (Parameters::run_checker_outline_of_rectangle > 0)
    {
      filtersOrder[Parameters::run_checker_outline_of_rectangle] = 2;
    }
    if (Parameters::run_checker_depth_area > 0)
    {
      filtersOrder[Parameters::run_checker_depth_area] = 3;
    }
    if (Parameters::run_checker_brushfire_outline_to_rectangle > 0)
    {
      filtersOrder[Parameters::
        run_checker_brushfire_outline_to_rectangle] = 4;
    }
    if (Parameters::run_checker_depth_homogenity > 0)
    {
      filtersOrder[Parameters::run_checker_depth_homogenity] = 5;
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
        initialPointCloud,
        conveyor,
        Parameters::rectangle_inflation_size,
        &probabilitiesVector->at(counter),
        &imgs,
        &msgs);

      counter++;
    } //!< o_it iterator ends

    #ifdef DEBUG_SHOW
    if(Parameters::debug_show_check_holes) // Debug
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
    @brief Apply a cascade-like hole checker. Each filter applied is attached
    to an order which relates to the sequence of the overall filter execution.
    @param[in] method [const unsigned int&] The filter identifier to execute
    @param[in] img [const cv::Mat&] The input depth image
    @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&] The
    original point cloud that corresponds to the input depth image
    @param[in] conveyor [const HolesConveyor&] The structure that
    holds the final holes' data
    @param[in] inflationSize [const int&] The amount of pixels by which each
    bounding box is inflated
    @param[out] probabilitiesVector [std::vector<float>*] A vector
    of probabilities hinting to the certainty degree with which each
    candidate hole is associated. While the returned set may be reduced in
    size, the size of this vector is the same throughout and equal to the
    number of keypoints found and published by the rgb node.
    @param[in][out] imgs [std::vector<cv::Mat>*] A vector of images which shows
    the holes that are considered valid by each filter
    @param[in][out] msgs [std::vector<std::string>*] Debug messages
    @return void
   **/
  void DepthFilters::applyFilter(
    const unsigned int& method,
    const cv::Mat& img,
    const PointCloudXYZPtr& pointCloud,
    const HolesConveyor& conveyor,
    const int& inflationSize,
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

    //!< Initialize structures
    finalMsgs.clear();
    msgs_.clear();

    switch(method)
    {
      //!< Filter #1 (through difference of depth)------------------------------
      case 1 :
        {
          checkHolesDepthDiff(
            img,
            conveyor,
            &msgs_,
            inflationSize,
            probabilitiesVector);

          windowMsg = "Filter: Depth difference";
          break;
        }
        //!< Filter #2----------------------------------------------------------
        //!< Inflate the bounding boxes by an inflation size.
        //!< For a blob to be at least a potential hole, all the points that
        //!< constitute the inflated rectangle should lie on exactly one plane.
      case 2 :
        {
          checkHolesRectangleOutline(
            img,
            pointCloud,
            conveyor,
            inflationSize,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Outline of rectangle on plane";
          break;
        }
        //!< Filter #3 (depth & area comparison)--------------------------------
      case 3 :
        {
          checkHolesDepthArea(
            img,
            conveyor,
            &msgs_,
            probabilitiesVector);

          windowMsg = "Filter: Area / Depth";
          break;
        }
        //!< Filter #4----------------------------------------------------------
        //!< Brushfire from blob outline to blob bounding box
        //!< with an inflation size (inflates the rectangle by x pixels).
        //!< If the points between the blob's outline and the inflated rectangle
        //!< lie on one plane, this blob is a hole.
      case 4 :
        {
          checkHolesBrushfireOutlineToRectangle(
            img,
            pointCloud,
            conveyor,
            inflationSize,
            probabilitiesVector,
            &msgs_);

          windowMsg = "Filter: Points around blob to plane";
          break;
        }
        //!< Filter #5 (Depth homogenity)---------------------------------------
        //!< All holes are considered valid except for those that are edgeless
        //!< inside the area denoted by the conveyor->outlines points
      case 5 :
        {
          checkHolesDepthHomogenity(
            img,
            conveyor,
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
    if(Parameters::debug_show_check_holes) // Debug
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

