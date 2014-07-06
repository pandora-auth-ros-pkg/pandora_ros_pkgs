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

#include "utils/blob_detection.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
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
  void BlobDetection::brushfireKeypoint(
    const cv::KeyPoint& inKeyPoint,
    cv::Mat* edgesImage,
    std::vector<cv::Point2f>* blobOutlineVector,
    float* blobArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfireKeypoint", "validateBlobs");
    #endif

    // Get a pointer on the edges image
    unsigned char* ptr = edgesImage->ptr();

    // The sets needed by the brushfire implementation
    std::set<unsigned int> current, next, visited;

    // The indices of the blob's outline points stored in a set.
    // If the blobOutlineVector was used, in it there would be redundant copies
    // of each outline point
    std::set<unsigned int> blobOutlineSet;

    current.insert(
      static_cast<int>(round(inKeyPoint.pt.y) * edgesImage->cols)
      + static_cast<int>(round(inKeyPoint.pt.x)));

    visited.insert(
      static_cast<int>(round(inKeyPoint.pt.y) * edgesImage->cols)
      + static_cast<int>(round(inKeyPoint.pt.x)));

    while (current.size() != 0)
    {
      for (std::set<unsigned int>::iterator it = current.begin();
        it != current.end(); it++)
      {
        // Sweep the neighbors of the current point
        for (int m = -1; m < 2; m++)
        {
          for (int n = -1; n < 2; n++)
          {
            // This check is needed because it is possible to access
            // a point diagonal to the current one
            // that goes out of the image's borders (value == 0),
            // while it shouldn't.
            // E.g. this happens when there are "cracks" to the border of a
            // 1-pixel border
            if (abs(m) + abs(n) < 2)
            {
              // Column-wise coordinate
              int x = static_cast<int>(*it) % edgesImage->cols + m;

              // Row-wise coordinate
              int y = static_cast<int>(*it) / edgesImage->cols + n;

              // The index of this neighbor inside image edgesImage
              int ind = y * edgesImage->cols + x;

              // If this neighbor is not within the image's borders,
              // discard it and move on
              if (x < 0 || y < 0 ||
                x > edgesImage->cols - 1 || y > edgesImage->rows - 1)
              {
                continue;
              }

              // The value of the point with index ind in edgesImage
              char v = ptr[ind];

              // If this neighbor is a blank pixel and
              // it has not been visited, put it in set "next"
              if ((v == 0) && visited.find(ind) == visited.end())
              {
                next.insert(ind);
              }

              // If this neighbor has a non-zero value, it is an outline point
              if (v != 0)
              {
                blobOutlineSet.insert(ind);
              }

              // Whatever the condition, this neighbor has been visited
              visited.insert(ind);
            }
          }
        }
      }

      // The set of all future points becomes the one with which the
      // brushfire will begin
      current.swap(next);

      // Clear set "next" for the next iteration
      next.clear();
    }

    // Fill the blobOutlineVector with the
    // transformed content of the blobOutlineSet
    for (std::set<unsigned int>::iterator it = blobOutlineSet.begin();
      it != blobOutlineSet.end(); it++)
    {
      blobOutlineVector->push_back(
        cv::Point2f(
          static_cast<int>(*it) % edgesImage->cols,
          static_cast<int>(*it) / edgesImage->cols)
        );
    }

    // The area of the blob is essentialy the number of points visited
    *blobArea = static_cast<float>(visited.size());

    #ifdef DEBUG_TIME
    Timer::tick("brushfireKeypoint");
    #endif
  }



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
  void BlobDetection::brushfireKeypoints(
    const std::vector<cv::KeyPoint>& inKeyPoints,
    cv::Mat* edgesImage,
    std::vector<std::vector<cv::Point2f> >* blobsOutlineVector,
    std::vector<float>* blobsArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfireKeypoint", "validateBlobs");
    #endif

    for (int keypointId = 0; keypointId < inKeyPoints.size(); keypointId++)
    {
      // The outline points of the current blob
      std::vector<cv::Point2f> blobOutlineVector;

      // The area of the current blob
      float blobArea;

      // Apply the brushfire algorithm for the current keypoint
      brushfireKeypoint(
        inKeyPoints[keypointId], edgesImage, &blobOutlineVector, &blobArea);

      // Push back the blobOutlineVector to the overall outline points vector
      blobsOutlineVector->push_back(blobOutlineVector);

      // Push back the area of the blob to the overall areas vector
      blobsArea->push_back(blobArea);
    }

    #ifdef DEBUG_TIME
    Timer::tick("brushfireKeypoint");
    #endif
  }



  /**
    @brief Implements the brushfire algorithm. Its specific purpose is
    to find the points between a blob's outline and its bounding box
    (not necessarily one of least area).
    @param[in] inPoint [const cv::Point2f&] The input point
    @param[in] inImage [cv::Mat*] The input image
    @param[out] visited [std::set<unsigned int>*] The points between two
    areas of non-zero value pixels.
    @return void
   **/
  void BlobDetection::brushfirePoint(
    const cv::Point2f& inPoint,
    cv::Mat* inImage,
    std::set<unsigned int>* visited)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfirePoint");
    #endif

    // Get a pointer on the input image
    unsigned char* ptr = inImage->ptr();

    // The sets needed by the brushfire implementation
    std::set<unsigned int> current, next;

    current.insert(static_cast<int>(round(inPoint.y) * inImage->cols)
      + static_cast<int>(round(inPoint.x)));
    visited->insert(static_cast<int>(round(inPoint.y) * inImage->cols)
      + static_cast<int>(round(inPoint.x)));

    while (current.size() != 0)
    {
      for (std::set<unsigned int>::iterator it = current.begin() ;
        it != current.end() ; it++)
      {
        // sweep the neighbors of the current point
        for (int m = -1; m < 2; m++)
        {
          for (int n = -1; n < 2; n++)
          {
            // This check is needed because it is possible to access
            // a point diagonal to the current one
            // that goes out of the image's borders (value == 0),
            // while it shouldn't.
            // E.g. this happens when there are "cracks" to the border of a
            // 1-pixel border
            if (abs(m) + abs(n) < 2)
            {
              // Column-wise coordinate
              int x = static_cast<int>(*it) % inImage->cols + m;

              // Row-wise coordinate
              int y = static_cast<int>(*it) / inImage->cols + n;

              // The index of this neighbor inside image inImage
              int ind = y * inImage->cols + x;

              // If this neighbor is not within the image's borders,
              // discard it and move on
              if (x < 0 || y < 0 ||
                x > inImage->cols - 1 || y > inImage->rows - 1)
              {
                continue;
              }

              // The value of the point with index ind in edgesImage
              char v = ptr[ind];

              // If this neighbor is a blank pixel and
              // it has not been visited, put it in set "next"
              if ((v == 0) && visited->find(ind) == visited->end())
              {
                next.insert(ind);
              }

              // Whatever the condition, this neighbor has been visited
              visited->insert(ind);
            }
          }
        }
      }

      // The set of all future points becomes the one with which the
      // brushfire will begin
      current.swap(next);

      // Clear set "next" for the next iteration
      next.clear();
    }

    #ifdef DEBUG_TIME
    Timer::tick("brushfirePoint");
    #endif
  }



  /**
    @brief Detects blobs in an image
    @param[in] inImage [const cv::Mat&] The input image
    @param[out] keyPointsOut [std::vector<cv::KeyPoint>*] The ouput
    @return void
   **/
  void BlobDetection::detectBlobs(const cv::Mat& inImage,
    std::vector<cv::KeyPoint>* keyPointsOut)
  {
    #ifdef DEBUG_TIME
    Timer::start("detectBlobs", "findHoles");
    #endif

    cv::SimpleBlobDetector::Params params;

    params.minThreshold = Parameters::Blob::min_threshold; //40;
    params.maxThreshold = Parameters::Blob::max_threshold; //60;
    params.thresholdStep = Parameters::Blob::threshold_step;

    params.minArea = Parameters::Blob::min_area;
    params.maxArea = Parameters::Blob::max_area;

    params.minConvexity = Parameters::Blob::min_convexity; //0.6;
    params.maxConvexity = Parameters::Blob::max_convexity;

    params.minInertiaRatio = Parameters::Blob::min_inertia_ratio;//0.5;

    params.maxCircularity = Parameters::Blob::max_circularity;
    params.minCircularity = Parameters::Blob::min_circularity; //0.3;

    params.filterByColor = Parameters::Blob::filter_by_color;
    params.filterByCircularity = Parameters::Blob::filter_by_circularity;

    cv::SimpleBlobDetector blobDetector(params);
    blobDetector.create("SimpleBlob");

    std::vector<cv::KeyPoint> keyPoints;

    // detect blobs. store their center point
    blobDetector.detect(inImage, keyPoints);

    for (int keypointId = 0; keypointId < keyPoints.size(); keypointId++)
    {
      // if the keypoint is out of image limits, discard it
      if (keyPoints[keypointId].pt.x < inImage.cols &&
        keyPoints[keypointId].pt.x >= 0 &&
        keyPoints[keypointId].pt.y < inImage.rows &&
        keyPoints[keypointId].pt.y >= 0)
      {
        keyPointsOut->push_back(keyPoints[keypointId]);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("detectBlobs");
    #endif
  }



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
  void BlobDetection::raycastKeypoint(
    const cv::KeyPoint& inKeyPoint,
    cv::Mat* edgesImage,
    const int& partitions,
    const bool& findArea,
    std::vector<cv::Point2f>* blobOutlineVector,
    float* blobArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("raycastKeypoint");
    #endif

    // Get a pointer on edgesImage
    unsigned char* ptr = edgesImage->ptr();

    // A vector storing the non-zero pixels surrounding the keypoint
    std::vector<cv::Point2f> keypointOutline;

    // The angle of a ray relative to the horizontal axis
    float theta = 0;

    // The increment of a ray's angle
    float thetaIncrement = 2 * M_PI / partitions;

    // Make a complete revolution
    for (unsigned int angleId = 0; angleId < partitions; angleId++)
    {
      // Indicates whether this ray has hit a non-zero value point
      bool outlineFound = false;

      // Variable responsible for advancing the tip of the ray until it
      // finds a non-zero value point
      int counter = 0;

      // A ray can hit up to 5 outline points, but only one must be chosen.
      // Store these points as potential outline points.
      // We will select only the first one found
      std::vector<cv::Point2f> singleRayPotentialOutlinePoints;

      while(!outlineFound)
      {
        // Advance the tip of the ray forwards
        counter++;

        for (int m = -1; m < 2; m++)
        {
          for (int n = -1; n < 2; n++)
          {
            int x = inKeyPoint.pt.x + m + counter * cos(theta);
            int y = inKeyPoint.pt.y + n + counter * sin(theta);

            // Has the ray gone out of the image's bounds?
            // If yes, impose limits
            bool outOfBounds = false;

            if (x < 0)
            {
              x = 0;
              outOfBounds = true;
            }

            if (y < 0)
            {
              y = 0;
              outOfBounds = true;
            }

            if (x > edgesImage->cols - 1)
            {
              x = edgesImage->cols - 1;
              outOfBounds = true;
            }

            if (y > edgesImage->rows - 1)
            {
              y = edgesImage->rows - 1;
              outOfBounds = true;
            }

            // The index of the neighbor of the ray's tip
            int ind = y * edgesImage->cols + x;


            // .. and its value in the edgesImage image
            char v = ptr[ind];

            // If the neighbor has a non-zero value, or it is a boundary
            // pixel (hence there is no valid value,
            // and one has to be imposed), this point is an outline point
            if (v != 0 || outOfBounds)
            {
              outlineFound = true;
              singleRayPotentialOutlinePoints.push_back(cv::Point2f(x, y));
            }
          }
        }
      } // End {while outline not found} loop


      // From the, at most 5, outline points found,
      // regard only one of them as an outline point, so that, in total,
      // their number equals the number of partitions
      // (Needed to approximate fairly accurately the blob's area)
      keypointOutline.push_back(singleRayPotentialOutlinePoints[0]);

      // Increase the angle of the ray
      theta += thetaIncrement;
    }

    // If the area of the blob needs to be returned
    if (findArea)
    {
      // Calculate each blob's approximate area by heron's formula
      // https://en.wikipedia.org/wiki/Heron's_formula
      for (unsigned int t = 0; t < keypointOutline.size(); t++)
      {
        // calculate the area of each triangle found
        // O is the keypoint and A, B two successive outline points
        float lengthOA = sqrt(
          pow(inKeyPoint.pt.x - keypointOutline[t].x, 2)
          + pow(inKeyPoint.pt.y - keypointOutline[t].y, 2));

        float lengthOB = sqrt(
          pow(inKeyPoint.pt.x - keypointOutline[(t + 1) % partitions].x, 2)
          + pow(inKeyPoint.pt.y - keypointOutline[(t + 1) % partitions].y, 2));

        float lengthAB = sqrt(
          pow(keypointOutline[t].x
            - keypointOutline[(t + 1) % partitions].x, 2)
          + pow(keypointOutline[t].y
            - keypointOutline[(t + 1) % partitions].y, 2));

        float semiperimeter = (lengthOA + lengthOB + lengthAB) / 2;

        *blobArea += sqrt(semiperimeter
          * (semiperimeter - lengthOA)
          * (semiperimeter - lengthOB)
          * (semiperimeter - lengthAB));
      }
    }
    else
    {
      *blobArea = 0.0;
    }

    // Instead of keeping the sparce points that are the product of
    // the raycast algorithm, connect them linearly in order to
    // have a coherent set of points as the hole's outline
    cv::Mat canvas = cv::Mat::zeros(edgesImage->size(), CV_8UC1);

    // Draw the connected outline of the i-th hole onto canvas
    for(unsigned int j = 0; j < keypointOutline.size(); j++)
    {
      cv::line(canvas, keypointOutline[j],
        keypointOutline[(j + 1) % keypointOutline.size()],
        cv::Scalar(255, 0, 0), 1, 8 );
    }

    // Clear the outline vector. It will be filled with the points
    // drawn on the canvas image
    keypointOutline.clear();

    // Every non-zero point is a point drawn; it is a point that
    // belongs to the outline of the hole
    for (unsigned int rows = 0; rows < canvas.rows; rows++)
    {
      for (unsigned int cols = 0; cols < canvas.cols; cols++)
      {
        if (canvas.at<unsigned char>(rows, cols) != 0)
        {
          keypointOutline.push_back(cv::Point2f(cols, rows));
        }
      }
    }

    // The final outline points vector
    *blobOutlineVector = keypointOutline;

    #ifdef DEBUG_TIME
    Timer::tick("raycastKeypoint");
    #endif
  }



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
  void BlobDetection::raycastKeypoints(
    const std::vector<cv::KeyPoint>& inKeyPoints,
    cv::Mat* edgesImage,
    const int& partitions,
    std::vector<std::vector<cv::Point2f> >* blobsOutlineVector,
    std::vector<float>* blobsArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("raycastKeypoint", "validateBlobs");
    #endif

    for (int i = 0; i < inKeyPoints.size(); i++)
    {
      // The current blob's outline
      std::vector<cv::Point2f> keypointOutline;

      // The current blob's area
      float area = 0.0;

      // Find the outline and the area of the current keypoint
      raycastKeypoint(inKeyPoints[i],
        edgesImage,
        partitions,
        true,
        &keypointOutline,
        &area);

      // Push the blob's area back into the vector of areas
      blobsArea->push_back(area);

      // Push the blob's outline back into the vector of blobs' outline points
      blobsOutlineVector->push_back(keypointOutline);
    }

    #ifdef DEBUG_TIME
    Timer::tick("raycastKeypoint");
    #endif
  }

} // namespace pandora_vision
