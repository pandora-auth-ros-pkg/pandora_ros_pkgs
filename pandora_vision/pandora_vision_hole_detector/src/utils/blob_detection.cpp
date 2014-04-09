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

#include "utils/blob_detection.h"

namespace pandora_vision
{
  /**
    @brief Implements the brushfire algorithm for all blob keypoints in order to
    find a blob limits
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
    @param[in] edgesImage [cv::Mat*] The input image
    @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point> >*]
    The output vector containing the blobs' outline
    @param[out] blobsArea [std::vector<float>*] The area of each blob
    @return void
   **/
  void BlobDetection::brushfireKeypoint(
    const std::vector<cv::KeyPoint>& inKeyPoints,
    cv::Mat* edgesImage,
    std::vector<std::vector<cv::Point> >* blobsOutlineVector,
    std::vector<float>* blobsArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfireKeypoint", "validateBlobs");
    #endif

    unsigned char* ptr = edgesImage->ptr();

    for (int keypointId = 0; keypointId < inKeyPoints.size(); keypointId++)
    {
      std::set<unsigned int> current, next, visited;

      std::vector<cv::Point> keypointOutline;

      current.insert(
        static_cast<int>(
          round(inKeyPoints[keypointId].pt.y) * edgesImage->cols)
        + static_cast<int>(round(inKeyPoints[keypointId].pt.x)));

      visited.insert(
        static_cast<int>(
          round(inKeyPoints[keypointId].pt.y) * edgesImage->cols)
        + static_cast<int>(round(inKeyPoints[keypointId].pt.x)));

      while (current.size() != 0)
      {
        for (std::set<unsigned int>::iterator it = current.begin() ;
          it != current.end() ; it++)
        {
          //!< sweep the neighbors of the current point
          for (int m = -1; m < 2; m++)
          {
            for (int n = -1; n < 2; n++)
            {
              int x = static_cast<int>(*it) % edgesImage->cols + m;
              int y = static_cast<int>(*it) / edgesImage->cols + n;
              int ind = y * edgesImage->cols + x;

              if (x < 0 ||
                y < 0 ||
                x > edgesImage->cols - 1 ||
                y > edgesImage->rows - 1)
              {
                continue;
              }

              char v = ptr[ind];
              if ((v == 0) && visited.find(ind) == visited.end())
              {
                next.insert(ind);
              }

              if (v != 0)
              {
                keypointOutline.push_back(cv::Point(x, y));
              }

              visited.insert(ind);
            }
          }
        }
        current.swap(next);
        next.clear();
      }

      blobsArea->push_back(static_cast<float>(visited.size()));
      blobsOutlineVector->push_back(keypointOutline);
    }

    #ifdef DEBUG_TIME
    Timer::tick("brushfireKeypoint");
    #endif
  }



  /**
    @brief Implements the brushfire algorithm. Its specific purpose is
    to find the points between a blob's outline and its bounding box
    (not necessarily one of least area).
    @param[in] inPoint [const cv::Point&] The input point
    @param[in] inImage [const cv::Mat*] The input image
    @param[out] pointsCovered [std::set<unsigned int>*] The points between two
    areas of non-zero value pixels.
    @return void
   **/
  void BlobDetection::brushfirePoint(
    const cv::Point& inPoint,
    cv::Mat* inImage,
    std::set<unsigned int>* visited)
  {
    #ifdef DEBUG_TIME
    Timer::start("brushfirePoint");
    #endif

    unsigned char* ptr = inImage->ptr();

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
        //!< sweep the neighbors of the current point
        for (int m = -1; m < 2; m++)
        {
          for (int n = -1; n < 2; n++)
          {
            //!< This check is needed because it is possible to access
            //!< a point diagonal to the current one
            //!< that goes out of the border's of interest bounds
            //!< (value == 0) while it shouldn't.
            //!< This happens when there are "cracks" to the border of a
            //!< 1-pixel border
            if (abs(m) + abs(n) < 2)
            {
              int x = static_cast<int>(*it) % inImage->cols + m;
              int y = static_cast<int>(*it) / inImage->cols + n;

              int ind = y * inImage->cols + x;

              if (x < 0 || y < 0 ||
                  x > inImage->cols - 1 || y > inImage->rows - 1)
              {
                continue;
              }

              char v = ptr[ind];
              if ((v == 0) && visited->find(ind) == visited->end())
              {
                next.insert(ind);
              }

              visited->insert(ind);
            }
          }
        }
      }
      current.swap(next);
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

    params.minThreshold = Parameters::blob_min_threshold; //40;
    params.maxThreshold = Parameters::blob_max_threshold; //60;
    params.thresholdStep = Parameters::blob_threshold_step;

    params.minArea = Parameters::blob_min_area;
    params.maxArea = Parameters::blob_max_area;

    params.minConvexity = Parameters::blob_min_convexity; //0.6;
    params.maxConvexity = Parameters::blob_max_convexity;

    params.minInertiaRatio = Parameters::blob_min_inertia_ratio;//0.5;

    params.maxCircularity = Parameters::blob_max_circularity;
    params.minCircularity = Parameters::blob_min_circularity; //0.3;

    params.filterByColor = Parameters::blob_filter_by_color;
    params.filterByCircularity = Parameters::blob_filter_by_circularity;

    cv::SimpleBlobDetector blobDetector(params);
    blobDetector.create("SimpleBlob");

    std::vector<cv::KeyPoint> keyPoints;

    //!< detect blobs. store their center point
    blobDetector.detect(inImage, keyPoints);

    for (int keypointId = 0; keypointId < keyPoints.size(); keypointId++)
    {
      //!< if the keypoint is out of image limits, discard it
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
    @brief Implements a raycast algorithm for all blob keypoints in order
    to find the blob limits
    @param[in][out] inKeyPoints [std::vector<cv::KeyPoint>*] The keypoints
    @param[in] edgesImage [cv::Mat*] The input image
    @param[in] partitions [const int&] The number of directions
    towards which the outline of the blob will be sought,
    or the number of partitions in which the blob will be divided by
    the rays.  Same deal.
    @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point> >*]
    The output vector containing the blobs' (rough approximate) outline
    @param[out] blobsArea [std::vector<float>*] The area of each blob
    @return void
   **/
  void BlobDetection::raycastKeypoint(
      std::vector<cv::KeyPoint>* inKeyPoints,
      cv::Mat* edgesImage,
      const int& partitions,
      std::vector<std::vector<cv::Point> >* blobsOutlineVector,
      std::vector<float>* blobsArea)
  {
    #ifdef DEBUG_TIME
    Timer::start("raycastKeypoint", "validateBlobs");
    #endif


    unsigned char* ptr = edgesImage->ptr();

    //!< Traverse the inKeyPoints vector backwards because a keypoint
    //!< might be deleted due to it not being entirely surrounded by
    //!< non-zero pixels
    for (int keypointId = inKeyPoints->size() - 1; keypointId >= 0;
      keypointId--)
    {
      //!< A priori, this keypoint is not to be deleted
      bool deleteThisKeypoint = false;

      //!< A vector storing the non-zero pixels surrounding the keypoint
      std::vector<cv::Point> keypointOutline;

      float theta = 0;
      float thetaIncrement = 2 * 3.1415926535897 / partitions;

      for (unsigned int angleId = 0; angleId < partitions; angleId++)
      {
        bool outlineFound = false;
        int counter = 0;

        while(!outlineFound && !deleteThisKeypoint)
        {
          counter++;

          //!< If the 8 pixels surrounding the ray's pixel, including the center
          //!< pixel, are found to be all outside the image's borders,
          //!< something has gone horribly wrong. The keypoint is not surrounded
          //!< entirely by non-zero pixels. The best choice is to delete it.
          int numPixelsOutsideImageBorders = 0;

          for (int m = -1; m < 2; m++)
          {
            for (int n = -1; n < 2; n++)
            {
              int x = (*inKeyPoints)[keypointId].pt.x + m + counter * cos(theta);
              int y = (*inKeyPoints)[keypointId].pt.y + n + counter * sin(theta);

              //!< If the ray point is not within image borders
              if (x < 0 || y < 0 ||
                x > edgesImage->cols - 1 ||
                y > edgesImage->rows - 1)
              {
                numPixelsOutsideImageBorders++;

                if (numPixelsOutsideImageBorders >= 9)
                {
                  deleteThisKeypoint = true;
                }

                continue;
              }

              int ind = y * edgesImage->cols + x;
              char v = ptr[ind];

              if (v != 0)
              {
                outlineFound = true;
                keypointOutline.push_back(cv::Point(x, y));
              }
            }
          }
        } //!< End {while outline not found} loop

        //!< If this keypoint is to be deleted, break from the angles loop
        //!< in order to delete the keypoint and move on to the next one
        if (deleteThisKeypoint)
        {
          break;
        }
        else
        {
          theta += thetaIncrement;
        }
      }

      if (deleteThisKeypoint)
      {
        keypointOutline.erase(keypointOutline.begin(),
          keypointOutline.end());

        inKeyPoints->erase(inKeyPoints->begin() + keypointId);
        continue;
      }


      blobsOutlineVector->push_back(keypointOutline);

      //!< Calculate each blob's approximate area by heron's formula
      //!< https://en.wikipedia.org/wiki/Heron's_formula
      float area = 0.0;
      for (unsigned int t = 0; t < partitions; t++)
      {
        //!< calculate the area of each triangle found
        //!< O is the keypoint and A, B any two successive outline points
        float lengthOA = sqrt(
          pow((*inKeyPoints)[keypointId].pt.x - keypointOutline[t].x, 2)
          + pow((*inKeyPoints)[keypointId].pt.y - keypointOutline[t].y, 2));
        float lengthOB = sqrt(
          pow((*inKeyPoints)[keypointId].pt.x
            - keypointOutline[(t + 1) % partitions].x, 2)
          + pow((*inKeyPoints)[keypointId].pt.y
            - keypointOutline[(t + 1) % partitions].y, 2));
        float lengthAB = sqrt(
          pow(keypointOutline[t].x
            - keypointOutline[(t + 1) % partitions].x, 2)
          + pow(keypointOutline[t].y
            - keypointOutline[(t + 1) % partitions].y, 2));
        float perimeter = lengthOA + lengthOB + lengthAB;

        area += sqrt(perimeter
          * (perimeter - lengthOA)
          * (perimeter - lengthOB)
          * (perimeter - lengthAB));
      }

      blobsArea->push_back(area);
    }

    //!< Because the keypoints vector is traversed backwards,
    //!< but the elements are pushed back into keypointOutline and blobsArea,
    //!< we must reverse them in order for them to be accurately corresponded
    //!< to one another
    std::reverse(blobsOutlineVector->begin(), blobsOutlineVector->end());
    std::reverse(blobsArea->begin(), blobsArea->end());

    #ifdef DEBUG_TIME
    Timer::tick("raycastKeypoint");
    #endif
  }



  /**
    @brief Takes as input a binary image and stores in
    @outlines the outlines of closed curves.
    (Assumes that the input image comprises entirely of closed curves.)
    @param[in] inImage [cv::Mat*] The input binary image
    @param[out] outlines [std::vector<std::vector<cv::Point> >*] The points
    that each detected closed curve consists of
    @return void
   **/
  void BlobDetection::getClosedCurves(cv::Mat* inImage,
    std::vector<std::vector<cv::Point> >* outlines)
  {
    #ifdef DEBUG_TIME
    Timer::start("getClosedCurves");
    #endif

    unsigned char* ptr = inImage->ptr();

    std::set<unsigned int> current, next, visited, checked;
    std::vector<cv::Point> curve;

    for (unsigned int rows = 0; rows < inImage->rows; rows++)
    {
      for (unsigned int cols = 0; cols < inImage->cols; cols++)
      {
        int ind = rows * inImage->cols + cols;
        char v = ptr[ind];

        //!< Found a new curve?
        if (v != 0 && checked.find(ind) == checked.end())
        {
          current.insert(ind);
          visited.insert(ind);
          curve.push_back(cv::Point(rows, cols));

          while (current.size() != 0)
          {
            for (std::set<unsigned int>::iterator it = current.begin() ;
                it != current.end() ; it++)
            {
              //!< sweep the neighbors of the current point
              for (int m = -1; m < 2; m++)
              {
                for (int n = -1; n < 2; n++)
                {

                  int x = static_cast<int>(*it) % inImage->cols + m;
                  int y = static_cast<int>(*it) / inImage->cols + n;

                  if (x < 0 || y < 0 ||
                      x > inImage->cols - 1 || y > inImage->rows - 1)
                  {
                    continue;
                  }

                  int nInd = y * inImage->cols + x;
                  char nV = ptr[nInd];

                  if ((nV != 0) && visited.find(nInd) == visited.end())
                  {
                    next.insert(nInd);
                    curve.push_back(cv::Point(y, x));
                  }

                  visited.insert(nInd);
                  checked.insert(nInd);
                }
              }
            }
            current.swap(next);
            next.clear();
          }

          outlines->push_back(curve);

          current.clear();
          next.clear();
          visited.clear();
        }

        checked.insert(ind);
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("getClosedCurves");
    #endif
  }
} // namespace pandora_vision
