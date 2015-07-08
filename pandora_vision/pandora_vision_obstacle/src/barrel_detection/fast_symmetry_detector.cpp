/*********************************************************************
 * The MIT License (MIT)
 * 
 * Copyright (c) 2014 Saburo Okita saburo.okita@gmail.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *********************************************************************/

#include <vector>
#include <utility>
#include <limits>
#include "pandora_vision_obstacle/barrel_detection/fast_symmetry_detector.h"

#define within(val, bottom, top) (val > bottom && val < top)
namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  FastSymmetryDetector::FastSymmetryDetector(const cv::Size imageSize,
                    const cv::Size houghSize, const int rotResolution)
  {
    this->imageSize = imageSize;
    this->center = cv::Point2f(imageSize.width - 1.0, imageSize.height - 1.0) * 0.5;
    this->diagonal = hypotf(imageSize.width, imageSize.height);
    this->rhoDivision = diagonal;
    this->rhoMax = houghSize.width;
    this->thetaMax = houghSize.height;

    rotMatrices.resize(thetaMax, cv::Mat(2, 2, CV_32FC1));

    float thetaIncDeg = 180.0f / thetaMax;
    float halfThetaMax = thetaMax * 0.5f;

    /* Pre calculate rotation matrices from -90 deg to 90 deg (actually to 89 deg) */
    for (int t = 0; t < thetaMax; t ++)
    {
      double angle = thetaIncDeg * (t - halfThetaMax);
      cv::Mat rotation = cv::getRotationMatrix2D(cv::Point2f(0.0, 0.0), angle, 1.0);
      rotation.convertTo(rotation, CV_32FC1);

      rotMatrices[t] = cv::Mat(rotation, cv::Rect(0, 0, 2, 2));
      rotMatrices[t].row(0) *= 0.5;
    }

    accum = cv::Mat::zeros(thetaMax + 2, rhoMax, CV_32FC1);
    rotEdges = cv::Mat::zeros(rhoDivision, diagonal, CV_32FC1);
    reRows.resize(rhoDivision);
    maxDistance = std::numeric_limits<float>::min();
    maxY = std::numeric_limits<float>::min();
    minY = std::numeric_limits<float>::max();
  }

  /**
   * Rotate each edge for a given theta, and append them into rotated edges matrix
   */
  void FastSymmetryDetector::rotateEdges(const std::vector<cv::Point2f>& edges, int theta)
  {
    /* Get the cos and sin values from our pre-calculated rotation matrices */
    float r0 = rotMatrices[theta].at<float>(0, 0);
    float r1 = rotMatrices[theta].at<float>(0, 1);
    float r2 = rotMatrices[theta].at<float>(1, 0);
    float r3 = rotMatrices[theta].at<float>(1, 1);

    /* Reset our row pointers to start of each row in rotated edges matrix */
    for (int i = 0; i < rhoDivision; i ++)
      reRows[i] = rotEdges.ptr<float>(i);

    float halfDiag = cvRound(diagonal) * 0.5;
    float fourthRho = rhoMax * 0.25;

    /* Now append the corresponding rho values of the rotated edges ot rotated edges matrix */
    for (int i = 0; i < edges.size(); i ++)
    {
      int rho = r2 * edges[i].x + r3 * edges[i].y + halfDiag;
      *(reRows[rho]++) = r0 * edges[i].x + r1 * edges[i].y + fourthRho;
      if (r1 * edges[i].y > maxY)
        maxY = r1 * edges[i].y;
      else if (r1 * edges[i].y < minY)
        minY = r1 * edges[i].y;
    }
  }

  /**
   * Create the hough accumulation matrix, and vote for each pair of symmetrical edges
   */
  void FastSymmetryDetector::vote(const cv::Mat& image, int minPairDist, int maxPairDist)
  {
    float minDist = minPairDist * 0.5;
    float maxDist = maxPairDist * 0.5;

    /* Make sure that we reset the accumulation matrix and rotated edges matrix */
    accum = cv::Scalar::all(0);
    rotEdges = cv::Scalar::all(0);

    /* Find all the pixels of the edges */
    std::vector<cv::Point> tempEdges;
    if (cv::countNonZero(image) != 0)
      cv::findNonZero(image, tempEdges);

    /* Translate them in relation to center of the image */
    std::vector<cv::Point2f> edges;
    for (int i = 0; i < tempEdges.size(); i ++)
      edges.push_back(cv::Point2f(tempEdges[i].x - center.x, tempEdges[i].y - center.y));


    /* For each degree of rotation */
    for (int t = 0; t < thetaMax; t ++)
    {
      float* accum_ptr = accum.ptr<float>(t);

      /* Rotate edge to that degree */
      rotateEdges(edges, t);

      for (int i = 0; i < rhoDivision; i ++)
      {
        float* colStart = rotEdges.ptr<float>(i);
        float* colEnd = reRows[i];

        /* Ignore edges that have smaller number of pairings */
        if ((colEnd - colStart) <= 1)
        {
          continue;
        }

        /* Vote for Hough matrix */
        for (float* x0 = colStart; x0 != colEnd - 1; x0 ++)
        {
          for (float* x1 = x0 + 1; x1 != colEnd; x1 ++)
          {
            float dist = std::fabs(*x1 - *x0);

            if (!within(dist, minDist, maxDist))
              break;
            if (dist > maxDistance)
              maxDistance = dist;

            int rhoIndex = static_cast<int>(*x0 + *x1);
            accum_ptr[rhoIndex]++;
          }
        }
      }
    }
  }


  /**
   * Retrieve the accumulation matrix
   */
  cv::Mat FastSymmetryDetector::getAccumulationMatrix(float thresh)
  {
    cv::Mat thresholded;
    accum.copyTo(thresholded, accum >= thresh);
    return thresholded.t();
  }



  /**
   * Find the lines that fit the symmetrical object from the calculated Hough accumulation matrix
   **/
  std::vector<std::pair<cv::Point, cv::Point> > FastSymmetryDetector::getResult(int noOfPeaks, float threshold)
  {
    std::vector<std::pair<cv::Point, cv::Point> > result;

    /* Make sure that we have appropriate peaks */
    noOfPeaks = MAX(0, noOfPeaks);

    /* Pre-set the size of the neighbors */
    int rhoNeighbors = rhoMax / 20.0f,
        thetaNeighbors = thetaMax / 20.0f;

    /* Create a mask to avoid searching peaks around the padding, and for everything that's below the specified
       threshold */
    cv::Mat mask(accum.size(), CV_8UC1, cv::Scalar(255));
    mask.row(0) = cv::Scalar(0);
    mask.row(mask.rows - 1) = cv::Scalar(0);
    mask = mask & (accum >= threshold);

    cv::Mat temp = accum.clone();


    for (int peak = 0; peak < noOfPeaks; peak ++)
    {
      /* Find the peak from the Hough accumulation matrix */
      double maxVal;
      cv::Point maxLoc;
      cv::minMaxLoc(temp, NULL, &maxVal, NULL, &maxLoc, mask);

      int rhoIndex   = maxLoc.x;
      int thetaIndex = maxLoc.y;

      if (!within(rhoIndex, 0, rhoMax - 1))
        break;

      if (!within(thetaIndex, 0, thetaMax))
        break;

      /* Convert from Hough space back to x-y space */
      result.push_back(getLine(rhoIndex, thetaIndex));

      /* Try to zero out the peak and the neighborhood of the peak, so that */
      /* we can move on to find the second highest peak */
      int r0 = MAX(0, rhoIndex - rhoNeighbors);
      int r1 = MIN(rhoMax - 1, rhoIndex + rhoNeighbors);
      int t0 = thetaIndex - thetaNeighbors;
      int t1 = thetaIndex + thetaNeighbors;

      if (t0 <= 0 || t1 >= thetaMax + 1)
      {
        /* Handles the edge case that wraps around the matrix */
        cv::Rect neighbor1, neighbor2;
        if (t0 <= 0)
        {
          neighbor1 = cv::Rect(r0, 0, r1 - r0, t1);
          r1 = rhoDivision - r1;
          neighbor2 = cv::Rect(r1, t0 + thetaMax - 1, r1 - r0, thetaMax + 1);
        }

        if (t1 >= thetaMax + 1)
        {
          neighbor1 = cv::Rect(r0, t0, r1 - r0, thetaMax + 1);
          neighbor2 = cv::Rect(r0, 0, r1 - r0, t1 - (thetaMax + 1));
        }

        cv::rectangle(temp, neighbor1, cv::Scalar(0), CV_FILLED);
        cv::rectangle(temp, neighbor2, cv::Scalar(0), CV_FILLED);
      }
      else
      {
        cv::rectangle(temp, cv::Rect(r0, t0, r1 - r0, t1 - t0), cv::Scalar(0), CV_FILLED);
      }
    }


    return result;
  }


  /**
   * Return a pair of points that describe the line based on the given
   * rho and theta in the Hough space
   */
  std::pair<cv::Point, cv::Point> FastSymmetryDetector::getLine(float rhoIndex, float thetaIndex)
  {
    float halfRhoMax = rhoMax * 0.5f;
    float halfThetaMax = thetaMax * 0.5f;

    float rho = (rhoIndex - halfRhoMax + 0.5f) * (diagonal / (rhoMax - 1.0f));
    float theta = (thetaIndex - halfThetaMax - 1.0f) * (M_PI / thetaMax);

    float cosTheta = cosf(theta);
    float sinTheta = sinf(theta);

    float xR = center.x + rho * cosTheta;
    float yR = center.y + rho * sinTheta;

    std::vector<float> d(4, std::numeric_limits<float>::max());
    if (sinTheta != 0.0)
    {
      d[0] = xR / sinTheta;
      d[1] = (xR - imageSize.width + 1) / sinTheta;
    }
    if (cosTheta != 0.0)
    {
      d[2] = -yR / cosTheta;
      d[3] = (imageSize.height - 1 - yR) / cosTheta;
    }

    float minD = std::numeric_limits<float>::max();
    for (int i = 0; i < d.size(); i ++)
    {
      if (d[i] > 0)
        minD = MIN(minD, d[i]);
    }

    cv::Point p0(-minD * sinTheta + xR, minD * cosTheta + yR);

    d.assign(4, std::numeric_limits<float>::max());
    if (sinTheta != 0.0)
    {
      d[0] = -xR / sinTheta;
      d[1] = (imageSize.width - xR - 1) / sinTheta;
    }
    if (cosTheta != 0.0)
    {
      d[2] = yR / cosTheta;
      d[3] = (1 + yR - imageSize.height) / cosTheta;
    }
    minD = std::numeric_limits<float>::max();
    for (int i = 0; i < d.size(); i ++)
    {
      if (d[i] > 0)
        minD = MIN(minD, d[i]);
    }

    cv::Point p1(minD * sinTheta + xR, -minD * cosTheta + yR);
    std::pair<cv::Point, cv::Point> pairReturn(p0, p1);
    return pairReturn;

    // return pair<Point, Point> { p0, p1, };
  }

  void FastSymmetryDetector::getMaxDistance(float* maxDistance)
  {
    (*maxDistance) = this->maxDistance;
  }

  void FastSymmetryDetector::getYCoords(float* maxY, float* minY)
  {
    (*maxY) = this->maxY;
    (*minY) = this->minY;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

