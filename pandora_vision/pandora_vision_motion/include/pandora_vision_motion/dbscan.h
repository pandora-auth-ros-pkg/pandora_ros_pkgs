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
* Author:  Despoina Pascahlidou
*********************************************************************/

#ifndef PANDORA_VISION_MOTION_DBSCAN_H
#define PANDORA_VISION_MOTION_DBSCAN_H

#include "opencv2/opencv.hpp"
#include <map>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace pandora_vision
{
  typedef std::vector<int32_t> Neighbors;
  typedef std::vector<bool> NoisePoints;
  typedef std::vector<bool> VisitedPoints;
  typedef std::vector<bool> ClusteredPoints;

  class DBSCAN
  {
    private:
      //!< Eps radius, two points are neighbors if their distance
      //!< is smaller than this threshold value
      double _eps;
      //!< Minimum number of points required to form a dense region
      size_t _minPts;

      int _cluster_id;

      //!< Vector of visited points
      VisitedPoints _visitedPoints;
      //!< Vector of clustered data
      ClusteredPoints _clusteredPoints;
      NoisePoints _noise;
      std::map<int, int> _labels;

      std::vector<cv::Rect>& _data;

      /**
       @brief Function that initializes all vectors to begin with the
       clustering process. At the beginning both visited and clustered data
       are set to false, for each point of interest.
       @param num_of_points: Number of points to be created
       @return void
      */
      void init(unsigned int num_of_points);

      /**
       @brief Function that check if a point has already been visited, if yes
       true is returned, if no false is returned.
       @param iterator Index of vector _visitedPoints that corresponds
       to the current point we are checking
       @return void
      */
      bool isVisited(int iterator);

      /**
       @brief Function tha caclulates distance between two points
       @param pt1: First point
       @param pt2: Second point
       @return their distance
      */
      double dist2d(cv::Point2d pt1, cv::Point2d pt2);

      /**
       @brief Function that returns all points with P's eps-neighborhood
       @param P:current point,we are processeing
       @return vector of all point in the neighborhoud
      */
      std::vector<int> regionQuery(int p);

       /**
       @brief If a point is found to be a dense part of a cluster, its
       ε-neighborhood is also part of that cluster. Hence, all points that
       are found within the ε-neighborhood are added, as is their own ε-neighborhood when
       they are also dense. This process continues until the density-connected cluster
       is completely found.
       @param p Point p, whose ε-neighborhood we are calculating
       @param  neighbours, found neighbours for current point
       @param void
      */
      void expandCluster(int p, std::vector<int> neighbours);

      double calculateDistanceMatrix(int ai, int bi);

      double *DP;

    public:
      //!< Class constructor
      explicit DBSCAN(std::vector<cv::Rect>& _data, double eps, int minPts);

      ~DBSCAN();

      void dbscan_cluster();

      /**
       @brief Function that returns all clusters calculated from the given
       set of points
       @param void
       @return vector of clusters
      */
      std::vector<std::vector<cv::Rect> > getGroups();
  };
}  // namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_DBSCAN_H
