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
* Author:  Despoina Paschalidou
*********************************************************************/

#include <vector>
#include "pandora_vision_motion/dbscan.h"

namespace pandora_vision
{
namespace pandora_vision_motion
{

  /**
   @brief Class constructor
  */
  DBSCAN::DBSCAN(std::vector<cv::Point>& data, double eps, int minPts): _data(data)
  {
    _cluster_id = -1;
    _eps = eps;
    _minPts = minPts;

    for (int i = 0; i < data.size(); i++)
    {
      _labels[i] = -99;
      _visitedPoints.push_back(false);
      // _clusteredPoints.push_back(false);
    }
  }

  DBSCAN::~DBSCAN()
  {
  }

  /**
   @brief Function that initializes all vectors to begin with the
   clustering process. At the beginning both visited and clustered data
   are set to false, for each point of interest.
   @param num_of_points: Number of points to be created
   @return void
  */
  void DBSCAN::init(unsigned int num_of_points)
  {
    for (int i = 0; i < num_of_points; i++)
    {
      _labels[i] = -99;
      _visitedPoints.push_back(false);
      _clusteredPoints.push_back(false);
    }
  }

  /**
   @brief Function that check if a point has already been visited, if yes
   true is returned, if no false is returned.
   @param iterator Index of vector _visitedPoints that corresponds
   to the current point we are checking
   @return void
  */
  bool DBSCAN::isVisited(int iterator)
  {
    if (_visitedPoints.at(iterator) == false)
      return false;
    else
      return true;
  }

  /**
   @brief Function tha caclulates distance between two points
   @param pt1: First point
   @param pt2: Second point
   @return their distance
  */
  double DBSCAN::dist2d(cv::Point pt1, cv::Point pt2)
  {
    return sqrt(pow(pt1.x-pt2.x, 2) + pow(pt1.y-pt2.y, 2));
  }

  /**
   @brief Function that returns all points with P's eps-neighborhood
   @param P:current point,we are processeing
   @return vector of all point in the neighborhoud
  */
  void DBSCAN::regionQuery(int p, std::vector<int>* res)
  {
    int c =0;
    for (int i = 0; i < _data.size(); i++)
    {
      if (DP[i*_data.size()+p] <= _eps)  // (calculateDistanceMatrix(p, i) < _eps)// (DP[i*_data.size()+p] <= _eps)
      {
        res->push_back(i);
        int x = (*res)[c];
        c++;
      }
    }
  }

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
  void DBSCAN::expandCluster(int p, std::vector<int>* neighbours)
  {
    _labels[p] = _cluster_id;

    std::vector<int> neighbours_p, newNeighours;
    /// For each point in P' neighbor
    for (int i = 0; i < neighbours->size(); i++)
    {
      /// If P' is not visited
      if (!isVisited(neighbours->at(i)))
      {
        /// Mark P' as visited
        _visitedPoints.at(neighbours->at(i)) = true;

        _labels[neighbours->at(i)] = _cluster_id;

        neighbours_p.clear();
        regionQuery(neighbours->at(i), &neighbours_p);
        if (neighbours_p.size() >= _minPts)
        {
          bool flag;
          for (int j = 0; j < neighbours_p.size(); j++)
          {
            flag = false;
            newNeighours.clear();
            for (int k = 0; k < neighbours->size(); k++)
            {
               if (neighbours_p[j] == neighbours->at(k))
               {
                 flag = true;
                 break;
               }
            }
            if (!flag)
            {
              newNeighours.push_back(neighbours_p[j]);
            }

          neighbours->insert(neighbours->end(), newNeighours.begin(), newNeighours.end());
          }
        }
        // expandCluster(neighbours[i], neighbours_p);
      }
      if (_labels[neighbours->at(i)]== -99)
      {
        _labels[neighbours->at(i)] = _cluster_id;
      }
    }
  }

  /**
   @brief Function that returns all clusters calculated from the given
   set of points
   @param void
   @return vector of clusters
  */
  std::vector<std::vector<cv::Point> > DBSCAN::getClusters()
  {
    std::vector<std::vector<cv::Point> > clusters;
    for (int i = 0; i <= _cluster_id; i++)
    {
      clusters.push_back(std::vector<cv::Point>());
      for (int j = 0; j < _data.size(); j++)
      {
        if (_labels[j] == i)
          clusters[clusters.size()-1].push_back(_data[j]);
       /*  if(_labels[j] == -1) */
          /* noise.push_back(_data[j]); */
      }
    }

    // *noisePoints = noise;
    return clusters;
  }

  std::vector<double> DBSCAN::getCohesion(const std::vector<std::vector<cv::Point> >& clusters)
  {
    std::vector<double> cohesion;
    std::vector<cv::Point2d> centroid;

    /// calculate centroids
    for (int i = 0; i < clusters.size(); i++)
    {
      cohesion.push_back(0.0);
      centroid.push_back(cv::Point2d(0, 0));
      for (int j = 0; j < clusters[i].size(); j++)
      {
        centroid[i].x += clusters[i][j].x;
        centroid[i].y += clusters[i][j].y;
      }
      ROS_INFO_STREAM("d");
      centroid[i].x /= clusters[i].size();
      centroid[i].y /= clusters[i].size();
    }

    /// calculate cohesion
    for (int i = 0; i < clusters.size(); i++)
      for (int j = 0; j < clusters[i].size(); j++)
      {
        cohesion[i] += sqrt(pow(clusters[i][j].x - centroid[i].x, 2) +
                    pow(clusters[i][j].y - centroid[i].y, 2));
      }

    /* for (int i = 0; i < clusters.size(); i++) */
    // {

      // ROS_INFO_STREAM("d");
      // cohesion.push_back(0.0);
      // for( int j = 0; j < clusters[i].size(); j++)
        // for (int k = 0; k < clusters[i].size(); k++)
        // {
          // cohesion[i] += dist2d(clusters[i][j], clusters[i][k]);
        // }
      // cohesion[i] /= clusters[i].size();
    /* } */
    return cohesion;
  }

  void DBSCAN::cluster()
  {
     try
    {
      DP = new double[_data.size() * _data.size()];
      for (int i = 0; i < _data.size(); i++)
      {
        for (int j = 0; j < _data.size(); j++)
        {
          if (i == j)
              DP[i*_data.size()+j] = 0.0;
          else
              DP[i*_data.size()+j] = dist2d(_data[i], _data[j]);
          double x = DP[i*_data.size()+j];
           // ROS_INFO_STREAM("DP["<<i<<"]["<<j<<"]="<<x);
        }
      }
    for (int i = 0; i < _data.size(); i++)
    {
      if (isVisited(i) == false)
      {
        /// Mark current point as visited
        _visitedPoints.at(i) = true;
        std::vector<int> neighbours;
        regionQuery(i, &neighbours);
        // ROS_INFO_STREAM("FIND NEIGHBORS OF POINT " << i << " SIZE="<< neighbours.size());
          if (neighbours.size() < _minPts)/*  && i != 0) */
          {
            /// Mark P as noise
            // ROS_INFO_STREAM("POINT "<< i <<" is noise");
            _labels[i] = -1;
            _noise.push_back(i);
          }
          else
          {
            /// Expand cluster
            _cluster_id++;
            // ROS_INFO_STREAM("EXPAND" << _cluster_id);
            expandCluster(i, &neighbours);
          }
      }
    }

    delete [] DP;
    }
    catch (std::bad_alloc& ba)
    {
      std::cerr << "bad_alloc caught: " << ba.what() << '\n';
    }
  }

  double DBSCAN::calculateDistanceMatrix(int pt1, int pt2)
  {
    /* if (DP[pt1][pt2] != -1.0)  */
    // {
      // double x = DP[pt1][pt2];
      // // ROS_INFO_STREAM("dist"<< pt1 << " "<< pt2 <<"=" << x);
      // return DP[pt1][pt2];
    /* }  */
    double minDist = dist2d(_data[pt1], _data[pt2]);

  /*   cv::Rect a = _data[pt1]; */
    // cv::Rect b = _data[pt2];

    // cv::Point2d tla = cv::Point2d(a.x, a.y);
    // cv::Point2d tra = cv::Point2d(a.x + a.width, a.y);
    // cv::Point2d bla = cv::Point2d(a.x, a.y + a.height);
    // cv::Point2d bra = cv::Point2d(a.x + a.width, a.y + a.height);

    // cv::Point2d tlb = cv::Point2d(b.x, b.y);
    // cv::Point2d trb = cv::Point2d(b.x + b.width, b.y);
    // cv::Point2d blb = cv::Point2d(b.x, b.y + b.height);
    // cv::Point2d brb = cv::Point2d(b.x + b.width, b.y + b.height);

    // double minDist = 9999999;

    // minDist = std::min(minDist, dist2d(tla, tlb));
    // minDist = std::min(minDist, dist2d(tla, trb));
    // minDist = std::min(minDist, dist2d(tla, blb));
    // minDist = std::min(minDist, dist2d(tla, brb));

    // minDist = std::min(minDist, dist2d(tra, tlb));
    // minDist = std::min(minDist, dist2d(tra, trb));
    // minDist = std::min(minDist, dist2d(tra, blb));
    // minDist = std::min(minDist, dist2d(tra, brb));

    // minDist = std::min(minDist, dist2d(bla, tlb));
    // minDist = std::min(minDist, dist2d(bla, trb));
    // minDist = std::min(minDist, dist2d(bla, blb));
    // minDist = std::min(minDist, dist2d(bla, brb));

    // minDist = std::min(minDist, dist2d(bra, tlb));
    // minDist = std::min(minDist, dist2d(bra, trb));
    // minDist = std::min(minDist, dist2d(bra, blb));
    /* minDist = std::min(minDist, dist2d(bra, brb)); */

    // DP[pt1][pt2] = minDist;
    // DP[pt2][pt1] = minDist;
     // // ROS_INFO_STREAM("DIST"<< pt1 << " "<< pt2 <<"=" << minDist);
    return minDist;
  }
}  // namespace pandora_vision_motion
}  // namespace pandora_vision
