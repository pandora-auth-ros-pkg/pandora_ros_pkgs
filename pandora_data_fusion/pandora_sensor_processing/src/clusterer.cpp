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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include "pandora_sensor_processing/clusterer.h"

namespace pandora_sensor_processing
{

  Clusterer::
    Clusterer(int measurementSize,
        int maxClusterMemory, int maxIterations) :
      measurementSize_(measurementSize),
      maxClusterMemory_(maxClusterMemory),
      maxIterations_(maxIterations),
      readyToCluster_(false),
      measurementsCounter_(0) {}

  /**
   * @details if dataSet_ has still room for another measurement to track,
   * meaning that clusterer has not currently reached its maxClusterMemory_
   * threshold, it will resize dataSet_ and append current measurement.
   * Otherwise, it will overwrite the oldest measurement among those in
   * dataSet_. measurementsCounter_, that keeps track of that, is updated.
   */
  void Clusterer::
    renewDataSet(Eigen::MatrixXf newMeasurement)
    {
      if (newMeasurement.cols() != measurementSize_ ||
          newMeasurement.rows() != 4)
        throw std::range_error("New measurement has not right dimensions.");

      // get current measurement time
      currentTime_ = newMeasurement.col(0)(2);
      currentExistsInCluster1_ = false;
      currentExistsInCluster2_ = false;

      measurementsCounter_++;
      if (measurementsCounter_ <= maxClusterMemory_)
      {
        dataSet_.conservativeResize(4,
            measurementSize_ * measurementsCounter_);
      }
      dataSet_.block(
          0, measurementSize_ * ((measurementsCounter_ - 1) % maxClusterMemory_),
          4, measurementSize_) << newMeasurement;

      readyToCluster_ = true;
    }

  /**
   * @details Basic operation of Clusterer class.
   * Uses 2-means clustering to group dataSet_ into 2 categories. Hopefully,
   * one of the two exactly is going to qualify into a thermal alert.
   * Method also keeps track of data's mean in each cluster that correspond
   * to current measurement (where 'current' is used to mark the measurement
   * which was just added in DataSet_ through renewDataSet() method.
   */
  bool Clusterer::
    cluster()
    {
      if (!readyToCluster_)
        throw std::logic_error("Clusterer is not ready to cluster. Needs new measurement.");

      int startCurrentInCluster1 = -1, currentsInCluster1 = 0;
      int startCurrentInCluster2 = -1, currentsInCluster2 = 0;
      bool finished = false;

      //!< Implementation of 2-means clustering
      chooseInitialClusters();
      for (int ii = 0; ii < maxIterations_; ++ii)
      {
        startCurrentInCluster1 = -1;
        startCurrentInCluster2 = -1;
        currentsInCluster1 = 0;
        currentsInCluster2 = 0;
        cluster1_.resize(4, 0);
        cluster2_.resize(4, 0);
        for (int jj = 0; jj < dataSet_.cols(); ++jj)
        {
          float dist1, dist2;
          //!< Choosing cluster according to datum euclidean distance from means.
          // dist1 = Utils::getMahalanobisDistance(dataSet_.col(jj), mean1_, covariance1_);
          // dist2 = Utils::getMahalanobisDistance(dataSet_.col(jj), mean2_, covariance2_);
          dist1 = (dataSet_.col(jj) - mean1_).norm();
          dist2 = (dataSet_.col(jj) - mean2_).norm();
          if (dist1 < dist2)
          {
            //!< Tracking data in cluster which correspond to current measurement.
            if (dataSet_.col(jj)(2) == currentTime_)
            {
              currentsInCluster1++;
              if (currentsInCluster1 == 1)
                startCurrentInCluster1 = cluster1_.cols();
            }
            //!< Resizing cluster1_ by on column and appending qualified datum.
            cluster1_.conservativeResize(Eigen::NoChange, cluster1_.cols() + 1);
            cluster1_.col(cluster1_.cols() - 1) << dataSet_.col(jj);
          }
          else
          {
            //!< Tracking data in cluster which correspond to current measurement.
            if (dataSet_.col(jj)(2) == currentTime_)
            {
              currentsInCluster2++;
              if (currentsInCluster2 == 1)
                startCurrentInCluster2 = cluster2_.cols();
            }
            //!< Resizing cluster2_ by on column and appending qualified datum.
            cluster2_.conservativeResize(Eigen::NoChange, cluster2_.cols() + 1);
            cluster2_.col(cluster2_.cols() - 1) << dataSet_.col(jj);
          }
        }
        //!< Calculate clusters' means and covariances and check for convergence.
        //!< If converged then return success and find for each cluster
        //!< current measurement's means. If there is no data in a cluster from
        //!< current measurement, then this procedure is skipped.
        if (calculateMeans())
        {
          finished = true;
        }
        calculateCovariances();
        if (finished)
          break;
      }

      readyToCluster_ = false;
      if (currentsInCluster1 > 0)
      {
        currentExistsInCluster1_ = true;
        currentMean1_ = cluster1_.block(0, startCurrentInCluster1,
            4, currentsInCluster1).rowwise().mean();
      }
      if (currentsInCluster2 > 0)
      {
        currentExistsInCluster2_ = true;
        currentMean2_ = cluster2_.block(0, startCurrentInCluster2,
            4, currentsInCluster2).rowwise().mean();
      }
      return finished;
    }

  void Clusterer::
    calculateCovariances()
    {
      int ps = 0;
      Eigen::MatrixXf cluster_centered;

      ps = cluster1_.cols();
      cluster_centered = cluster1_.colwise() - mean1_;
      covariance1_ = (cluster_centered * cluster_centered.transpose()) / (ps - 1);

      ps = cluster2_.cols();
      cluster_centered = cluster2_.colwise() - mean2_;
      covariance2_ = (cluster_centered * cluster_centered.transpose()) / (ps - 1);
    }

  /**
   * @details Finds the means of current clusters. Also comprares these with
   * the means before to check for convergence.
   */
  bool Clusterer::
    calculateMeans()
    {
      Eigen::Vector4f temp1, temp2;
      if (cluster1_.cols() != 0)
        temp1 = cluster1_.rowwise().mean();
      if (cluster2_.cols() != 0)
        temp2 = cluster2_.rowwise().mean();

      bool converged = (temp1 - mean1_).norm() < 0.01 && (temp2 - mean2_).norm() < 0.01;

      mean1_ = temp1;
      mean2_ = temp2;

      return converged;
    }

  /**
   * @details Third row corresponds to temperature. Choose initial clusters
   * to be those which contain data that have temperature closest to the highest
   * and the lowest temperature respectively in data set. Find initial centroids
   * from these clusters.
   */
  void Clusterer::
    chooseInitialClusters()
    {
      int minCol = 0, maxCol = 0;
      float highest = dataSet_.row(3).maxCoeff(&maxCol);
      float lowest = dataSet_.row(3).minCoeff(&minCol);

      cluster1_.resize(4, 0);
      cluster2_.resize(4, 0);
      for (int jj = 0; jj < dataSet_.cols(); ++jj)
      {
        //!< Choosing cluster according to datum euclidean distance from means.
        if (fabs(dataSet_(3, jj) - highest) <
            fabs(dataSet_(3, jj) - lowest) || jj == maxCol)
        {
          //!< Resizing cluster1_ by on column and appending qualified datum.
          cluster1_.conservativeResize(Eigen::NoChange, cluster1_.cols() + 1);
          cluster1_.col(cluster1_.cols() - 1) << dataSet_.col(jj);
        }
        else
        {
          //!< Resizing cluster2_ by on column and appending qualified datum.
          cluster2_.conservativeResize(Eigen::NoChange, cluster2_.cols() + 1);
          cluster2_.col(cluster2_.cols() - 1) << dataSet_.col(jj);
        }
      }

      calculateMeans();
      calculateCovariances();
    }

}  // namespace pandora_sensor_processing
