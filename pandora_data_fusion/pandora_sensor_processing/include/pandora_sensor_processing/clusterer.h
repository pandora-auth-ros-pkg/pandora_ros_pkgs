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

#ifndef SENSOR_PROCESSING_CLUSTERER_H
#define SENSOR_PROCESSING_CLUSTERER_H

#include <boost/shared_ptr.hpp>

#include "pandora_sensor_processing/utils.h"

namespace pandora_sensor_processing
{

  /**
   * @brief Class containing various utility functions
   * and mathematical manipulations.
   */
  class Clusterer : private boost::noncopyable
  {
    public:
      /**
       * @brief Constructor
       * @param measurementSize [int] raw input's number of cells
       * @param maxClusterMemory [int] clusterer's memory depth
       * @param maxIterations [int] maximum times that clustering will iterate
       */
      Clusterer(int measurementSize,
          int maxClusterMemory = 3, int maxIterations = 100);

      /**
       * @brief Puts new measurement data to data set. Overwrites oldest's data,
       * if dataSet_ tracks already maxTracked_.
       * @param newMeasurement [Eigen::MatrixXf] must be (4, measurementSize_)
       * @throw Throws std::range_error if measurement from the same frame has not
       * consistent size.
       * @return void
       */
      void renewDataSet(Eigen::MatrixXf newMeasurement);

      /**
       * @brief Performs 2-means clustering with current dataSet
       * @return bool true if 'converged' successfully, false if reached iteration
       * limit.
       */
      bool cluster();

      /**
       * @brief Getter for how many measurements has this clusterer clustered.
       * @return unsigned int measurementsCounter_
       */
      unsigned int getMeasurementsCounter() const
      {
        return measurementsCounter_;
      }

      /**
       * @brief Getter for current measurement's mean that
       * belongs to cluster 1.
       * @param mean [Eigen::Vector4f*] vector representing vector to be filled.
       * @return bool true if there exists at least one cell from current
       * measurement to cluster 1.
       */
      bool getCurrentMean1(Eigen::Vector4f* mean) const
      {
        if (currentExistsInCluster1_)
        {
          *mean = currentMean1_;
          return true;
        }
        return false;
      }

      /**
       * @brief Getter for current measurement's mean that
       * belongs to cluster 2.
       * @param mean [Eigen::Vector4f*] vector representing vector to be filled.
       * @return bool true if there exists at least one cell from current
       * measurement to cluster 2.
       */
      bool getCurrentMean2(Eigen::Vector4f* mean) const
      {
        if (currentExistsInCluster2_)
        {
          *mean = currentMean2_;
          return true;
        }
        return false;
      }

      /**
       * @brief Getter for mean of cluster 1.
       * @return Eigen::Vector4f mean
       */
      Eigen::Vector4f getMean1() const
      {
        return mean1_;
      }

      /**
       * @brief Getter for mean of cluster 2.
       * @return Eigen::Vector4f mean
       */
      Eigen::Vector4f getMean2() const
      {
        return mean2_;
      }

      /**
       * @brief Getter for covariance matrix of cluster 1.
       * @return Eigen::Matrix4f covariance
       */
      Eigen::Matrix4f getCovariance1() const
      {
        return covariance1_;
      }

      /**
       * @brief Getter for covariance matrix of cluster 2
       * @return Eigen::Matrix4f covariance
       */
      Eigen::Matrix4f getCovariance2() const
      {
        return covariance2_;
      }

      /**
       * @brief Getter for cluster 1
       * @return Eigen::MatrixXf cluster 1
       */
      Eigen::MatrixXf getCluster1() const
      {
        return cluster1_;
      }

      /**
       * @brief Getter for cluster 2
       * @return Eigen::MatrixXf cluster 2
       */
      Eigen::MatrixXf getCluster2() const
      {
        return cluster2_;
      }

      /**
       * @brief Setter for max iterations tha clustering makes.
       * @param iterations [int] maximum times that clustering iterates
       * @return void
       */
      void setMaxIterations(int iterations)
      {
        maxIterations_ = iterations;
      }

    private:
      void
        calculateCovariances();
      bool
        calculateMeans();

      /**
       * @brief Chooses from dataSet_ two data that will be the initial
       * clusters from which 2-means clustering will begin.
       * @return void
       */
      void
        chooseInitialClusters();

    private:
      float currentTime_;
      bool readyToCluster_;
      int measurementSize_;
      int maxClusterMemory_;
      unsigned int measurementsCounter_;
      int maxIterations_;
      Eigen::MatrixXf dataSet_;

      Eigen::Vector4f mean1_;
      Eigen::Vector4f mean2_;
      Eigen::Matrix4f covariance1_;
      Eigen::Matrix4f covariance2_;
      Eigen::MatrixXf cluster1_;
      Eigen::MatrixXf cluster2_;

      bool currentExistsInCluster1_;
      Eigen::Vector4f currentMean1_;
      bool currentExistsInCluster2_;
      Eigen::Vector4f currentMean2_;

    private:
      friend class ClustererTest;
  };

  typedef boost::shared_ptr<Clusterer> ClustererPtr;
  typedef boost::shared_ptr<Clusterer const> ClustererConstPtr;

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_CLUSTERER_H
