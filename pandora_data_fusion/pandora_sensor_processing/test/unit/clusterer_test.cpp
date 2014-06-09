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

#include "gtest/gtest.h"

#include "sensor_processing/clusterer.h"

using Eigen::MatrixXf;
using Eigen::Vector4f;

namespace pandora_sensor_processing
{

    class ClustererTest : public ::testing::Test
    { 
      public: 
        /**
         * @brief Constructor
         */
        ClustererTest() :
          dataSet_(0, 0),
          clusterMemory_(3),
          clusterer_(9, clusterMemory_, 100) {}

      protected:
        /* Helper Functions */

        bool areEquals(MatrixXf matrixA, MatrixXf matrixB)
        {
          if (matrixA.rows() != matrixB.rows() || matrixA.cols() != matrixB.cols())
          {
            throw std::range_error("Matrices have not the same dimensions.");
          }
          return (matrixA - matrixB).norm() < 0.0001;
        }

        /**
         * @brief function to fill clusterer_'s dataSet_ with a matrix
         */
        void fillDataSet1()
        {
          dataSet_.conservativeResize(4, 9 + clusterer_.dataSet_.cols());
          dataSet_.block(0, dataSet_.cols() - 9,
              4, 9) << 0, 0, 0, 1, 1, 1, 2, 2, 2,
                       0, 1, 2, 0, 1, 2, 0, 1, 2,
                       0, 0, 0, 0, 0, 0, 0, 0, 0,
                       35, 37, 35, 20, 20, 20, 20, 20, 18;
          clusterer_.dataSet_ = dataSet_;
          clusterer_.measurementsCounter_++;
        }

        /**
         * @brief function to fill clusterer_'s dataSet_ with a matrix
         */
        void fillDataSet2()
        {
          dataSet_.conservativeResize(4, 9 + clusterer_.dataSet_.cols());
          dataSet_.block(0, dataSet_.cols() - 9,
              4, 9) << 0, 0, 0, 1, 1, 1, 2, 2, 2,
                       0, 1, 2, 0, 1, 2, 0, 1, 2,
                       1, 1, 1, 1, 1, 1, 1, 1, 1,
                       35, 35, 35, 20, 20, 20, 20, 20, 20;
          clusterer_.dataSet_ = dataSet_;
          clusterer_.measurementsCounter_++;
        }

        /**
         * @brief function to fill clusterer_'s dataSet_ with a matrix
         */
        void fillDataSet3()
        {
          dataSet_.conservativeResize(4, 9 + clusterer_.dataSet_.cols());
          dataSet_.block(0, dataSet_.cols() - 9,
              4, 9) << 0, 0, 0, 1, 1, 1, 2, 2, 2,
                       0, 1, 2, 0, 1, 2, 0, 1, 2,
                       2, 2, 2, 2, 2, 2, 2, 2, 2,
                       35, 35, 30, 30, 20, 20, 20, 20, 20;
          clusterer_.dataSet_ = dataSet_;
          clusterer_.measurementsCounter_++;
        }

        /**
         * @brief function to fill clusterer_'s dataSet_ with a matrix
         */
        void fillDataSet4()
        {
          dataSet_.conservativeResize(4, 9 + clusterer_.dataSet_.cols());
          dataSet_.block(0, dataSet_.cols() - 9,
              4, 9) << 0, 0, 0, 1, 1, 1, 2, 2, 2,
                       0, 1, 2, 0, 1, 2, 0, 1, 2,
                       5, 5, 5, 5, 5, 5, 5, 5, 5,
                       23, 18, 21, 20.8, 19.7, 20, 20, 21, 23;
          clusterer_.dataSet_ = dataSet_;
          clusterer_.measurementsCounter_++;
        }

        /**
         * @brief function to fill a measurement
         * @param measurement [MatrixXf*] pointer to measurement to be filled
         * @param count [int] number of measurements since the start of clusterer.
         */
        void fillMeasurement(MatrixXf* measurement, int count)
        {
          measurement->resize(4, 9);
          *measurement << 0, 0, 0, 1, 1, 1, 2, 2, 2,
                          0, 1, 2, 0, 1, 2, 0, 1, 2,
                          3, 3, 3, 3, 3, 3, 3, 3, 3,
                          31, 30, 32, 28, 20, 20, 21, 20, 19;
          if (count <= clusterMemory_)
          {
            dataSet_.conservativeResize(4, 9 * count);
          }
          dataSet_.block(0, 9 * ((count - 1) % clusterMemory_), 4, 9) << *measurement;
        }

        /**
         * @brief function to fill clusterer_'s clusters
         */
        void fillClusters()
        {
          MatrixXf cluster1(4, 4);
          cluster1 << 0, 0, 0, 1, 
                      0, 1, 2, 0, 
                      2, 2, 2, 2,
                      35, 35, 30, 30;
          clusterer_.cluster1_ = cluster1;
          MatrixXf cluster2(4, 5);
          cluster2 << 1, 1, 2, 2, 2,
                      1, 2, 0, 1, 2,
                      2, 2, 2, 2, 2,
                      20, 20, 20, 20, 20;
          clusterer_.cluster2_ = cluster2;
        }

        /* Accessors to private variables */

        MatrixXf getDataSet()
        {
          return clusterer_.dataSet_;
        }

        float* currentTime()
        {
          return &clusterer_.currentTime_;
        }

        bool* readyToCluster()
        {
          return &clusterer_.readyToCluster_;
        }

        int* measurementSize()
        {
          return &clusterer_.measurementSize_;
        }

        int* maxClusterMemory()
        {
          return &clusterer_.maxClusterMemory_;
        }

        unsigned int* measurementsCounter()
        {
          return &clusterer_.measurementsCounter_;
        }

        int* maxIterations()
        {
          return &clusterer_.maxIterations_;
        }

        bool* currentExistsInCluster1()
        {
          return &clusterer_.currentExistsInCluster1_;
        }

        bool* currentExistsInCluster2()
        {
          return &clusterer_.currentExistsInCluster2_;
        }

        MatrixXf* getCluster1()
        {
          return &clusterer_.cluster1_;
        }

        MatrixXf* getCluster2()
        {
          return &clusterer_.cluster2_;
        }

        /* Accessors to private methods */

        void calculateCovariances()
        {
          clusterer_.calculateCovariances();
        }

        bool calculateMeans()
        {
          return clusterer_.calculateMeans();
        }

        void chooseInitialClusters()
        {
          clusterer_.chooseInitialClusters();
        }

        /* Variables */

        int clusterMemory_;
        MatrixXf dataSet_;
        Clusterer clusterer_;
    };

    /* Test Cases */

    TEST_F(ClustererTest, constructor)
    {
      EXPECT_EQ(9, *measurementSize());
      EXPECT_EQ(3, *maxClusterMemory());
      EXPECT_EQ(100, *maxIterations());
      EXPECT_FALSE(*readyToCluster());
      EXPECT_EQ(0, *measurementsCounter());
      EXPECT_THROW(clusterer_.cluster(), std::logic_error);
    }

    TEST_F(ClustererTest, calculateMeans)
    {
      fillClusters();
      Vector4f mean;

      EXPECT_FALSE(calculateMeans());
      mean << 0.25, 0.75, 2, 32.5;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean1()));
      mean << 1.6, 1.2, 2, 20;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean2()));
      EXPECT_TRUE(calculateMeans());
    }

    TEST_F(ClustererTest, calculateCovariances)
    {
      fillClusters();
      calculateMeans();
      calculateCovariances();
      Matrix4f cov;
      cov << 0.25, -0.25, 0, -0.833333,
             -0.25, 0.916667, 0, -0.833333,
             0, 0, 0, 0,
             -0.833333, -0.833333, 0, 8.33333;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance1()));
      cov << 0.30000, -0.15000, 0.00000, 0.00000,
             -0.15000, 0.70000, 0.00000, 0.00000,
             0.00000, 0.00000, 0.00000, 0.00000,
             0.00000, 0.00000, 0.00000, 0.00000;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance2()));
    }

    TEST_F(ClustererTest, chooseInitialClusters)
    {
      fillDataSet1();
      chooseInitialClusters();

      MatrixXf matrix;
      matrix.resize(4, 3);
      matrix << 0, 0, 0,
                0, 1, 2,
                0, 0, 0,
                35, 37, 35;
      EXPECT_TRUE(areEquals(matrix, *getCluster1()));
      matrix.resize(4, 6);
      matrix << 1, 1, 1, 2, 2, 2,
                0, 1, 2, 0, 1, 2,
                0, 0, 0, 0, 0, 0,
                20, 20, 20, 20, 20, 18;
      EXPECT_TRUE(areEquals(matrix, *getCluster2()));

      Vector4f mean;
      mean << 0, 1, 0, 35.66667; 
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean1()));
      mean << 1.5, 1, 0, 19.66667;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean2()));

      Matrix4f cov;
      cov << 0.00000, 0.00000, 0.00000, 0.00000,
             0.00000, 1.00000, 0.00000, 0.00000,
             0.00000, 0.00000, 0.00000, 0.00000,
             0.00000, 0.00000, 0.00000, 1.33333;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance1()));
      cov << 0.30000, 0.00000, 0.00000, -0.20000,
             0.00000, 0.80000, 0.00000, -0.40000,
             0.00000, 0.00000, 0.00000, 0.00000,
             -0.20000, -0.40000, 0.00000, 0.66667;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance2()));
    }

    TEST_F(ClustererTest, renewDataSet_firstLegitMeasurement)
    {
      MatrixXf meas;
      fillMeasurement(&meas, 1);
      clusterer_.renewDataSet(meas);
      EXPECT_EQ(meas(2, 0), *currentTime());
      EXPECT_FALSE(*currentExistsInCluster1());
      EXPECT_FALSE(*currentExistsInCluster2());
      EXPECT_TRUE(*readyToCluster());
      EXPECT_EQ(1, *measurementsCounter());
      EXPECT_EQ(dataSet_, getDataSet());
    }

    TEST_F(ClustererTest, renewDataSet_secondLegitMeasurement)
    {
      MatrixXf meas;
      fillDataSet1();
      fillMeasurement(&meas, 2);
      clusterer_.renewDataSet(meas);
      EXPECT_EQ(meas(2, 0), *currentTime());
      EXPECT_FALSE(*currentExistsInCluster1());
      EXPECT_FALSE(*currentExistsInCluster2());
      EXPECT_TRUE(*readyToCluster());
      EXPECT_EQ(2, *measurementsCounter());
      EXPECT_EQ(dataSet_, getDataSet());
    }

    TEST_F(ClustererTest, renewDataSet_thirdLegitMeasurement)
    {
      MatrixXf meas;
      fillDataSet1();
      fillDataSet2();
      fillMeasurement(&meas, 3);
      clusterer_.renewDataSet(meas);
      EXPECT_EQ(meas(2, 0), *currentTime());
      EXPECT_FALSE(*currentExistsInCluster1());
      EXPECT_FALSE(*currentExistsInCluster2());
      EXPECT_TRUE(*readyToCluster());
      EXPECT_EQ(3, *measurementsCounter());
      EXPECT_EQ(dataSet_, getDataSet());
    }

    TEST_F(ClustererTest, renewDataSet_fourthLegitMeasurement)
    {
      MatrixXf meas;
      fillDataSet1();
      fillDataSet2();
      fillDataSet3();
      fillMeasurement(&meas, 4);
      clusterer_.renewDataSet(meas);
      EXPECT_EQ(meas(2, 0), *currentTime());
      EXPECT_FALSE(*currentExistsInCluster1());
      EXPECT_FALSE(*currentExistsInCluster2());
      EXPECT_TRUE(*readyToCluster());
      EXPECT_EQ(4, *measurementsCounter());
      EXPECT_EQ(dataSet_, getDataSet());
    }

    TEST_F(ClustererTest, renewDataSet_flawedSize)
    {
      MatrixXf meas;
      meas.resize(2, 9);
      EXPECT_THROW(clusterer_.renewDataSet(meas), std::range_error);
      meas.resize(4, 24);
      EXPECT_THROW(clusterer_.renewDataSet(meas), std::range_error);
      meas.resize(30, 1);
      EXPECT_THROW(clusterer_.renewDataSet(meas), std::range_error);
    }

    TEST_F(ClustererTest, cluster_oneMeasurement)
    {
      fillDataSet1();
      *currentTime() = 0;
      *currentExistsInCluster1() = false;
      *currentExistsInCluster2() = false;
      *readyToCluster() = true;
      clusterer_.cluster();

      EXPECT_FALSE(*readyToCluster());
      EXPECT_THROW(clusterer_.cluster(), std::logic_error);

      EXPECT_TRUE(*currentExistsInCluster1());
      EXPECT_TRUE(*currentExistsInCluster2());

      MatrixXf matrix;
      matrix.resize(4, 3);
      matrix << 0, 0, 0,
                0, 1, 2,
                0, 0, 0,
                35, 37, 35;
      EXPECT_TRUE(areEquals(matrix, *getCluster1()));
      matrix.resize(4, 6);
      matrix << 1, 1, 1, 2, 2, 2,
                0, 1, 2, 0, 1, 2,
                0, 0, 0, 0, 0, 0,
                20, 20, 20, 20, 20, 18;
      EXPECT_TRUE(areEquals(matrix, *getCluster2()));

      Vector4f mean, currentMean;
      mean << 0, 1, 0, 35.66667; 
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean1()));
      EXPECT_TRUE(clusterer_.getCurrentMean1(&currentMean));
      EXPECT_TRUE(areEquals(mean, currentMean));
      mean << 1.5, 1, 0, 19.66667;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean2()));
      EXPECT_TRUE(clusterer_.getCurrentMean2(&currentMean));
      EXPECT_TRUE(areEquals(mean, currentMean));

      Matrix4f cov;
      cov << 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 1.33333;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance1()));
      cov << 0.30000, 0.00000, 0.00000, -0.20000,
             0.00000, 0.80000, 0.00000, -0.40000,
             0.00000, 0.00000, 0.00000, 0.00000,
             -0.20000, -0.40000, 0.00000, 0.66667;
      EXPECT_TRUE(areEquals(cov, clusterer_.getCovariance2()));
    }
      
    TEST_F(ClustererTest, cluster_threeMeasurements_currentExistsInCluster)
    {
      fillDataSet1();
      fillDataSet2();
      fillDataSet3();
      *currentTime() = 2;
      *currentExistsInCluster1() = false;
      *currentExistsInCluster2() = false;
      *readyToCluster() = true;
      clusterer_.cluster();

      EXPECT_FALSE(*readyToCluster());
      EXPECT_THROW(clusterer_.cluster(), std::logic_error);

      EXPECT_TRUE(*currentExistsInCluster1());
      EXPECT_TRUE(*currentExistsInCluster2());

      Vector4f mean, trueMean;
      mean << 0.1, 0.9, 1.1, 34.2;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean1()));
      mean << 0.25, 0.75, 2, 32.5; 
      EXPECT_TRUE(clusterer_.getCurrentMean1(&trueMean));
      EXPECT_TRUE(areEquals(mean, trueMean));
      mean << 1.52941, 1.05882, 0.94118, 19.88235;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean2()));
      mean << 1.6, 1.2, 2, 20;
      EXPECT_TRUE(clusterer_.getCurrentMean2(&trueMean));
      EXPECT_TRUE(areEquals(mean, trueMean));
    }

    TEST_F(ClustererTest, cluster_threeMeasurements_currentDoesntExistsInCluster)
    {
      fillDataSet1();
      fillDataSet2();
      fillDataSet4();
      *currentTime() = 5;
      *currentExistsInCluster1() = false;
      *currentExistsInCluster2() = false;
      *readyToCluster() = true;
      clusterer_.cluster();

      EXPECT_FALSE(*readyToCluster());
      EXPECT_THROW(clusterer_.cluster(), std::logic_error);

      EXPECT_FALSE(*currentExistsInCluster1());
      EXPECT_TRUE(*currentExistsInCluster2());

      Vector4f mean, trueMean;
      mean << 0, 1, 0.5, 35.3333;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean1()));
      EXPECT_FALSE(clusterer_.getCurrentMean1(&trueMean));
      mean << 1.2857, 1.0000, 2.4286, 20.2143;
      EXPECT_TRUE(areEquals(mean, clusterer_.getMean2()));
      mean << 1, 1, 5, 20.7222;
      EXPECT_TRUE(clusterer_.getCurrentMean2(&trueMean));
      EXPECT_TRUE(areEquals(mean, trueMean));
    }

}  // namespace pandora_sensor_processing

