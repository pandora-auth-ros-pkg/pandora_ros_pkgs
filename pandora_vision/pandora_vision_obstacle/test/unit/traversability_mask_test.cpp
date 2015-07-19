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
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *  Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#include <vector>
#include <limits>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/hard_obstacle_detection/RobotGeometryMaskDescription.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class TraversabilityMaskTest: public ::testing::Test
  {
    public:
      TraversabilityMaskTest()    {}

      typedef TraversabilityMask::MatPtr MatPtr;

      virtual void SetUp()
      {
        // Set the robot dimensions.
        descriptionPtr_.reset(new RobotGeometryMaskDescription);
        setRobotDescription();

        traversabilityMaskPtr_.reset(new TraversabilityMask(descriptionPtr_));

        wheelSize_ = traversabilityMaskPtr_->metersToSteps(descriptionPtr_->wheelD);
        robotSize_ = traversabilityMaskPtr_->metersToSteps(descriptionPtr_->robotD);
        barrelSize_ = traversabilityMaskPtr_->metersToSteps(descriptionPtr_->barrelD);
        totalSize_ = traversabilityMaskPtr_->metersToSteps(descriptionPtr_->totalD);

        updatedElevationMaskPtr_.reset(new cv::Mat(traversabilityMaskPtr_->robotGeometryMask_->size(), CV_64FC1));

        *updatedElevationMaskPtr_ = traversabilityMaskPtr_->robotGeometryMask_->clone();
      }

      void setRobotDescription()
      {
        descriptionPtr_->wheelH = 0.0;
        descriptionPtr_->barrelH = 0.067;
        descriptionPtr_->robotH = 0.134;
        descriptionPtr_->wheelD = 0.0742;
        descriptionPtr_->barrelD = 0.075;
        descriptionPtr_->robotD = 0.08;
        descriptionPtr_->totalD = descriptionPtr_->wheelD + 2 * descriptionPtr_->barrelD
          + descriptionPtr_->robotD;
        descriptionPtr_->maxPossibleAngle = 20;
        descriptionPtr_->RESOLUTION = 0.02;
      }

      void createUniformElevationMap(const MatPtr& elevationMapPtr, int width, int height, double elevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        elevationMapPtr->setTo(elevation);
        return;
      }

      void createLinearHorizontalElMap(const MatPtr& elevationMapPtr, int width, int height,
          double minElevation, double maxElevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        double slope = tan((maxElevation - minElevation) / width);
        for (int j = 0; j < width; ++j)
        {
          double val = slope * j + minElevation;
          for (int i = 0; i < height; ++i)
          {
            elevationMapPtr->at<double>(i, j) = val;
          }
        }
        return;
      }

      void createLinearVerticalElMap(const MatPtr& elevationMapPtr, int width, int height,
          double minElevation, double maxElevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        double slope = tan((maxElevation - minElevation) / height);

        for (int i = 0; i < height; ++i)
        {
          double val = slope * i + minElevation;

          for (int j = 0; j < width; ++j)
          {
            elevationMapPtr->at<double>(i, j) = val;
          }
        }
        return;
      }

      bool findElevatedLeftRight(MatPtr al, double hForward, double hBack, double d)
      {
        return traversabilityMaskPtr_->findElevatedLeftRight(al, hForward, hBack, d);
      }

      bool findElevatedTopBottom(MatPtr al, double hForward, double hBack, double d)
      {
        return traversabilityMaskPtr_->findElevatedTopBottom(al, hForward, hBack, d);
      }

      bool cropToWheel(const cv::Point& wheelPos, const MatPtr& wheel)
      {
        return traversabilityMaskPtr_->cropToWheel(wheelPos, wheel);
      }

      bool findHeightOnWheel(const cv::Point& wheelPos, double* meanHeight, double* stdDevHeight)
      {
        return traversabilityMaskPtr_->findHeightOnWheel(wheelPos, meanHeight, stdDevHeight);
      }

      inline double getMaskValue(int i, int j)
      {
        return traversabilityMaskPtr_->robotGeometryMask_->at<double>(i, j);
      }

      void createMaskFromDesc()
      {
        traversabilityMaskPtr_->createMaskFromDesc();
      }

      MatPtr getRobotMask()
      {
        return traversabilityMaskPtr_->robotGeometryMask_;
      }

      void
      setElevationMap(const boost::shared_ptr<cv::Mat const>& elevationMapPtr)
      {
        traversabilityMaskPtr_->elevationMapPtr_ = elevationMapPtr;
      }

      boost::shared_ptr<cv::Mat const> getElevationMap()
      {
        return traversabilityMaskPtr_->elevationMapPtr_;
      }

      int8_t findTraversability(const cv::Point& center)
      {
        return traversabilityMaskPtr_->findTraversability(center);
      }


      virtual ~TraversabilityMaskTest()
      {}

    protected:
      boost::shared_ptr<TraversabilityMask> traversabilityMaskPtr_;
      MatPtr updatedElevationMaskPtr_;
      boost::shared_ptr<RobotGeometryMaskDescription> descriptionPtr_;
      int wheelSize_;
      int robotSize_;
      int barrelSize_;
      int totalSize_;
  };

  TEST_F(TraversabilityMaskTest, ExtractWheelAreaTest)
  {
    MatPtr elevationMapPtr(new cv::Mat);
    int width = 300;
    int height = 300;

    createUniformElevationMap(elevationMapPtr, width, height, 0);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    traversabilityMaskPtr_->getRobotMaskPtr()->copyTo(*updatedElevationMaskPtr_);
    MatPtr wheelElevationPtr(new cv::Mat(wheelSize_, wheelSize_, CV_64FC1));
    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for height = !"
              << 0;
          }
        }
      }
    }

    createUniformElevationMap(elevationMapPtr, width, height, 0.3);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for height = !"
              << 0.3;
          }
        }
      }
    }
    createUniformElevationMap(elevationMapPtr, width, height, 0.5);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for height = !"
              << 0.5;
          }
        }
      }
    }
    createUniformElevationMap(elevationMapPtr, width, height, 0.75);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for height = !"
              << 0.75;
          }
        }
      }
    }

    createLinearHorizontalElMap(elevationMapPtr, width, height, 0, width);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for "
              << "Horizontal Linear Elevation Map (minHeight, maxHeight) = ( " << 0 << "," << width << " )";
          }
        }
      }
    }
    createLinearHorizontalElMap(elevationMapPtr, width, height, width / 2.0, width);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(i + k, j + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for "
              << "Horizontal Linear Elevation Map (minHeight, maxHeight) = ( " << 0 << "," << width / 2.0 << " )";
          }
        }
      }
    }
    createLinearVerticalElMap(elevationMapPtr, width, height, 0, height);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(j + k, i + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for "
              << "Vertical Linear Elevation Map (minHeight, maxHeight) = ( " << 0 << "," << height << " )";
          }
        }
      }
    }
    createLinearVerticalElMap(elevationMapPtr, width, height, height / 2.0, height);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = 0; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        for (int k = 0; k < wheelSize_; ++k)
        {
          for (int ii = 0; ii < wheelSize_; ++ii)
          {
            ASSERT_NEAR(wheelElevationPtr->at<double>(k, ii),
                elevationMapPtr->at<double>(j + k, i + ii), 0.1)
              << " Values for Wheel Position (i, j) = " << i << " , " << j << " are not equal for "
              << "Horizontal Linear Elevation Map (minHeight, maxHeight) = ( " << 0 << "," << height / 2.0 << " )";
          }
        }
      }
    }
  }

  TEST_F(TraversabilityMaskTest, WheelHeightCalculationTest)
  {
    MatPtr elevationMapPtr(new cv::Mat);
    int width = 300;
    int height = 300;

    double meanHeight, stdDevHeight;

    createUniformElevationMap(elevationMapPtr, width, height, 0);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);
    traversabilityMaskPtr_->getRobotMaskPtr()->copyTo(*updatedElevationMaskPtr_);

    MatPtr wheelElevationPtr(new cv::Mat(wheelSize_, wheelSize_, CV_64FC1));
    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for uniform elevation Map" <<
          " with height = 0.0";
        ASSERT_NEAR(meanHeight, 0, 0.1);
        cv::Mat squaredElevation;
        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }

    createUniformElevationMap(elevationMapPtr, width, height, 0.3);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for uniform elevation Map" <<
          " with height = 0.3";
        ASSERT_NEAR(meanHeight, 0.3, 0.1);
        cv::Mat squaredElevation;
        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }
    createUniformElevationMap(elevationMapPtr, width, height, 0.5);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for uniform elevation Map" <<
          " with height = 0.5";
        ASSERT_NEAR(meanHeight, 0.5, 0.1);
        cv::Mat squaredElevation;
        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }
    createUniformElevationMap(elevationMapPtr, width, height, 0.75);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for uniform elevation Map" <<
          " with height = 0.75";
        ASSERT_NEAR(meanHeight, 0.75, 0.1);
        cv::Mat squaredElevation;
        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }

    createLinearHorizontalElMap(elevationMapPtr, width, height, 0, width);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for horizontal linear Elevation" <<
          " Map with (minHeight, maxHeight) = ( " << 0 << " , " << width << " )";
        ASSERT_NEAR(meanHeight, cv::sum(*wheelElevationPtr)[0] / (wheelSize_ * wheelSize_), 0.1);
        cv::Mat squaredElevation;

        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }
    createLinearHorizontalElMap(elevationMapPtr, width, height, width / 2.0, width);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for horizontal linear Elevation" <<
          " Map with (minHeight, maxHeight) = ( " << width / 2.0 << " , " << width << " )";
        ASSERT_NEAR(meanHeight, cv::sum(*wheelElevationPtr)[0] / (wheelSize_ * wheelSize_), 0.1);
        cv::Mat squaredElevation;

        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }
    createLinearVerticalElMap(elevationMapPtr, width, height, 0, height);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);

    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for horizontal linear Elevation" <<
          " Map with (minHeight, maxHeight) = ( " << 0 << " , " << height << " )";
        ASSERT_NEAR(meanHeight, cv::sum(*wheelElevationPtr)[0] / (wheelSize_ * wheelSize_), 0.1);
        cv::Mat squaredElevation;

        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }

    createLinearVerticalElMap(elevationMapPtr, width, height, height / 2.0, height);
    traversabilityMaskPtr_->setElevationMap(elevationMapPtr);
    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_TRUE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << " for horizontal linear Elevation" <<
          " Map with (minHeight, maxHeight) = ( " << height / 2.0 << " , " << height << " )";
        ASSERT_NEAR(meanHeight, cv::sum(*wheelElevationPtr)[0] / (wheelSize_ * wheelSize_), 0.1);
        cv::Mat squaredElevation;

        cv::pow(*wheelElevationPtr - meanHeight, 2, squaredElevation);
        ASSERT_NEAR(stdDevHeight,
            sqrt(1.0 / (wheelSize_ * wheelSize_ - 1) * cv::sum(squaredElevation)[0]) , 0.1);
      }
    }

    elevationMapPtr->setTo(- std::numeric_limits<double>::max());
    for (int i = wheelSize_; i < elevationMapPtr->rows - wheelSize_; ++i)
    {
      for (int j = wheelSize_; j < elevationMapPtr->cols - wheelSize_; ++j)
      {
        cv::Point wheelPos(j, i);
        TraversabilityMaskTest::cropToWheel(wheelPos, wheelElevationPtr);
        ASSERT_FALSE(TraversabilityMaskTest::findHeightOnWheel(wheelPos, &meanHeight, &stdDevHeight))
          << " Failure at (i, j) = " << i << " , " << j << " )" << ". Function did not return "
          <<  " at invalid height values!";
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedLeftHorizontal)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.0;

    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_TRUE(allowedAngle);
    for (int ii = 0; ii < tempMapPtr->rows; ++ii)
    {
      for (int jj = 0; jj < tempMapPtr->cols; ++jj)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + ii, bBoxX + jj),
                    getMaskValue(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedRightHorizontal)
  {
    int bBoxX = updatedElevationMaskPtr_->cols - wheelSize_;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.0;

    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_TRUE(allowedAngle);
    for (int ii = 0; ii < tempMapPtr->rows; ++ii)
    {
      for (int jj = 0; jj < tempMapPtr->cols; ++jj)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + ii, bBoxX + jj),
                    getMaskValue(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedTopHorizontal)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = updatedElevationMaskPtr_->cols;
    int bBoxHeight = wheelSize_;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.0;

    bool allowedAngle = findElevatedTopBottom(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->cols, tempMapPtr->cols);
    ASSERT_EQ(wheelSize_, tempMapPtr->rows);
    ASSERT_TRUE(allowedAngle);
    for (int ii = 0; ii < tempMapPtr->rows; ++ii)
    {
      for (int jj = 0; jj < tempMapPtr->cols; ++jj)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + ii, bBoxX + jj),
                    getMaskValue(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedBottomHorizontal)
  {
    int bBoxX = 0;
    int bBoxY = updatedElevationMaskPtr_->rows - wheelSize_;
    int bBoxWidth = updatedElevationMaskPtr_->cols;
    int bBoxHeight = wheelSize_;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.0;

    bool allowedAngle = findElevatedTopBottom(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->cols, tempMapPtr->cols);
    ASSERT_EQ(wheelSize_, tempMapPtr->rows);
    ASSERT_TRUE(allowedAngle);
    for (int ii = 0; ii < tempMapPtr->rows; ++ii)
    {
      for (int jj = 0; jj < tempMapPtr->cols; ++jj)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + ii, bBoxX + jj),
                    getMaskValue(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedLeftSlopeBackWheelHigher)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.1;

    double slope = fabs(firstWheelMeanHeight - secondWheelMeanHeight) /
                   (descriptionPtr_->totalD);
    double slopeResolution = 0.0;
    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_TRUE(allowedAngle);
    for (int jj = wheelSize_; jj < tempMapPtr->rows - wheelSize_; ++jj)
    {
      slopeResolution = jj * fabs(firstWheelMeanHeight - secondWheelMeanHeight) / tempMapPtr->rows;
      for (int ii = 0; ii < tempMapPtr->cols; ++ii)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + jj, bBoxX + ii),
                    getMaskValue(jj, ii) + slopeResolution, 1e-6)
                    << "Failure at (" << jj << ", " << ii << ")";
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedLeftSlopeBackWheelHigherHighAngle)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.0;
    double secondWheelMeanHeight = 0.3;

    double slope = fabs(firstWheelMeanHeight - secondWheelMeanHeight) /
                   (descriptionPtr_->totalD);
    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_FALSE(allowedAngle);
    for (int jj = wheelSize_; jj < tempMapPtr->rows - wheelSize_; ++jj)
    {
      for (int ii = 0; ii < tempMapPtr->cols; ++ii)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + jj, bBoxX + ii),
                    getMaskValue(jj, ii), 1e-6)
                    << "Failure at (" << jj << ", " << ii << ")";
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedLeftSlopeFrontWheelHigher)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.1;
    double secondWheelMeanHeight = 0.0;

    double slope = fabs(firstWheelMeanHeight - secondWheelMeanHeight) /
                   descriptionPtr_->totalD;
    double slopeResolution = 0.0;
    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_TRUE(allowedAngle);
    for (int jj = wheelSize_; jj < tempMapPtr->rows - wheelSize_; ++jj)
    {
      slopeResolution = fabs(firstWheelMeanHeight - secondWheelMeanHeight) * jj / tempMapPtr->rows;
      std::cout << slopeResolution << std::endl;
      for (int ii = 0; ii < tempMapPtr->cols; ++ii)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + totalSize_ - jj, bBoxX + ii),
                    getMaskValue(jj, ii) + slopeResolution, 1e-6)
                    << "Failure at (" << jj << ", " << ii << ")";
      }
    }
  }

  TEST_F(TraversabilityMaskTest, findElevatedLeftSlopeFrontWheelHigherHighAngle)
  {
    int bBoxX = 0;
    int bBoxY = 0;
    int bBoxWidth = wheelSize_;
    int bBoxHeight = updatedElevationMaskPtr_->rows;
    MatPtr tempMapPtr(new cv::Mat(*updatedElevationMaskPtr_,
        cv::Rect(bBoxX, bBoxY, bBoxWidth, bBoxHeight)));

    double firstWheelMeanHeight = 0.3;
    double secondWheelMeanHeight = 0.0;

    double slope = fabs(firstWheelMeanHeight - secondWheelMeanHeight) /
                   descriptionPtr_->totalD;
    bool allowedAngle = findElevatedLeftRight(tempMapPtr, firstWheelMeanHeight,
        secondWheelMeanHeight, descriptionPtr_->totalD);

    ASSERT_EQ(updatedElevationMaskPtr_->rows, tempMapPtr->rows);
    ASSERT_EQ(wheelSize_, tempMapPtr->cols);
    ASSERT_FALSE(allowedAngle);
    for (int jj = wheelSize_; jj < tempMapPtr->rows - wheelSize_; ++jj)
    {
      for (int ii = 0; ii < tempMapPtr->cols; ++ii)
      {
        EXPECT_NEAR(updatedElevationMaskPtr_->at<double>(bBoxY + totalSize_ - jj, bBoxX + ii),
                    getMaskValue(jj, ii), 1e-6)
                    << "Failure at (" << jj << ", " << ii << ")";
      }
    }
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

