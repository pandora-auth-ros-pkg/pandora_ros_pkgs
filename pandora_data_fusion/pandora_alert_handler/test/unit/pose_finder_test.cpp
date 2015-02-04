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
 *   Chamzas Konstantinos <chamzask@gmail.com>
 *********************************************************************/

#include <utility>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <pandora_testing_tools/map_loader/map_loader.h>
#include "alert_handler/pose_finder.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class PoseFinderTest
     * @brief Basic Test Fixture for testing PoseFinder
     */
    class PoseFinderTest : public ::testing::Test
    {
      protected:
        void
          testOrientation(float expectedZ, float expectedW, float frameX,
              float frameY, float alertX, float alertY)
          {
            geometry_msgs::Quaternion result, expected;
            expected.x = 0;
            expected.y = 0;
            Point framePoint;
            framePoint.z = 0;
            Point alertPoint;
            alertPoint.z = 0;
            expected.z = expectedZ;
            expected.w = expectedW;
            framePoint.x = frameX;
            framePoint.y = frameY;
            alertPoint.x = alertX;
            alertPoint.y = alertY;
            result = findNormalVectorOnWall(framePoint, alertPoint);
            EXPECT_NEAR(expected.x, result.x, 0.1);
            EXPECT_NEAR(expected.y, result.y, 0.1);
            EXPECT_NEAR(expected.z, result.z, 0.1);
            EXPECT_NEAR(expected.w, result.w, 0.1);
          }


        /* Accessors for private methods of PoseFinder */
        Point
          positionOnWall(Point startPoint, float angle) const
          {
            return poseFinder_->positionOnWall(startPoint, angle);
          }

        float
          calcHeight(float alertPitch, float height, float distFromAlert) const
          {
            return poseFinder_->calcHeight(alertPitch, height, distFromAlert);
          }

        geometry_msgs::Quaternion
          findNormalVectorOnWall(Point framePoint, Point alertPoint) const
          {
            return poseFinder_->findNormalVectorOnWall(framePoint, alertPoint);
          }

        std::pair<Point, Point>
          findDiameterEndPointsOnWall(std::vector<Point> points) const
          {
            return poseFinder_->findDiameterEndPointsOnWall(points);
          }

        float
          getOrientationCircle() const
          {
            return poseFinder_->ORIENTATION_CIRCLE;
          }

        float
          getOrientationDist() const
          {
            return poseFinder_->ORIENTATION_DIST;
          }

        float
          getHeightHighThres() const
          {
            return poseFinder_->HEIGHT_HIGH_THRES;
          }

        float
          getHeightLowThres() const
          {
            return poseFinder_->HEIGHT_LOW_THRES;
          }

        float
          getOccupiedCellThres() const
          {
            return poseFinder_->OCCUPIED_CELL_THRES;
          }

        /* Methods */
        /**
         * @brief Constructor
         */
        PoseFinderTest()
        {
          ros::Time::init();

          map_.reset( new Map );
          *map_ = map_loader::loadMap(
              ros::package::getPath("pandora_alert_handler")
              +"/test/test_maps/map1.yaml");
        }

        /**
         * @brief Function to SetUp the Test Fixture
         * @return void
         */
        virtual void
          SetUp()
          {
            defaultRotation_.setValue(tfScalar(1), tfScalar(0), tfScalar(0),
                tfScalar(0), tfScalar(1), tfScalar(0),
                tfScalar(0), tfScalar(0), tfScalar(1));
            defaultTranslation_.setValue(tfScalar(0), tfScalar(0), tfScalar(0));
            defaultTransform_.setBasis(defaultRotation_);
            defaultTransform_.setOrigin(defaultTranslation_);

            poseFinder_.reset( new PoseFinder(map_, "TEST") );
            poseFinder_->updateParams(0.5, 1.2, 0, 0.5, 0.25);
          }

        /* Variables */
        MapPtr map_;

        tf::Matrix3x3 defaultRotation_;
        tf::Vector3 defaultTranslation_;
        tf::Transform defaultTransform_;

        PoseFinderPtr poseFinder_;
    };

    TEST_F(PoseFinderTest, updateParamsTest)
    {
      // Expect default parameters
      EXPECT_NEAR(0.5, getOccupiedCellThres(), 0.0001);
      EXPECT_NEAR(1.2, getHeightHighThres(), 0.0001);
      EXPECT_NEAR(0, getHeightLowThres(), 0.0001);
      EXPECT_NEAR(0.5, getOrientationDist(), 0.0001);
      EXPECT_NEAR(0.25, getOrientationCircle(), 0.0001);

      poseFinder_->updateParams(0.6, 1.5, 0.3, 1, 0.55);
      // Expect updated parameters
      EXPECT_NEAR(0.6, getOccupiedCellThres(), 0.0001);
      EXPECT_NEAR(1.5, getHeightHighThres(), 0.0001);
      EXPECT_NEAR(0.3, getHeightLowThres(), 0.0001);
      EXPECT_NEAR(1, getOrientationDist(), 0.0001);
      EXPECT_NEAR(0.55, getOrientationCircle(), 0.0001);
    }

    TEST_F(PoseFinderTest, findAlertPoseTest)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 1, 0.5);

      // Make a tfTransform [tf::Transform], check for various yaw [float]
      // and pitches [float] for the expected Pose

      float alertYaw, alertPitch;
      tf::Transform transform;
      Point position;
      geometry_msgs::Quaternion orientation;
      orientation.x = 0;
      orientation.y = 0;
      Pose expected, result;

      alertYaw = -0.785398;
      alertPitch = 0.52360;
      transform = defaultTransform_;
      tf::Vector3 origin(tfScalar(5), tfScalar(5), tfScalar(0.3));
      transform.setOrigin(origin);
      position.x = 5.76;
      position.y = 5.76;
      position.z = 0.9119;
      expected.position = position;
      orientation.z = -0.70711;
      orientation.w = 0.70711;
      expected.orientation = orientation;
      result = poseFinder_->findAlertPose(alertYaw, alertPitch, transform);
      EXPECT_NEAR(expected.position.x, result.position.x, 0.1);
      EXPECT_NEAR(expected.position.y, result.position.y, 0.1);
      EXPECT_NEAR(expected.position.z, result.position.z, 0.0001);
      EXPECT_NEAR(expected.orientation.x, result.orientation.x, 0.0001);
      EXPECT_NEAR(expected.orientation.y, result.orientation.y, 0.0001);
      EXPECT_NEAR(expected.orientation.z, result.orientation.z, 0.1);
      EXPECT_NEAR(expected.orientation.w, result.orientation.w, 0.1);
    }

    TEST_F(PoseFinderTest, positionOnWallTest)
    {
      poseFinder_->updateParams(0.3, 1.5, 0, 20, 10);

      // With given map [OccupancyGrid], make points [Point] and test
      // their supposed positions on wall with various angles [float]

      Point startPoint, expected, result;
      startPoint.z = 0;
      expected.z = 0;
      float angle;

      startPoint.x = 8;
      startPoint.y = 9;
      angle = 0;
      expected.x = 10.48;
      expected.y = 9;
      ROS_INFO_STREAM_NAMED("pose_finder_test", "PositionOnWallTest 1: x = "
          << startPoint.x << " y = " << startPoint.y << " angle = " << angle);
      result = positionOnWall(startPoint, angle);
      EXPECT_NEAR(expected.x, result.x, 0.01);
      EXPECT_NEAR(expected.y, result.y, 0.01);

      startPoint.x = 5;
      startPoint.y = 5;
      angle = 0.785398;
      expected.x = 5.7495;
      expected.y = 5.7495;
      ROS_INFO_STREAM_NAMED("pose_finder_test", "PositionOnWallTest 2: x = "
          << startPoint.x << " y = " << startPoint.y << " angle = " << angle);
      result = positionOnWall(startPoint, angle);
      EXPECT_NEAR(expected.x, result.x, 0.01);
      EXPECT_NEAR(expected.y, result.y, 0.01);

      startPoint.x = 1;
      startPoint.y = 3;
      angle = 0;
      expected.x = 2.62;
      expected.y = 3;
      ROS_INFO_STREAM_NAMED("pose_finder_test", "PositionOnWallTest 3: x = "
          << startPoint.x << " y = " << startPoint.y << " angle = " << angle);
      result = positionOnWall(startPoint, angle);
      EXPECT_NEAR(expected.x, result.x, 0.01);
      EXPECT_NEAR(expected.y, result.y, 0.01);
    }

    TEST_F(PoseFinderTest, calcHeightTest)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 20, 10);

      // Given the height [float] of the robot coord. frame origin, check for
      // various distances from wall (distFromAlert) [float] and pitches [float]
      // the corresponding alert height on wall

      float h = 0.3;
      // EXPECT_THROW( calcHeight(1.04720, h, -1) , AlertException );
      EXPECT_NEAR(0.30000, calcHeight(1.04720, h, 0), 0.0001);
      EXPECT_THROW(calcHeight(1.04720, h, 1) , AlertException);
      EXPECT_THROW(calcHeight(1.04720, h, 1.5), AlertException);
      EXPECT_THROW(calcHeight(1.04720, h, 2), AlertException);

      // EXPECT_THROW( calcHeight(0.52360, h, -1) , AlertException );
      EXPECT_NEAR(0.30000, calcHeight(0.52360, h, 0), 0.0001);
      EXPECT_NEAR(0.87735, calcHeight(0.52360, h, 1), 0.0001);
      EXPECT_NEAR(1.16603, calcHeight(0.52360, h, 1.5), 0.0001);
      EXPECT_NEAR(1.45470, calcHeight(0.52360, h, 2), 0.0001);

      // EXPECT_THROW( calcHeight(-0.52360, h, -1) , AlertException );
      EXPECT_NEAR(0.30000, calcHeight(-0.52360, h, 0), 0.0001);
      EXPECT_THROW(calcHeight(-0.52360, h, 1), AlertException);
      EXPECT_THROW(calcHeight(-0.52360, h, 1.5), AlertException);
      EXPECT_THROW(calcHeight(-0.52360, h, 2), AlertException);

      h = 0.5;
      // EXPECT_THROW( calcHeight(-0.26180, h, -1) , AlertException );
      EXPECT_NEAR(0.500000, calcHeight(-0.26180, h, 0), 0.0001);
      EXPECT_NEAR(0.232051, calcHeight(-0.26180, h, 1), 0.0001);
      EXPECT_NEAR(0.098076, calcHeight(-0.26180, h, 1.5), 0.0001);
      EXPECT_THROW(calcHeight(-0.26180, h, 2), AlertException);
    }

    TEST_F(PoseFinderTest, findNormalVectorOnWallTest1)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.25);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      testOrientation(-0.70711, 0.70711, 5, 5, 5.76, 5.76);
    }

    TEST_F(PoseFinderTest, findNormalVectorOnWallTest2)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.25);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      testOrientation(1, 0, 7, 1, 7.86, 1);
    }

    TEST_F(PoseFinderTest, findNormalVectorOnWallTest3)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.25);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      testOrientation(1, 0, 10.0, 12.3, 10.5, 12.6);
    }

    TEST_F(PoseFinderTest, findNormalVectorOnWallTest4)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.25);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      testOrientation(0, 1, 12, 11, 11.5, 11);
    }


    TEST_F(PoseFinderTest, findNormalVectorOnWallTest5)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.15);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      testOrientation(0.70171 , 0.70711, 4.14, 4.7, 3, 3.58);
    }

    TEST_F(PoseFinderTest, findNormalVectorTestOnCorners1)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.2);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      // Near corner (inside)

      float expectedYaw = 0;
      geometry_msgs::Quaternion result;
      Point framePoint;
      framePoint.z = 0;
      Point alertPoint;
      alertPoint.z = 0;
      framePoint.x = 5.25;
      framePoint.y = 8.33;
      alertPoint.x = 4.85;
      alertPoint.y = 7.78;
      result = findNormalVectorOnWall(framePoint, alertPoint);
      expectedYaw = acos(result.w);
      if (asin(result.z) < 0)
        expectedYaw = -expectedYaw;
      expectedYaw = 2*expectedYaw*180/PI;
      EXPECT_NEAR(0.0, result.x, 0.001);
      EXPECT_NEAR(0.0, result.y, 0.001);
      EXPECT_LT(expectedYaw, 70);
      EXPECT_GT(expectedYaw, 20);
    }

    TEST_F(PoseFinderTest, findNormalVectorTestOnCorners2)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 0.2);
      // Test if the returned normal vector on wall is right
      // [geometry_msgs::Quaternion] with the given map [OccupancyGrid]
      // and various frame points [Point] and alert points [Point]

      // Near corner (outside)

      float expectedYaw = 0;
      geometry_msgs::Quaternion result;
      Point framePoint;
      framePoint.z = 0;
      Point alertPoint;
      alertPoint.z = 0;
      framePoint.x = 4.7;
      framePoint.y = 7.34;
      alertPoint.x = 4.85;
      alertPoint.y = 7.66;
      result = findNormalVectorOnWall(framePoint, alertPoint);
      expectedYaw = acos(result.w);
      if (asin(result.z) < 0)
        expectedYaw = -expectedYaw;
      expectedYaw = 2*expectedYaw*180/PI;
      EXPECT_NEAR(0.0, result.x, 0.001);
      EXPECT_NEAR(0.0, result.y, 0.001);
      EXPECT_LT(expectedYaw, -110);
      EXPECT_GT(expectedYaw, -160);
    }

    TEST_F(PoseFinderTest, findDiameterEndPointsOnWallTest)
    {
      poseFinder_->updateParams(0.5, 1.5, 0, 20, 10);

      // Given various vectors of points [std::vector<Point>] test if the
      // largest distance between them id given by the two points
      // [std::pair<Point, Point>] returned.

      std::vector<Point> points;
      std::pair<Point, Point> result;

      points.clear();
      EXPECT_THROW(findDiameterEndPointsOnWall(points), AlertException);

      Point first;
      first.x = 0;
      first.y = 0;
      first.z = 0;
      points.push_back(first);
      EXPECT_THROW(findDiameterEndPointsOnWall(points), AlertException);

      Point second;
      second.x = 1;
      second.y = 0;
      second.z = 0;
      points.push_back(second);
      result = findDiameterEndPointsOnWall(points);
      EXPECT_NEAR(first.x, result.first.x, 0.0001);
      EXPECT_NEAR(first.y, result.first.y, 0.0001);
      EXPECT_NEAR(first.z, result.first.z, 0.0001);
      EXPECT_NEAR(second.x, result.second.x, 0.0001);
      EXPECT_NEAR(second.y, result.second.y, 0.0001);
      EXPECT_NEAR(second.z, result.second.z, 0.0001);

      Point third;
      third.x = 0;
      third.y = 0;
      third.z = 0;
      points.push_back(third);
      result = findDiameterEndPointsOnWall(points);
      EXPECT_NEAR(first.x, result.first.x, 0.0001);
      EXPECT_NEAR(first.y, result.first.y, 0.0001);
      EXPECT_NEAR(first.z, result.first.z, 0.0001);
      EXPECT_NEAR(second.x, result.second.x, 0.0001);
      EXPECT_NEAR(second.y, result.second.y, 0.0001);
      EXPECT_NEAR(second.z, result.second.z, 0.0001);

      Point fourth;
      fourth.x = 0;
      fourth.y = -1;
      fourth.z = 0;
      points.push_back(fourth);
      result = findDiameterEndPointsOnWall(points);
      EXPECT_NEAR(second.x, result.first.x, 0.0001);
      EXPECT_NEAR(second.y, result.first.y, 0.0001);
      EXPECT_NEAR(second.z, result.first.z, 0.0001);
      EXPECT_NEAR(fourth.x, result.second.x, 0.0001);
      EXPECT_NEAR(fourth.y, result.second.y, 0.0001);
      EXPECT_NEAR(fourth.z, result.second.z, 0.0001);

      Point fifth;
      fifth.x = 41;
      fifth.y = 2.42;
      fifth.z = -12;
      points.push_back(fifth);
      result = findDiameterEndPointsOnWall(points);
      EXPECT_NEAR(fourth.x, result.first.x, 0.0001);
      EXPECT_NEAR(fourth.y, result.first.y, 0.0001);
      EXPECT_NEAR(fourth.z, result.first.z, 0.0001);
      EXPECT_NEAR(fifth.x, result.second.x, 0.0001);
      EXPECT_NEAR(fifth.y, result.second.y, 0.0001);
      EXPECT_NEAR(fifth.z, result.second.z, 0.0001);

      Point sixth;
      sixth.x = 10000;
      sixth.y = 10000;
      sixth.z = 10000;
      points.push_back(sixth);
      result = findDiameterEndPointsOnWall(points);
      EXPECT_NEAR(fourth.x, result.first.x, 0.0001);
      EXPECT_NEAR(fourth.y, result.first.y, 0.0001);
      EXPECT_NEAR(fourth.z, result.first.z, 0.0001);
      EXPECT_NEAR(sixth.x, result.second.x, 0.0001);
      EXPECT_NEAR(sixth.y, result.second.y, 0.0001);
      EXPECT_NEAR(sixth.z, result.second.z, 0.0001);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

