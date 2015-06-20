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

#include "gtest/gtest.h"

#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class ObjectsTest : public ::testing::Test
    {
      protected:
        ObjectsTest()
        {
          qr1_.reset( new Qr );
          qr2_.reset( new Qr );
          hazmat1_.reset( new Hazmat );
          hazmat2_.reset( new Hazmat );
          hole1_.reset( new Hole );
          hole2_.reset( new Hole );
          thermal1_.reset( new Thermal );
          thermal2_.reset( new Thermal );
        }

        // Seting up all the different objects that will be used
        // for our test cases. The distance between the points
        // (0,0,0), (4,3,0) is 5.
        virtual void SetUp()
        {
          pose1_.position.x = 0;
          pose1_.position.y = 0;
          pose1_.position.z = 0;
          pose2_.position.x = 4;
          pose2_.position.y = 3;
          pose2_.position.z = 0;
          EXPECT_EQ(5, distance(pose1_, pose2_));

          Qr::setObjectType("QR");
          Qr::setDistanceThres(6);
          Qr::getFilterModel()->initializeMeasurementModel(5);
          Qr::setProbabilityThres(0.80);
          qr1_->setPose(pose1_);
          qr1_->setId(1);
          qr1_->setProbability(0.5);
          qr1_->initializeObjectFilter();

          //!< Check that initializeObjectFilter works as intended!
          ASSERT_FLOAT_EQ(Utils::stdDevFromProbability(
                qr1_->getDistanceThres(), 0.5), qr1_->getStdDevX());

          qr2_->setPose(pose2_);
          qr2_->setId(2);
          qr2_->setProbability(0.5);
          qr2_->initializeObjectFilter();

          Hazmat::setObjectType("HAZMAT");
          Hazmat::setDistanceThres(6);
          Hazmat::getFilterModel()->initializeMeasurementModel(5);
          Hazmat::setProbabilityThres(0.75);
          hazmat1_->setPose(pose1_);
          hazmat1_->setId(1);
          hazmat1_->setProbability(0.5);
          hazmat1_->initializeObjectFilter();
          hazmat2_->setPose(pose2_);
          hazmat2_->setId(2);
          hazmat2_->setProbability(0.5);
          hazmat2_->initializeObjectFilter();

          Hole::setObjectType("HOLE");
          Hole::setDistanceThres(6);
          Hole::getFilterModel()->initializeMeasurementModel(5);
          Hole::setProbabilityThres(0.75);
          hole1_->setPose(pose1_);
          hole1_->setId(1);
          hole1_->setProbability(0.5);
          hole1_->initializeObjectFilter();
          hole2_->setPose(pose2_);
          hole2_->setId(2);
          hole2_->setProbability(0.5);
          hole2_->initializeObjectFilter();

          Thermal::setObjectType("THERMAL");
          Thermal::setDistanceThres(6);
          Thermal::getFilterModel()->initializeMeasurementModel(5);
          Thermal::setProbabilityThres(0.75);
          thermal1_->setPose(pose1_);
          thermal1_->setId(2);
          thermal1_->setProbability(0.5);
          thermal1_->initializeObjectFilter();
          thermal2_->setPose(pose2_);
          thermal2_->setId(2);
          thermal2_->setProbability(0.5);
          thermal2_->initializeObjectFilter();
        }

        /* Helper functions */

        float distance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
        {
          float x = pose1.position.x - pose2.position.x;
          float y = pose1.position.y - pose2.position.y;
          float z = pose1.position.z - pose2.position.z;
          return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
        }

        /* Variables */

        geometry_msgs::geometry_msgs::Pose pose1_;
        geometry_msgs::geometry_msgs::Pose pose2_;
        QrPtr qr1_;
        QrPtr qr2_;
        HazmatPtr hazmat1_;
        HazmatPtr hazmat2_;
        HolePtr hole1_;
        HolePtr hole2_;
        ThermalPtr thermal1_;
        ThermalPtr thermal2_;
    };

    TEST_F(ObjectsTest, QrConstrustor)
    {
      EXPECT_FALSE(qr1_->getLegit());
      EXPECT_EQ(Qr::getObjectType(), qr1_->getType());
    }

    TEST_F(ObjectsTest, HazmatConstrustor)
    {
      EXPECT_FALSE(hazmat1_->getLegit());
      EXPECT_EQ(Hazmat::getObjectType(), hazmat1_->getType());
    }

    TEST_F(ObjectsTest, HoleConstrustor)
    {
      EXPECT_FALSE(hole1_->getLegit());
      EXPECT_EQ(Hole::getObjectType(), hole1_->getType());
    }

    TEST_F(ObjectsTest, ThermalConstrustor)
    {
      EXPECT_FALSE(thermal1_->getLegit());
      EXPECT_EQ(Thermal::getObjectType(), thermal1_->getType());
    }

    TEST_F(ObjectsTest, isSameObject)
    {
      EXPECT_FALSE(qr1_->isSameObject(hazmat1_));
      EXPECT_FALSE(hazmat1_->isSameObject(hole1_));
      EXPECT_FALSE(hole1_->isSameObject(thermal1_));
      EXPECT_FALSE(thermal1_->isSameObject(qr1_));

      EXPECT_TRUE(thermal1_->isSameObject(thermal2_));
      EXPECT_TRUE(thermal2_->isSameObject(thermal1_));

      EXPECT_TRUE(hole1_->isSameObject(hole2_));
    }

    TEST_F(ObjectsTest, isSameQr)
    {
      qr1_->setContent("Voula");
      qr2_->setContent("Voula");
      EXPECT_TRUE(qr1_->isSameObject(qr2_));
      Qr::setDistanceThres(4);
      EXPECT_FALSE(qr1_->isSameObject(qr2_));
      Qr::setDistanceThres(6);
      qr2_->setContent("Vicky");
      EXPECT_FALSE(qr1_->isSameObject(qr2_));
    }

    TEST_F(ObjectsTest, isSameHazmat)
    {
      hazmat1_->setPattern(1);
      hazmat2_->setPattern(1);
      EXPECT_TRUE(hazmat1_->isSameObject(hazmat2_));
      Hazmat::setDistanceThres(4);
      EXPECT_FALSE(hazmat1_->isSameObject(hazmat2_));
      Hazmat::setDistanceThres(6);
      hazmat2_->setPattern(4);
      EXPECT_FALSE(hazmat1_->isSameObject(hazmat2_));
    }

    TEST_F(ObjectsTest, checkLegit)
    {
      qr1_->setProbability(0.8);
      EXPECT_FALSE(qr1_->getLegit());
      qr1_->checkLegit();
      EXPECT_TRUE(qr1_->getLegit());
    }

    TEST_F(ObjectsTest, update)
    {
      ASSERT_EQ(0.5, qr1_->getProbability());
      ASSERT_GE(qr2_->getProbability(), 0.5);
      float probabilityBefore = qr1_->getProbability();
      float stdDevBefore = qr1_->getStdDevX();
      geometry_msgs::Pose poseBefore = qr1_->getPose();
      EXPECT_FALSE(qr1_->getLegit());
      qr1_->update(qr2_);
      EXPECT_FALSE(qr1_->getLegit());
      EXPECT_GT(qr1_->getProbability(), probabilityBefore);
      EXPECT_LT(qr1_->getStdDevX(), stdDevBefore);
      EXPECT_LT(distance(qr1_->getPose(), qr2_->getPose()),
          distance(poseBefore, qr2_->getPose()));

      probabilityBefore = qr1_->getProbability();
      stdDevBefore = qr1_->getStdDevX();
      poseBefore = qr1_->getPose();
      qr1_->update(qr2_);
      EXPECT_TRUE(qr1_->getLegit());
      EXPECT_GT(qr1_->getProbability(), probabilityBefore);
      EXPECT_LT(qr1_->getStdDevX(), stdDevBefore);
      EXPECT_LT(distance(qr1_->getPose(), qr2_->getPose()),
          distance(poseBefore, qr2_->getPose()));
    }

    // TEST_F(ObjectsTest, updateWithZeroProbability)
    // {
    //   qr2_->setProbability(0);
    //   qr2_->initializeObjectFilter();
    //   float probabilityBefore = qr1_->getProbability();
    //   float stdDevBefore = qr1_->getStdDevX();
    //   geometry_msgs::Pose poseBefore = qr1_->getPose();
    //   qr1_->update(qr2_);
    //   EXPECT_FALSE(qr1_->getLegit());
    //   EXPECT_LT(qr1_->getProbability(), probabilityBefore);
    //   EXPECT_GT(qr1_->getStdDevX(), stdDevBefore);
    //   EXPECT_LT(distance(qr1_->getPose(), poseBefore), 0.6);
    // }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

