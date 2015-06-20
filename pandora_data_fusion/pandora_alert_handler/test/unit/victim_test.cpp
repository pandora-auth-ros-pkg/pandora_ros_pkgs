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

#include "alert_handler/victim.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class VictimTest : public ::testing::Test
    {
      protected:
        VictimTest()
        {
          ros::Time::init();
          victim1_.reset( new Victim );
          victim2_.reset( new Victim );
          objConstPtrVect1_.reset( new ObjectConstPtrVector );
          objConstPtrVect2_.reset( new ObjectConstPtrVector );
          objConstPtrVect3_.reset( new ObjectConstPtrVector );
        }

        virtual void SetUp()
        {
          Hole::setObjectType("hole");
          Thermal::setObjectType("thermal");
          //!< create the Objectvector1 and fill it with various objects
          //!< Thermal1(2, 3, 4) Thermal2(4, 3, 2) Hole1(1, 2, 0)
          //!< (yaw = 0) ApproachDist = 5
          createVariousObjects1(objConstPtrVect1_);
          //!< create the Objectvector2 and fill it with various objects
          //!< Thermal1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)
          //!< (yaw = PI/4) ApproachDist = 6
          createVariousObjects2(objConstPtrVect2_);
          //!< create the Objectvector3 and fill it with various objects
          //!< Thermal1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)
          //!< (yaw = PI/4) ApproachDist = 6
          createVariousObjects3(objConstPtrVect3_);
        }

        virtual void TearDown()
        {
          clear();
        }

        /* Helper functions */

        //!< Returns distance between 2 objects.
        float distance(const ObjectConstPtr& object1, const ObjectConstPtr& object2)
        {
          float x_ = object1->getPose().position.x - object2->getPose().position.x;
          float y_ = object1->getPose().position.y - object2->getPose().position.y;
          float z_ = object1->getPose().position.z - object2->getPose().position.z;
          return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        }

        //!< We create manually Thermal1(2, 3, 4),
        //!< Thermal2(4, 3, 2), Hole1(1, 2, 0)
        void createVariousObjects1(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(2, 3, 4));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          ThermalPtr thermalPtr2(new Thermal);
          thermalPtr2->setPose(makePose(4, 3, 2));
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          HolePtr holePtr1(new Hole);
          holePtr1->setPose(makePose(1, 2, 0));
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(thermalPtr2);
          objConstPtrVectPtr->push_back(holePtr1);
        }

        //!< We create manually Thermal1(2, 3, 4),
        //!< Hole1(4, 3, 2), Hole2(0, 4, 2) with yaw=PI/4
        void createVariousObjects2(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(2, 3, 4));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          thermalPtr1->setId(1);
          thermalPtr1->update(thermalPtr1);
          thermalPtr1->update(thermalPtr1);
          HolePtr holePtr1(new Hole);
          holePtr1->setPose(makePose(4, 3, 2));
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          holePtr1->setId(1);
          HolePtr holePtr2(new Hole);
          holePtr2->setPose(makePose(0, 4, 2, PI/4));
          holePtr2->setProbability(0.5);
          holePtr2->initializeObjectFilter();
          holePtr2->setId(2);
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(holePtr1);
          objConstPtrVectPtr->push_back(holePtr2);
        }

        //!< We create manually thermal1(2, 3, 4) thermal2(4, 3, 2) thermal3(0, 4, 2)
        //!< Yaw = PI/4
        void createVariousObjects3(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(2, 3, 4));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          thermalPtr1->setId(1);
          ThermalPtr thermalPtr2(new Thermal);
          thermalPtr2->setPose(makePose(2, 3, 4, PI/4));
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          thermalPtr2->setId(2);
          ThermalPtr thermalPtr3(new Thermal);
          thermalPtr3->setPose(makePose(0, 4, 2, PI/4));
          thermalPtr3->setProbability(0.5);
          thermalPtr3->initializeObjectFilter();
          thermalPtr3->setId(3);
          thermalPtr3->update(thermalPtr3);
          thermalPtr3->update(thermalPtr3);
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(thermalPtr2);
          objConstPtrVectPtr->push_back(thermalPtr3);
        }

        //!< We create manually thermal1(2, 3, 4) thermal2(4, 3, 2) thermal3(0, 4, 2)
        //!< Yaw = PI/4
        geometry_msgs::Pose makePose(float x, float y, float z, float yaw = 0)
        {
          geometry_msgs::Pose pose;
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,  0,  yaw);
          return pose;
        }

        /* Accessors to private functions */

        template <class ObjectType>
        void findRepresentativeObject(VictimPtr victim,
            const ObjectConstPtrVector& objects)
        {
          victim->findRepresentativeObject<ObjectType>(objects);
        }

        void clear()
        {
          Victim::lastVictimId_ = 0;
        }

        /* Accessors to private variables */

        float getStdDevX(ObjectConstPtr object)
        {
          if (object->getType() == "hole")
            return boost::dynamic_pointer_cast<const Hole>(object)->getStdDevX();
          if (object->getType() == "thermal")
            return boost::dynamic_pointer_cast<const Thermal>(object)->getStdDevX();
        }

        int* getLastVictimId(VictimPtr victim)
        {
          return &(victim->lastVictimId_);
        }

        /* Variables */

        VictimPtr victim1_;
        VictimPtr victim2_;
        ObjectConstPtrVectorPtr objConstPtrVect1_;
        ObjectConstPtrVectorPtr objConstPtrVect2_;
        ObjectConstPtrVectorPtr objConstPtrVect3_;
    };

    TEST_F(VictimTest, constructor)
    {
      clear();
      VictimPtr first( new Victim );
      EXPECT_FALSE(first->getVisited());
      EXPECT_FALSE(first->getValid());
      EXPECT_EQ(1, *getLastVictimId(first));
      EXPECT_EQ(-1, first->getSelectedObjectIndex());

      VictimPtr second( new Victim );
      EXPECT_FALSE(first->getVisited());
      EXPECT_FALSE(first->getValid());
      EXPECT_EQ(2, *getLastVictimId(first));
      EXPECT_EQ(2, *getLastVictimId(second));
      EXPECT_EQ(-1, second->getSelectedObjectIndex());
    }

    TEST_F(VictimTest, isSameObject)
    {
      EXPECT_EQ(2, *getLastVictimId(victim1_));
      victim1_->setPose(makePose(5, 1, 0));
      victim2_->setPose(makePose(0, 1, 2));
      Victim::setDistanceThres(7);
      EXPECT_TRUE(victim1_->isSameObject(victim2_));
      Victim::setDistanceThres(-5);
      EXPECT_FALSE(victim1_->isSameObject(victim2_));
      Victim::setDistanceThres(1);
      EXPECT_FALSE(victim1_->isSameObject(victim2_));
    }

    TEST_F(VictimTest, setObjects1)
    {
      victim1_->setObjects(*objConstPtrVect1_);
      // There will be 2 objects left in victim1_. One Hole and one Thermal type.
      EXPECT_EQ(2, victim1_->getObjects().size());
      EXPECT_EQ("hole", victim1_->getObjects().at(0)->getType());
      EXPECT_EQ("thermal", victim1_->getObjects().at(1)->getType());
      // Hole objects are prefered over Thermal objects regarding representatives.
      // victim1_'s Thermal object should be the thermalPtr1! thermalPtr1 had more confidence
      // in its belief about position that thermalPtr2, so it was prefered during
      // setObject()
      EXPECT_GE(getStdDevX(objConstPtrVect1_->at(0)),
          getStdDevX(victim1_->getObjects().at(1)));
      EXPECT_GT(getStdDevX(objConstPtrVect1_->at(1)) -
          getStdDevX(victim1_->getObjects().at(1)),
          getStdDevX(objConstPtrVect1_->at(0)) -
          getStdDevX(victim1_->getObjects().at(1)));
      // thermalPtr1 object should be updated so that its position is
      // closer to thermalPtr2's
      EXPECT_GT(distance(objConstPtrVect1_->at(0), objConstPtrVect1_->at(1)),
          distance(victim1_->getObjects().at(1), objConstPtrVect1_->at(1)));
      EXPECT_GT(distance(objConstPtrVect1_->at(0), objConstPtrVect1_->at(1)),
          distance(victim1_->getObjects().at(1), objConstPtrVect1_->at(0)));
    }

    TEST_F(VictimTest, setObjects2)
    {
      victim2_->setObjects(*objConstPtrVect2_);
      // There will be 2 objects left in victim1_. One Hole and one Thermal type.
      EXPECT_EQ(2, victim2_->getObjects().size());
      EXPECT_EQ("hole", victim2_->getObjects().at(0)->getType());
      EXPECT_EQ(1, victim2_->getObjects().at(0)->getId());
      EXPECT_EQ("thermal", victim2_->getObjects().at(1)->getType());
      EXPECT_EQ(1, victim2_->getObjects().at(1)->getId());
      // Hole objects are prefered over Thermal objects regarding representatives.
      // victim1_'s Thermal object should be the thermalPtr1! thermalPtr1 had more confidence
      // in its belief about position that thermalPtr2, so it was prefered during
      // setObject()
      EXPECT_GE(getStdDevX(objConstPtrVect2_->at(1)),
          getStdDevX(victim2_->getObjects().at(0)));
      EXPECT_GT(getStdDevX(objConstPtrVect2_->at(2)) -
          getStdDevX(victim2_->getObjects().at(0)),
          getStdDevX(objConstPtrVect2_->at(1)) -
          getStdDevX(victim2_->getObjects().at(0)));
      // thermalPtr1 object should be updated so that its position is
      // closer to thermalPtr2's
      EXPECT_GT(distance(objConstPtrVect2_->at(1), objConstPtrVect2_->at(2)),
          distance(victim2_->getObjects().at(0), objConstPtrVect2_->at(2)));
      EXPECT_GT(distance(objConstPtrVect2_->at(1), objConstPtrVect2_->at(2)),
          distance(victim2_->getObjects().at(0), objConstPtrVect2_->at(1)));
    }

    TEST_F(VictimTest, findRepresentativeObject)
    {
      findRepresentativeObject<Thermal>(victim2_, *objConstPtrVect3_);
      ASSERT_EQ(1, victim2_->getObjects().size());
      EXPECT_EQ("thermal", victim2_->getObjects().at(0)->getType());
      EXPECT_EQ(3, victim2_->getObjects().at(0)->getId());
    }

    TEST_F(VictimTest, eraseObjectAt)
    {
      // Thermal1(2, 3, 4) Thermal2(4, 3, 2) Hole1(1, 2, 0)
      victim1_->setObjects(*objConstPtrVect1_);
      victim1_->eraseObjectAt(1);
      // thermal1 is erased.
      EXPECT_EQ(1, victim1_->getPose().position.x);
      EXPECT_EQ(2, victim1_->getPose().position.y);
      EXPECT_EQ(0, victim1_->getPose().position.z);
      EXPECT_EQ(0, victim1_->getSelectedObjectIndex());
      EXPECT_EQ(1, victim1_->getObjects().size());
      // hole1 is erased
      victim1_->eraseObjectAt(0);
      EXPECT_EQ(0, victim1_->getObjects().size());

      // Thermal1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)
      victim2_->setObjects(*objConstPtrVect2_);
      // Erase hole1. Now thermal1 is the representative object.
      victim2_->eraseObjectAt(0);
      EXPECT_EQ(2, victim2_->getPose().position.x);
      EXPECT_EQ(3, victim2_->getPose().position.y);
      EXPECT_EQ(4, victim2_->getPose().position.z);
      EXPECT_EQ(0, victim2_->getSelectedObjectIndex());
      EXPECT_EQ(1, victim2_->getObjects().size());
      // Erase the last object (thermal1).
      victim2_->eraseObjectAt(1);
      EXPECT_EQ(0, victim2_->getObjects().size());
      EXPECT_EQ(-1, victim2_->getSelectedObjectIndex());
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

