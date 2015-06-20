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

#include "alert_handler/victim_list.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class VictimListTest : public ::testing::Test
    {
      protected:
        VictimListTest()
        {
          ros::Time::init();
          victim1_.reset( new Victim );
          victim2_.reset( new Victim );
          victim3_.reset( new Victim );
          victim4_.reset( new Victim );
        }

        virtual void SetUp()
        {
          Hole::setObjectType("HOLE");
          Thermal::setObjectType("THERMAL");

          victimList_.reset( new VictimList );
          objConstPtrVect1_.reset( new ObjectConstPtrVector );
          objConstPtrVect2_.reset( new ObjectConstPtrVector );
          objConstPtrVect3_.reset( new ObjectConstPtrVector );
          objConstPtrVect4_.reset( new ObjectConstPtrVector );

          createVariousObjects1(objConstPtrVect1_);
          createVariousObjects2(objConstPtrVect2_);
          createVariousObjects3(objConstPtrVect3_);
          createVariousObjects4(objConstPtrVect4_);

          fillVictim(victim1_, objConstPtrVect1_);
          fillVictim(victim2_, objConstPtrVect2_);
          fillVictim(victim3_, objConstPtrVect3_);
          fillVictim(victim4_, objConstPtrVect4_);

          fillVictimListAdd(victimList_);
        }

        virtual void TearDown()
        {
          victimList_->clear();
        }

        /* Helper functions */

        geometry_msgs::Pose makePose(float x, float y, float z, float yaw = 0)
        {
          geometry_msgs::Pose pose;
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,  0,  yaw);
          return pose;
        }

        void fillVictimListAdd(VictimListPtr victimList)
        {
          victimList->clear();
          victimList->add(victim1_);
          victimList->add(victim2_);
          victimList->add(victim3_);
          victimList->add(victim4_);
        }

        void fillVictimListAddUnchanged(VictimListPtr victimList)
        {
          victimList->clear();
          victimList->addUnchanged(victim1_);
          victimList->addUnchanged(victim2_);
          victimList->addUnchanged(victim3_);
          victimList->addUnchanged(victim4_);
        }

        void fillVictim(VictimPtr victim, ObjectConstPtrVectorPtr objConstPtrVect)
        {
          victim->setObjects(*objConstPtrVect);
        }

        //!< We fill the iteratorList_ specifically around victim2_.
        void fillIteratorList(VictimListPtr victimList,
            VictimList::IteratorList* iteratorListPtr)
        {
          for (VictimList::iterator it = victimList->objects_.begin();
              it != victimList->objects_.end(); ++it)
          {
            if ((*it)->isSameObject(victim2_))
            {
              iteratorListPtr->push_back(it);
            }
          }
        }

        //!< Thermal1(-1, 0, 0), Thermal2(1, 0, 0), Hole1(0 , 1, 0)
        void createVariousObjects1(ObjectConstPtrVectorPtr objConstPtrVect)
        {
          ThermalPtr thermalPtr1( new Thermal );
          thermalPtr1->setPose(makePose(-1, 0, 0));
          thermalPtr1->setId(1);
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          ThermalPtr thermalPtr2( new Thermal );
          thermalPtr2->setPose(makePose(1, 0, 0));
          thermalPtr2->setId(2);
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          thermalPtr2->update(thermalPtr2);
          thermalPtr2->update(thermalPtr2);
          HolePtr holePtr1( new Hole );
          holePtr1->setPose(makePose(0, 1, 0));
          holePtr1->setId(1);
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          objConstPtrVect->push_back(thermalPtr1);
          objConstPtrVect->push_back(thermalPtr2);
          objConstPtrVect->push_back(holePtr1);
        }

        //!< Thermal1(2, 3, 0), Hole1(3, 3, 0), Hole2(2, 2.5, 0)
        void createVariousObjects2(ObjectConstPtrVectorPtr objConstPtrVect)
        {
          ThermalPtr thermalPtr1( new Thermal );
          thermalPtr1->setPose(makePose(2, 3, 0));
          thermalPtr1->setId(1);
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();

          HolePtr holePtr1( new Hole );
          holePtr1->setPose(makePose(3, 3, 0));
          holePtr1->setId(1);
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          holePtr1->update(holePtr1);
          holePtr1->update(holePtr1);

          HolePtr holePtr2( new Hole );
          holePtr2->setPose(makePose(2, 2.5, 0));
          holePtr2->setId(2);
          holePtr2->setProbability(0.5);
          holePtr2->initializeObjectFilter();

          objConstPtrVect->push_back(thermalPtr1);
          objConstPtrVect->push_back(holePtr1);
          objConstPtrVect->push_back(holePtr2);
        }

        //!< Thermal1(2.7, 3, 0), Thermal2(3, 3, 0)
        void createVariousObjects3(ObjectConstPtrVectorPtr objConstPtrVect)
        {
          ThermalPtr thermalPtr1( new Thermal );
          thermalPtr1->setPose(makePose(2.7, 3, 0));
          thermalPtr1->setId(1);
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();

          ThermalPtr thermalPtr2( new Thermal );
          thermalPtr2->setPose(makePose(3, 3, 0));
          thermalPtr2->setId(2);
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          thermalPtr2->update(thermalPtr2);
          thermalPtr2->update(thermalPtr2);

          objConstPtrVect->push_back(thermalPtr1);
          objConstPtrVect->push_back(thermalPtr2);
        }

        //!< Thermal1(3, 3, 0), Hole1(10, 3, 0)
        void createVariousObjects4(ObjectConstPtrVectorPtr objConstPtrVect)
        {
          ThermalPtr thermalPtr1( new Thermal );
          thermalPtr1->setPose(makePose(3, 3, 0));
          thermalPtr1->setId(1);
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          HolePtr holePtr1( new Hole );
          holePtr1->setPose(makePose(10, 3, 0));
          holePtr1->setId(1);
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          objConstPtrVect->push_back(thermalPtr1);
          objConstPtrVect->push_back(holePtr1);
        }

        /* Accesors to private functions */

        void updateObjects(VictimListPtr victimList,
            const VictimPtr& victim,
            const VictimList::IteratorList& iteratorList_)
        {
          victimList->updateObjects(victim, iteratorList_);
        }

        VictimList::List& getObjects(VictimListPtr victimList)
        {
          return victimList->objects_;
        }

        /* Variables */

        VictimListPtr victimList_;
        VictimPtr victim1_;
        VictimPtr victim2_;
        VictimPtr victim3_;
        VictimPtr victim4_;
        ObjectConstPtrVectorPtr objConstPtrVect1_;
        ObjectConstPtrVectorPtr objConstPtrVect2_;
        ObjectConstPtrVectorPtr objConstPtrVect3_;
        ObjectConstPtrVectorPtr objConstPtrVect4_;
        VictimList::IteratorList iteratorList_;
    };

    TEST_F(VictimListTest, contains)
    {
      // Victim1 Thermal(1, 0, 0) Hole(0, 1, 0)
      // Victim2 Thermal(2, 3, 0) Hole(3, 3, 0)
      // Victim3 Thermal(3, 3, 0)
      // Victim4 Thermal(3, 3, 0) Hole(10, 3, 0)

      // victim 3 is the same as victim 2 (samePosition)
      ASSERT_EQ(3, victimList_->size());
      EXPECT_TRUE(victimList_->contains(victim1_));
      EXPECT_TRUE(victimList_->contains(victim2_));
      EXPECT_TRUE(victimList_->contains(victim3_));
      EXPECT_TRUE(victimList_->contains(victim4_));
    }

    TEST_F(VictimListTest, updateObjects)
    {
      ObjectList<Victim>::iterator it;
      VictimListPtr victimList2( new VictimList );

      fillVictimListAddUnchanged(victimList2);
      fillIteratorList(victimList2 , &iteratorList_);
      ASSERT_EQ(4, victimList2->size());
      // VictimList2 has all 4 victims inside
      it = getObjects(victimList2).begin();
      EXPECT_EQ(victim1_, *(it));
      EXPECT_EQ(victim2_, *(++it));
      EXPECT_EQ(victim3_, *(++it));
      EXPECT_EQ(victim4_, *(++it));
      // Victim3 will be removed
      updateObjects(victimList2, victim2_, iteratorList_);
      ASSERT_EQ(3, victimList2->size());
      it = getObjects(victimList2).begin();
      EXPECT_EQ(victim1_, *(it));
      EXPECT_EQ(victim2_, *(++it));
      EXPECT_EQ(victim4_, *(++it));
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

