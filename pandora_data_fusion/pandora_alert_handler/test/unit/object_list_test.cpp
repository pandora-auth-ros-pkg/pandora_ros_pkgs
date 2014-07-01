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

#include <limits>
#include <cstdlib>
#include <ctime>

#include "gtest/gtest.h"

#include "alert_handler/object_list.h"
#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class ObjectListTest : public testing::Test
    {
      protected:
        /* Constructor/Destructor */

        ObjectListTest()
        {
          qrList.reset( new ObjectList<Qr> );
          qrList2.reset( new ObjectList<Qr> );
          qr1.reset( new Qr );
          qr2.reset( new Qr );
          qr3.reset( new Qr );
          qr4.reset( new Qr );
          qr5.reset( new Qr );
          qr6.reset( new Qr );
          qr7.reset( new Qr );
          qr8.reset( new Qr );
          qr9.reset( new Qr );
          qr10.reset( new Qr );
          qr11.reset( new Qr );
        }

        /* SetUp/TearDown definitions */

        virtual void SetUp()
        {
          std::srand(std::time(0));

          seed = time(NULL);

          Pose pose;

          Qr::setDistanceThres(0.5);
          Qr::setProbabilityThres(0.75);
          Qr::setObjectType("QR");

          qr1->setId(1);
          set(-0.5, 0, 0, qr1);

          qr2->setId(2);
          set(0, 0, 0, qr2);

          qr3->setId(3);
          set(0.5, 0, 0, qr3);

          qr4->setId(4);
          set(-1, 0, 0, qr4);

          qr5->setId(5);
          set(-0.125, -0.125, 0, qr5);

          qr6->setId(6);
          set(0, 0.4, 0, qr6);

          float x = static_cast<double>
            (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
          float y =
            static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
          float z =
            static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
          qr7->setId(7);
          set(x, y, z, qr7);

          qr8->setId(8);
          set(0.125, 0.125, 0, qr8);

          qr9->setId(9);
          set(0.4, 0.1, 0, qr9);

          qr10->setId(10);
          set(0.6, 0.2, 0.1, qr10);

          qr11->setId(11);
          set(-0.75, 0, 0, qr11);
        }

        /* Helper functions */
        /* Function to fill qrList.qrs_ */

        void fillList(QrListPtr objList)
        {
          getObjects(objList).clear();
          getObjects(objList).push_back(qr1);
          getObjects(objList).push_back(qr2);
          getObjects(objList).push_back(qr3);
        }

        void fillListRandom(QrListPtr objList, int n)
        {
          getObjects(objList).clear();

          for (int ii = 0; ii < n; ++ii)
          {
            Pose pose;
            pose.position.x =
              static_cast<double>(rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
            pose.position.y=
              static_cast<double>(rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
            pose.position.z =
              static_cast<double>(rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
            QrPtr qr(new Qr);
            qr->setPose(pose);
            getObjects(objList).push_back(qr);
          }
        }

        //!< Spawns Qrs In a fixed radius Around the qrX
        QrPtr QrSpawner(QrPtr qrX, float radius)
        {
          QrPtr closeQr( new Qr );
          double angle1(static_cast<double>((rand_r(&seed) % 314)) / 100);
          double angle2(static_cast<double>((rand_r(&seed) % 314)) / 200);

          float x = (cos(angle2) * radius) * sin(angle1) +
            qrX->getPose().position.x;
          float y = (cos(angle2) * radius) * cos(angle1) +
            qrX->getPose().position.y;
          float z = sin(angle2) * radius  + qrX->getPose().position.z;

          set(x, y, z, closeQr);

          return closeQr;
        }


        //!< Returns distance between 2 qrs.
        float distance(const Pose& pose1, const Pose& pose2)
        {
          float x_ = pose1.position.x - pose2.position.x;
          float y_ = pose1.position.y - pose2.position.y;
          float z_ = pose1.position.z - pose2.position.z;
          return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        }

        void set(float x, float y, float z, QrPtr qr)
        {
          Pose pose;
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          qr->setPose(pose);
          qr->setProbability(0.5);
          qr->initializeObjectFilter();
        }

        /* Accessors for private methods/members of QrList */

        bool isAnExistingObject(
            QrListPtr objList, const QrConstPtr& qr,
            ObjectList<Qr>::IteratorList* iteratorListPtr)
        {
          return objList->isAnExistingObject(qr, iteratorListPtr);
        }

        int* id(QrListPtr objList)
        {
          return &(objList->id_);
        }

        ObjectList<Qr>::List& getObjects(QrListPtr objList)
        {
          return objList->objects_;
        }

        /* Variables */

        unsigned int seed;
        QrListPtr qrList;
        QrListPtr qrList2;
        QrPtr qr1;
        QrPtr qr2;
        QrPtr qr3;
        QrPtr qr4;
        QrPtr qr5;
        QrPtr qr6;
        QrPtr qr7;
        QrPtr qr8;
        QrPtr qr9;
        QrPtr qr10;
        QrPtr qr11;
    };

    TEST_F(ObjectListTest, constructor)
    {
      EXPECT_EQ(0u, qrList->size());
      EXPECT_EQ(0u, *id(qrList));
    }

    TEST_F(ObjectListTest, isAnExistingObject)
    {
      fillList(qrList);
      ObjectList<Qr>::IteratorList iteratorList;
      ObjectList<Qr>::IteratorList::const_iterator it;

      ASSERT_EQ(0.5, qr1->getDistanceThres());

      // It shouldn't find that Qr4 already exists.
      // Qr4 won't correlate with Qr1 because dist equals DIST_THRES!

      EXPECT_FALSE(isAnExistingObject(qrList, qr4, &iteratorList));
      EXPECT_TRUE(iteratorList.empty());
      iteratorList.clear();

      // It should find that qr5  exists in 2 places.

      EXPECT_TRUE(isAnExistingObject(qrList, qr5, &iteratorList));
      it = iteratorList.begin();
      EXPECT_FALSE(iteratorList.empty());
      ASSERT_EQ(2u, iteratorList.size());
      EXPECT_EQ(qr1, **(it++));
      EXPECT_EQ(qr2, **(it));

      iteratorList.clear();

      // It should find that qr6 exists only  in 1 place( same as qr 3).
      EXPECT_TRUE(iteratorList.empty());
      EXPECT_TRUE(isAnExistingObject(qrList, qr6, &iteratorList));
      it = iteratorList.begin();
      EXPECT_FALSE(iteratorList.empty());
      ASSERT_EQ(1u, iteratorList.size());
      EXPECT_EQ(qr2, **it);
      EXPECT_EQ(qr2, **it);

      iteratorList.clear();

      // Changed distance must find Qr 6 in  3 places( qr1 qr2 qr3).
      Qr::setDistanceThres(1);
      EXPECT_TRUE(isAnExistingObject(qrList, qr6, &iteratorList));
      it = iteratorList.begin();
      EXPECT_FALSE(iteratorList.empty());
      ASSERT_EQ(3u, iteratorList.size());
      EXPECT_EQ(qr1, **(it++));
      EXPECT_EQ(qr2, **(it++));
      EXPECT_EQ(qr3, **it);

      iteratorList.clear();

      // Or it doesn't find it.
      Qr::setDistanceThres(0.2);

      EXPECT_FALSE(isAnExistingObject(qrList, qr6, &iteratorList));
      EXPECT_TRUE(iteratorList.empty());
      EXPECT_EQ(0u, iteratorList.size());

      iteratorList.clear();

      // Zero Distance Threshold makes impossible same Qr recognition.
      Qr::setDistanceThres(0);
      fillListRandom(qrList, 10000);

      EXPECT_FALSE(isAnExistingObject(qrList, qr7, &iteratorList));
      EXPECT_TRUE(iteratorList.empty());
      EXPECT_EQ(0u, iteratorList.size());

      iteratorList.clear();

      // Maximum (Infinite) Distance Threshold makes impossible Qr distinction.
      Qr::setDistanceThres(std::numeric_limits<float>::max());

      EXPECT_TRUE(isAnExistingObject(qrList, qr7, &iteratorList));
      EXPECT_FALSE(iteratorList.empty());
      EXPECT_EQ(qrList->size(), iteratorList.size());

      iteratorList.clear();
    }


    TEST_F(ObjectListTest, addManually)
    {
      ObjectList<Qr>::const_iterator_vol_ref it = getObjects(qrList).begin();
      ASSERT_EQ(0u, qrList->size());
      ASSERT_EQ(0.5, qr1->getDistanceThres());

      // Add (0.4,0.1,0)
      EXPECT_TRUE(qrList->add(qr9));
      ASSERT_EQ(1, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_FALSE(qr9->getLegit());

      // Add (0.5, 0, 0) Qr 3 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr3));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9 , *it);
      EXPECT_FALSE(qr9->getLegit());

      // Add (0.125, 0.125, 0) Qr 8 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr8));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());

      // Add (0.6, 0.2, 0.1) Qr 10 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr10));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());

      // Add (0.6, 0.2, 0.1) Qr 10 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr10));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());

      // Add (0.125, 0.125, 0) Qr 9 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr9));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());

      // Add (0.125, 0.125, 0) Qr 8 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr8));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());

      // Add (0.125, 0.125, 0) Qr 9 will not be Added Same as Qr 9
      EXPECT_FALSE(qrList->add(qr9));
      ASSERT_EQ(1u, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr9, *it);
      EXPECT_TRUE(qr9->getLegit());
    }

    TEST_F(ObjectListTest, addRandomly)
    {
      ObjectList<Qr>::const_iterator_vol_ref it = getObjects(qrList).begin();
      ASSERT_EQ(0u, qrList->size());
      ASSERT_EQ(0.5, qr1->getDistanceThres());

      EXPECT_TRUE(qrList->add(qr2));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.4)));
      ASSERT_EQ(1, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr2, *it);
      EXPECT_FALSE(qr2->getLegit());

      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.1)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.3)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.1)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.3)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr2, 0.1)));
      EXPECT_TRUE(qr2->getLegit());
    }

    TEST_F(ObjectListTest, addTwoQrs)
    {
      ObjectList<Qr>::const_iterator_vol_ref it = getObjects(qrList).begin();
      ASSERT_EQ(0u, qrList->size());
      ASSERT_EQ(0.5, qr1->getDistanceThres());

      EXPECT_TRUE(qrList->add(qr1));
      ASSERT_EQ(1, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr1, *it);
      EXPECT_FALSE(qr1->getLegit());

      EXPECT_TRUE(qrList->add(qr4));
      ASSERT_EQ(2, qrList->size());
      it = getObjects(qrList).begin();
      EXPECT_EQ(qr4, *(++it));
      EXPECT_FALSE(qr4->getLegit());

      // Qr  1(-0,5,0,0) Qr 4 (-1 ,0,0) Qr11 (-0.75 ,0, 0)
      // This will spawn Qrs in the middle so both Qrs should became legit
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.1)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.3)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.1)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.3)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.2)));
      EXPECT_FALSE(qrList->add(QrSpawner(qr11, 0.1)));

      // They should have moved closer to each other
      EXPECT_LT(distance(qr1->getPose(), qr4->getPose()), 0.5);

      // They should become Legit!
      EXPECT_TRUE(qr4->getLegit());
      EXPECT_TRUE(qr1->getLegit());
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

