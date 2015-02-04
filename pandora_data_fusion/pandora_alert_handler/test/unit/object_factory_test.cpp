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

#include <string>

#include <ros/package.h>

#include "gtest/gtest.h"
#include <pandora_testing_tools/map_loader/map_loader.h>

#include "alert_handler/object_factory.h"
#include "alert_handler/defines.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    class ObjectFactoryTest : public ::testing::Test
    {
      public:
        /* Constructor SetUp */

        //!< Loading the Map and initializing Objectfactory with a
        //!< very small orientation circle(The other Params are the same)
        ObjectFactoryTest()
          : map_type("TEST"), mapPtr( new Map )
        {
          ros::Time::init();
          *mapPtr = map_loader::loadMap(
              ros::package::getPath("pandora_alert_handler") +
              "/test/test_maps/map1.yaml");
          objectFactoryPtr.reset( new ObjectFactory(mapPtr, map_type) );
          objectFactoryPtr->dynamicReconfigForward(0.5, 1.2, 0, 0.5, 0.25);
        }

        //!< Creating all the Different Alerts we will use
        virtual void SetUp()
        {
          createHoleDirVector();
          createHazmatAlertVector();
          createQrAlertVector();
          createThermalDir();
        }

      protected:
        /* Helper Functions */

        //!< Creating 3 HoleAlerts two on Walls and one very High
        void createHoleDirVector()
        {
          holeDirVect1.header.seq = 1;
          holeDirVect1.header.frame_id = "Maria";
          holeDir1.yaw = -PI/4;
          holeDir1.pitch = PI/6;
          holeDir1.probability = 0.3;
          holeDir1.holeId = 1;
          holeDirVect1.holesDirections.push_back(holeDir1);
          holeDir2.yaw = -PI/2;
          holeDir2.pitch = 0;
          holeDir2.probability = 0.5;
          holeDir2.holeId = 2;
          holeDirVect1.holesDirections.push_back(holeDir2);
          holeDir3.yaw = 0;
          holeDir3.pitch = PI/2;
          holeDir3.probability = 0.8;
          holeDir3.holeId = 3;
          holeDirVect1.holesDirections.push_back(holeDir3);
        }

        // Creating 3 QrAlerts two on Walls and one very High
        void createQrAlertVector()
        {
          qrAlert1.yaw = -PI/4;
          qrAlert1.pitch = PI/6;
          qrAlert1.QRcontent =
            "No one can make you feel inferior without your consent.";
          qrAlertVect1.qrAlerts.push_back(qrAlert1);
          qrAlert2.yaw = -PI/2;
          qrAlert2.pitch = 0;
          qrAlert2.QRcontent =
            "Let him who would enjoy a good future waste none of his present.";
          qrAlertVect1.qrAlerts.push_back(qrAlert2);
          qrAlert3.yaw = 0;
          qrAlert3.pitch = PI/2;
          qrAlert3.QRcontent =
            "Live as if you were to die tomorrow. Learn as if you were to live forever.";
          qrAlertVect1.qrAlerts.push_back(qrAlert3);
        }

        // Creating 3 HazmatAlerts two on Walls and one very High
        void createHazmatAlertVector()
        {
          hazmatAlert1.yaw = -PI/4;
          hazmatAlert1.pitch = PI/6;
          hazmatAlert1.patternType = 1;
          hazmatAlertVect1.hazmatAlerts.push_back(hazmatAlert1);
          hazmatAlert2.yaw = -PI/2;
          hazmatAlert2.pitch = 0;
          hazmatAlert2.patternType = 2;
          hazmatAlertVect1.hazmatAlerts.push_back(hazmatAlert2);
          hazmatAlert3.yaw = 0;
          hazmatAlert3.pitch = PI/2;
          hazmatAlert3.patternType = 3;
          hazmatAlertVect1.hazmatAlerts.push_back(hazmatAlert3);
        }

        // Creating 2 ThermalAlerts one on Wall and one very High
        void createThermalDir()
        {
          thermalDir1.yaw = -PI/4;
          thermalDir1.pitch =  PI/6;
          thermalDir1.probability = 0.6;
          thermalDir2.yaw = 0;
          thermalDir2.pitch =  PI/2;
          thermalDir2.probability = 0.7;
        }

        /* variables */

        MapPtr mapPtr;
        const std::string map_type;
        ObjectFactoryPtr objectFactoryPtr;
        pandora_vision_msgs::HolesDirectionsVectorMsg holeDirVect1;
        pandora_vision_msgs::HoleDirectionMsg holeDir1;
        pandora_vision_msgs::HoleDirectionMsg holeDir2;
        pandora_vision_msgs::HoleDirectionMsg holeDir3;
        pandora_vision_msgs::QRAlertsVectorMsg qrAlertVect1;
        pandora_vision_msgs::QRAlertMsg qrAlert1;
        pandora_vision_msgs::QRAlertMsg qrAlert2;
        pandora_vision_msgs::QRAlertMsg qrAlert3;
        pandora_vision_msgs::HazmatAlertsVectorMsg hazmatAlertVect1;
        pandora_vision_msgs::HazmatAlertMsg hazmatAlert1;
        pandora_vision_msgs::HazmatAlertMsg hazmatAlert2;
        pandora_vision_msgs::HazmatAlertMsg hazmatAlert3;
        pandora_common_msgs::GeneralAlertMsg  thermalDir1;
        pandora_common_msgs::GeneralAlertMsg  thermalDir2;
    };

    // The tf that is used is created in mock object tfFinder
    // With origin (5,5,0.3) and (roll,pitch,yaw)=(0,0,0)

    /* Test Cases */

    TEST_F(ObjectFactoryTest, makeHoles)
    {
      HolePtrVectorPtr holesVectorPtr( new HolePtrVector );
      holesVectorPtr = objectFactoryPtr->makeHoles(holeDirVect1);
      ASSERT_EQ(2, holesVectorPtr->size());

      HolePtr holePtr1 = (*holesVectorPtr)[0];
      EXPECT_EQ(1, holePtr1->getHoleId());
      EXPECT_NEAR(5.76, holePtr1->getPose().position.x, 0.1);
      EXPECT_NEAR(5.76, holePtr1->getPose().position.y, 0.1);
      EXPECT_NEAR(0.9205, holePtr1->getPose().position.z, 0.01);
      EXPECT_NEAR(0, holePtr1->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, holePtr1->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, holePtr1->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, holePtr1->getPose().orientation.w, 0.1);

      HolePtr holePtr2 = (*holesVectorPtr)[1];
      EXPECT_EQ(2, holePtr2->getHoleId());
      EXPECT_NEAR(5, holePtr2->getPose().position.x, 0.1);
      EXPECT_NEAR(5.76, holePtr2->getPose().position.y, 0.1);
      EXPECT_NEAR(0.3, holePtr2->getPose().position.z, 0.01);
      EXPECT_NEAR(0, holePtr2->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, holePtr2->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, holePtr2->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, holePtr2->getPose().orientation.w, 0.1);
    }

    TEST_F(ObjectFactoryTest, makeQrs)
    {
      QrPtrVectorPtr qrsVectorPtr( new QrPtrVector );
      qrsVectorPtr = objectFactoryPtr->makeQrs(qrAlertVect1);
      ASSERT_EQ(2, qrsVectorPtr->size());

      QrPtr qrPtr1 = (*qrsVectorPtr)[0];
      EXPECT_STREQ("No one can make you feel inferior without your consent."
          , qrPtr1->getContent().c_str());
      EXPECT_NEAR(5.76, qrPtr1->getPose().position.x, 0.1);
      EXPECT_NEAR(5.76, qrPtr1->getPose().position.y, 0.1);
      EXPECT_NEAR(0.9205, qrPtr1->getPose().position.z, 0.01);
      EXPECT_NEAR(0, qrPtr1->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, qrPtr1->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, qrPtr1->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, qrPtr1->getPose().orientation.w, 0.1);

      QrPtr qrPtr2=(*qrsVectorPtr)[1];
      EXPECT_STREQ("Let him who would enjoy a good future waste none of his present."
          , qrPtr2->getContent().c_str());
      EXPECT_NEAR(5, qrPtr2->getPose().position.x, 0.01);
      EXPECT_NEAR(5.76, qrPtr2->getPose().position.y, 0.01);
      EXPECT_NEAR(0.3, qrPtr2->getPose().position.z, 0.01);
      EXPECT_NEAR(0, qrPtr2->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, qrPtr2->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, qrPtr2->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, qrPtr2->getPose().orientation.w, 0.1);
    }

    TEST_F(ObjectFactoryTest, makeHazmats)
    {
      HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
      hazmatsVectorPtr = objectFactoryPtr->makeHazmats(hazmatAlertVect1);
      EXPECT_EQ(2, hazmatsVectorPtr->size());

      HazmatPtr hazmatPtr1 = (*hazmatsVectorPtr)[0];
      EXPECT_EQ(1, hazmatPtr1->getPattern());
      EXPECT_NEAR(5.75, hazmatPtr1->getPose().position.x, 0.01);
      EXPECT_NEAR(5.75, hazmatPtr1->getPose().position.y, 0.01);
      EXPECT_NEAR(0.9205, hazmatPtr1->getPose().position.z, 0.01);
      EXPECT_NEAR(0, hazmatPtr1->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, hazmatPtr1->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, hazmatPtr1->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, hazmatPtr1->getPose().orientation.w, 0.1);
      HazmatPtr hazmatPtr2 = (*hazmatsVectorPtr)[1];
      EXPECT_EQ(2, hazmatPtr2->getPattern());
      EXPECT_NEAR(5, hazmatPtr2->getPose().position.x, 0.01);
      EXPECT_NEAR(5.76, hazmatPtr2->getPose().position.y, 0.01);
      EXPECT_NEAR(0.3, hazmatPtr2->getPose().position.z, 0.01);
      EXPECT_NEAR(0, hazmatPtr2->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, hazmatPtr2->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, hazmatPtr2->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711, hazmatPtr2->getPose().orientation.w, 0.1);
    }

    //!<  A vector is returned although thermal will always be alone :(
    TEST_F(ObjectFactoryTest, makeThermals)
    {
      ThermalPtrVectorPtr thermalsVectorPtr( new ThermalPtrVector );
      thermalsVectorPtr = objectFactoryPtr->makeObjects<Thermal>(thermalDir1);
      EXPECT_EQ(1, thermalsVectorPtr->size());
      ThermalPtr thermalPtr1 = (*thermalsVectorPtr)[0];
      EXPECT_NEAR(5.75, thermalPtr1->getPose().position.x, 0.01);
      EXPECT_NEAR(5.75, thermalPtr1->getPose().position.y, 0.01);
      EXPECT_NEAR(0.9205, thermalPtr1->getPose().position.z, 0.01);
      EXPECT_NEAR(0, thermalPtr1->getPose().orientation.x, 0.1);
      EXPECT_NEAR(0, thermalPtr1->getPose().orientation.y, 0.1);
      EXPECT_NEAR(-0.70711, thermalPtr1->getPose().orientation.z, 0.1);
      EXPECT_NEAR(+0.70711 , thermalPtr1->getPose().orientation.w, 0.1);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

