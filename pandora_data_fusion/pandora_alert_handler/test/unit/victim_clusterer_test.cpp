// "Copyright [2014] <Chamzas Konstantinos>"

#include "gtest/gtest.h"

#include "alert_handler/victim_clusterer.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    typedef boost::shared_ptr<VictimClusterer> VictimClustererSharedPtr;

    class VictimClustererTest : public ::testing::Test
    {
      protected:

        VictimClustererTest() 
          : victimClustererPtr_( new VictimClusterer(3) )
        {
          Hole::setType("HOLE");
          Thermal::setType("THERMAL");
        }

        /* Helper functions */

        //!< We create manually Thermal1(0, 3.87, 4), 
        //!< Thermal2(1, 0, 2), Hole1(-1, 0, 2)
        void createVariousObjects1(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(0, 3.87, 4));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          ThermalPtr thermalPtr2(new Thermal);
          thermalPtr2->setPose(makePose(1, 0, 3));
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          HolePtr holePtr1(new Hole);
          holePtr1->setPose(makePose(-1, 0, 2));
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
          thermalPtr1->update(thermalPtr1);
          thermalPtr1->update(thermalPtr1);
          HolePtr holePtr1(new Hole);
          holePtr1->setPose(makePose(4, 3, 2));
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          HolePtr holePtr2(new Hole);
          holePtr2->setPose(makePose(0, 4, 2, PI/4));
          holePtr2->setProbability(0.5);
          holePtr2->initializeObjectFilter();
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(holePtr1);
          objConstPtrVectPtr->push_back(holePtr2);
        } 

        //!< We create manually Thermal1(0, 3.87, 4), Thermal2(1, 0, 2), Hole1(-1, 0, 2)
        void createVariousObjects3(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(0, 3.87, 4));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          ThermalPtr thermalPtr2(new Thermal);
          thermalPtr2->setId(5);
          thermalPtr2->setPose(makePose(1, 0, 3));
          thermalPtr2->setProbability(0.5);
          thermalPtr2->initializeObjectFilter();
          thermalPtr2->update(thermalPtr2);
          thermalPtr2->update(thermalPtr2);
          thermalConstPtr_ = thermalPtr2;
          HolePtr holePtr1(new Hole);
          holePtr1->setId(6);
          holePtr1->setPose(makePose(-1, 0, 2));
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          holeConstPtr_ = holePtr1;
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(thermalConstPtr_);
          objConstPtrVectPtr->push_back(holeConstPtr_);
        }

        void createVariousObjects4(ObjectConstPtrVectorPtr objConstPtrVectPtr)
        {
          ThermalPtr thermalPtr1(new Thermal);
          thermalPtr1->setPose(makePose(10, 3, 0));
          thermalPtr1->setProbability(0.5);
          thermalPtr1->initializeObjectFilter();
          HolePtr holePtr1(new Hole);
          holePtr1->setPose(makePose(11, 5, 2, PI/6));
          holePtr1->setProbability(0.5);
          holePtr1->initializeObjectFilter();
          holePtr1->update(holePtr1);
          holePtr1->update(holePtr1);
          HolePtr holePtr2(new Hole);
          holePtr2->setPose(makePose(14, 3, 3));
          holePtr2->setProbability(0.5);
          holePtr2->initializeObjectFilter();
          objConstPtrVectPtr->push_back(thermalPtr1);
          objConstPtrVectPtr->push_back(holePtr1);
          objConstPtrVectPtr->push_back(holePtr2);
        }

        Pose makePose(float x, float y, float z, float yaw = 0)
        {
          geometry_msgs::Pose pose;
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,  0,  yaw);
          return pose;
        }

        /* Accesing private functions */

        ObjectConstPtrVectorVector groupObjects(const ObjectConstPtrVectorPtr& allObjects)
        {
          return victimClustererPtr_->groupObjects(allObjects); 
        }

        geometry_msgs::Point findGroupCenterPoint(const ObjectConstPtrVector& objects)
        {
          return victimClustererPtr_->findGroupCenterPoint(objects);
        }
        
        /* Accesing private variables */
        
        float* getclusterRadius()
        {
          return &victimClustererPtr_->CLUSTER_RADIUS;
        }

        /* variables */

        VictimClustererSharedPtr  victimClustererPtr_;
        HoleConstPtr holeConstPtr_;
        ThermalConstPtr thermalConstPtr_;
    };

    //!< Checks if construstors behave correctly.
    TEST_F(VictimClustererTest, constructor)
    {
      EXPECT_FLOAT_EQ(3, *getclusterRadius());

      victimClustererPtr_.reset( new VictimClusterer(2) );
      EXPECT_FLOAT_EQ(2, *getclusterRadius());
    }

    TEST_F(VictimClustererTest, updateParams)
    {
      victimClustererPtr_->updateParams(3);
      EXPECT_FLOAT_EQ(3, *getclusterRadius());

      victimClustererPtr_->updateParams(-3);
      EXPECT_FLOAT_EQ(-3, *getclusterRadius());
    } 

    TEST_F(VictimClustererTest, findGroupCenterPoint)
    {
      ObjectConstPtrVectorPtr objConstPtrVect1( new ObjectConstPtrVector );
      ObjectConstPtrVectorPtr objConstPtrVect2( new ObjectConstPtrVector );
      geometry_msgs::Point point1; 
      // Thermal1(0, 3.87, 4), Thermal2(1, 0, 2), Hole1(-1, 0, 2)
      createVariousObjects1(objConstPtrVect1);
      point1 = findGroupCenterPoint(*objConstPtrVect1);
      EXPECT_FLOAT_EQ(0, point1.x);
      EXPECT_FLOAT_EQ(1.29, point1.y); 
      // Thermal1(2, 3, 4), Hole1(4, 3, 2), Hole2(0, 4, 2)
      createVariousObjects2(objConstPtrVect2);
      point1 = findGroupCenterPoint(*objConstPtrVect2);
      EXPECT_FLOAT_EQ(2, point1.x);
      EXPECT_NEAR(3.334, point1.y, 0.001);  
    }

    TEST_F(VictimClustererTest, groupObjects)
    {
      ObjectConstPtrVectorPtr objectConstPtrVectorPtr1(new ObjectConstPtrVector);
      ObjectConstPtrVectorVector groupedObjects;

      createVariousObjects3(objectConstPtrVectorPtr1);
      createVariousObjects4(objectConstPtrVectorPtr1);

      victimClustererPtr_->updateParams(5);
      groupedObjects = groupObjects(objectConstPtrVectorPtr1);
      ASSERT_EQ(2, groupedObjects.size());
      EXPECT_EQ(3, groupedObjects[0].size());
      EXPECT_EQ(3, groupedObjects[1].size());

      victimClustererPtr_->updateParams(1.8);
      groupedObjects = groupObjects(objectConstPtrVectorPtr1);
      ASSERT_EQ(6, groupedObjects.size());
      EXPECT_EQ(1, groupedObjects[0].size());
      EXPECT_EQ(1, groupedObjects[1].size());
      EXPECT_EQ(1, groupedObjects[2].size());
      EXPECT_EQ(1, groupedObjects[3].size());
      EXPECT_EQ(1, groupedObjects[4].size());
      EXPECT_EQ(1, groupedObjects[5].size());

      victimClustererPtr_->updateParams(3);
      groupedObjects = groupObjects(objectConstPtrVectorPtr1);
      ASSERT_EQ(5, groupedObjects.size());
      EXPECT_EQ(1, groupedObjects[0].size());
      EXPECT_EQ(2, groupedObjects[1].size());
      EXPECT_EQ(1, groupedObjects[2].size());
      EXPECT_EQ(1, groupedObjects[3].size());
      EXPECT_EQ(1, groupedObjects[4].size());

      victimClustererPtr_->updateParams(3.7);
      groupedObjects = groupObjects(objectConstPtrVectorPtr1);
      ASSERT_EQ(4, groupedObjects.size());
      EXPECT_EQ(1, groupedObjects[0].size());
      EXPECT_EQ(2, groupedObjects[1].size());
      EXPECT_EQ(2, groupedObjects[2].size());
      EXPECT_EQ(1, groupedObjects[3].size());
    } 

    TEST_F(VictimClustererTest, createVictimList)
    {
      ObjectConstPtrVectorPtr objectConstPtrVectorPtr1(new ObjectConstPtrVector);
      VictimPtrVector victims;

      createVariousObjects3(objectConstPtrVectorPtr1);
      createVariousObjects4(objectConstPtrVectorPtr1);

      victimClustererPtr_->updateParams(5);
      victims = victimClustererPtr_->createVictimList(objectConstPtrVectorPtr1);
      ASSERT_EQ(2, victims.size());
      EXPECT_EQ(2, victims[0]->getObjects().size());
      EXPECT_EQ(2, victims[1]->getObjects().size());
      EXPECT_EQ(victims[0]->getObjects().at(0)->getId(), holeConstPtr_->getId());
      EXPECT_EQ(victims[0]->getObjects().at(0)->getType(), Hole::getObjectType());
      EXPECT_EQ(victims[0]->getObjects().at(1)->getId(), thermalConstPtr_->getId());
      EXPECT_EQ(victims[0]->getObjects().at(1)->getType(), Thermal::getObjectType());

      victimClustererPtr_->updateParams(1.8);
      victims = victimClustererPtr_->createVictimList(objectConstPtrVectorPtr1);
      ASSERT_EQ(6, victims.size());
      EXPECT_EQ(1, victims[0]->getObjects().size());
      EXPECT_EQ(1, victims[1]->getObjects().size());
      EXPECT_EQ(1, victims[2]->getObjects().size());
      EXPECT_EQ(1, victims[3]->getObjects().size());
      EXPECT_EQ(1, victims[4]->getObjects().size());
      EXPECT_EQ(1, victims[5]->getObjects().size());

      victimClustererPtr_->updateParams(3);
      victims = victimClustererPtr_->createVictimList(objectConstPtrVectorPtr1);
      ASSERT_EQ(5, victims.size());
      EXPECT_EQ(1, victims[0]->getObjects().size());
      EXPECT_EQ(2, victims[1]->getObjects().size());
      EXPECT_EQ(1, victims[2]->getObjects().size());
      EXPECT_EQ(1, victims[3]->getObjects().size());
      EXPECT_EQ(1, victims[4]->getObjects().size());

      victimClustererPtr_->updateParams(3.7);
      victims = victimClustererPtr_->createVictimList(objectConstPtrVectorPtr1);
      ASSERT_EQ(4, victims.size());
      EXPECT_EQ(1, victims[0]->getObjects().size());
      EXPECT_EQ(2, victims[1]->getObjects().size());
      EXPECT_EQ(2, victims[2]->getObjects().size());
      EXPECT_EQ(1, victims[3]->getObjects().size());
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

