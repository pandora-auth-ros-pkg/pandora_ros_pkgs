// "Copyright [2014] <Chamzas Konstantinos>"
#include "alert_handler/victim.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

double pi= M_PI;

class VictimTest : public ::testing::Test
{ 
 protected:

  VictimTest()
  {
  }

  virtual void SetUp()
  {  
    victim1.reset( new Victim ); 
    victim2.reset( new Victim );

    filterModelPtr.reset( new FilterModel );
    Victim::setHoleModel(filterModelPtr);
    Victim::setTpaModel(filterModelPtr);

    objConstPtrVectPtr3.reset( new ObjectConstPtrVector );

    //!< create the Objectvector1 and fill it with various objects 
    //!< Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) 
    //!< (yaw = 0) ApproachDist = 5
    createVariousObjects1(&objConstPtrVect1);
    //!< create the Objectvector2 and fill it with various objects 
    //!< Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) 
    //!< (yaw = pi/4) ApproachDist = 6
    createVariousObjects2(&objConstPtrVect2);
    //!< create the Objectvector3 and fill it with various objects 
    //!< Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) 
    //!< (yaw = pi/4) ApproachDist = 6
    createVariousObjects3(objConstPtrVectPtr3);
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
 
  //!< We create manually tpa1(2, 3, 4) tpa2(4, 3, 2) hole1(1, 2, 0) 
  //!< Yaw = 0 (no rotation)
  void createVariousObjects1(ObjectConstPtrVector* objConstPtrVectPtr)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(2, 3, 4, tpaPtr1);
    tpaPtr1->update(tpaPtr1, filterModelPtr);
    tpaPtr1->update(tpaPtr1, filterModelPtr);
    TpaPtr tpaPtr2(new Tpa);
    setPose(4, 3, 2, tpaPtr2);
    HolePtr holePtr1(new Hole);
    setPose(1, 2, 0, holePtr1);
    objConstPtrVectPtr->push_back(tpaPtr1);
    objConstPtrVectPtr->push_back(tpaPtr2);
    objConstPtrVectPtr->push_back(holePtr1);
  }

  //!< We create manually tpa1(2, 3, 4) hole1(4, 3, 2) hole2(0, 4, 2) 
  //!< Yaw = pi/4
  void createVariousObjects2(ObjectConstPtrVector* objConstPtrVectPtr)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(2, 3, 4, tpaPtr1);
    HolePtr holePtr1(new Hole);
    setPose(4, 3, 2, holePtr1, pi/4);
    holePtr1->update(holePtr1, filterModelPtr);
    holePtr1->update(holePtr1, filterModelPtr);
    HolePtr holePtr2(new Hole);
    setPose(0, 4, 2, holePtr2);
    objConstPtrVectPtr->push_back(tpaPtr1);
    objConstPtrVectPtr->push_back(holePtr1);
    objConstPtrVectPtr->push_back(holePtr2);
  } 

  //!< We create manually tpa1(2, 3, 4) tpa2(4, 3, 2) tpa3(0, 4, 2) 
  //!< Yaw = pi/4
  void createVariousObjects3(ObjectConstPtrVectorPtr objConstPtrVectPtr)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(2, 3, 4, tpaPtr1);
    TpaPtr tpaPtr2(new Tpa);
    setPose(4, 3, 2, tpaPtr2, pi/4);
    TpaPtr tpaPtr3(new Tpa);
    setPose(0, 4, 2, tpaPtr3);
    tpaPtr3->update(tpaPtr3, filterModelPtr);
    tpaPtr3->update(tpaPtr3, filterModelPtr);
    objConstPtrVectPtr->push_back(tpaPtr1);
    objConstPtrVectPtr->push_back(tpaPtr2);
    objConstPtrVectPtr->push_back(tpaPtr3);
  } 
  //!< We create manually tpa1(2, 3, 4) tpa2(4, 3, 2) tpa3(0, 4, 2) 
  //!< Yaw = pi/4
  void setPose(float x, float y, float z, ObjectPtr object, float yaw = 0)
  {
    geometry_msgs::Pose pose1;
    pose1.position.x = x;
    pose1.position.y = y;
    pose1.position.z = z;
    pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    object->setPose(pose1);
    object->initializeObjectFilter(0.5, 0.5, 0.5); 
  }
  
  /* Accessors to private functions */

  void updatePose(VictimPtr victim, geometry_msgs::Pose poseVic, float dist)
  {
    victim->updatePose(poseVic, dist);
  }
  
  int findRepresentativeObject(VictimPtr victim)
  {
    return victim->findRepresentativeObject();
  }
  
  void clear()
  {
    Victim::lastVictimId_ = 0;
  }
    
  /* Accessors to private variables */
  
  int* getLastVictimId(VictimPtr victim)
  {
    return &(victim->lastVictimId_);
  }
  
  ObjectConstPtrVector getObjects(VictimPtr victim)
  { 
    return victim->objects_;
  }

  /* Variables */
 
  FilterModelPtr filterModelPtr;
  VictimPtr victim1;
  VictimPtr victim2;
  ObjectConstPtrVector objConstPtrVect1;
  ObjectConstPtrVector objConstPtrVect2;
  ObjectConstPtrVectorPtr objConstPtrVectPtr3;

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
  EXPECT_EQ(2, *getLastVictimId(victim1));
  setPose(5, 1, 0, victim1);
  setPose(0, 1, 2, victim2);
  EXPECT_TRUE(victim1->isSameObject(victim2, 7));
  EXPECT_FALSE(victim1->isSameObject(victim2, -5));
  EXPECT_FALSE(victim1->isSameObject(victim2, 1));
}

TEST_F(VictimTest, updatePose)
{
  geometry_msgs::Pose poseVic;
  poseVic.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
  poseVic.position.x = 1;
  poseVic.position.y = 0;
  poseVic.position.z = 0;
  updatePose(victim1, poseVic, 5);
  EXPECT_EQ(1, victim1->getPose().position.x);
  EXPECT_EQ(0, victim1->getPose().position.y);
  EXPECT_EQ(0, victim1->getPose().position.z);
  EXPECT_EQ(6, victim1->getApproachPose().position.x);
  EXPECT_EQ(0, victim1->getApproachPose().position.y);
  EXPECT_EQ(0, victim1->getApproachPose().position.z);
}

TEST_F(VictimTest, setObjects)
{  
  victim1->setObjects(objConstPtrVect1, 5);
  // There will be 2 objects left in victim1. One Hole and one Tpa type.
  EXPECT_EQ(2, getObjects(victim1).size());
  EXPECT_EQ("hole", getObjects(victim1)[0]->getType());
  EXPECT_EQ("tpa", getObjects(victim1)[1]->getType());
  // Hole objects are prefered over Tpa objects regarding representatives.
  EXPECT_EQ(0, findRepresentativeObject(victim1));
  EXPECT_EQ(1, victim1->getPose().position.x);
  EXPECT_EQ(2, victim1->getPose().position.y);
  EXPECT_EQ(0, victim1->getPose().position.z);
  EXPECT_EQ(6, victim1->getApproachPose().position.x);
  EXPECT_EQ(2, victim1->getApproachPose().position.y);
  EXPECT_EQ(0, victim1->getApproachPose().position.z);
  // victim1's Tpa object should be the tpaPtr1! tpaPtr1 had more confidence
  // in its belief about position that tpaPtr2, so it was prefered during
  // setObject()
  EXPECT_GE(objConstPtrVect1[0]->getVarianceX(), 
      getObjects(victim1)[1]->getVarianceX());
  EXPECT_GT(objConstPtrVect1[1]->getVarianceX() - 
      getObjects(victim1)[1]->getVarianceX(),
      objConstPtrVect1[0]->getVarianceX() -
      getObjects(victim1)[1]->getVarianceX());
  // tpaPtr1 object should be updated so that its position is 
  // closer to tpaPtr2's
  EXPECT_GT(distance(objConstPtrVect1[0], objConstPtrVect1[1]), 
      distance(getObjects(victim1)[1], objConstPtrVect1[1]));
  EXPECT_GT(distance(objConstPtrVect1[0], objConstPtrVect1[1]), 
      distance(getObjects(victim1)[1], objConstPtrVect1[0]));
  
  victim2->setObjects(objConstPtrVect2, 6);
  // There will be 2 objects left in victim1. One Hole and one Tpa type.
  EXPECT_EQ(2, getObjects(victim2).size());
  EXPECT_EQ("hole", getObjects(victim2)[0]->getType());
  EXPECT_EQ("tpa", getObjects(victim2)[1]->getType());
  // Hole objects are prefered over Tpa objects regarding representatives.
  EXPECT_EQ(0, findRepresentativeObject(victim1));
  EXPECT_GT(4, victim2->getPose().position.x);
  EXPECT_LT(0, victim2->getPose().position.x);
  EXPECT_LT(3, victim2->getPose().position.y);
  EXPECT_GT(4, victim2->getPose().position.y);
  EXPECT_EQ(2, victim2->getPose().position.z);
  EXPECT_EQ(0, victim2->getApproachPose().position.z);  
  // victim1's Tpa object should be the tpaPtr1! tpaPtr1 had more confidence
  // in its belief about position that tpaPtr2, so it was prefered during
  // setObject()
  EXPECT_GE(objConstPtrVect2[1]->getVarianceX(), 
      getObjects(victim2)[0]->getVarianceX());
  EXPECT_GT(objConstPtrVect2[2]->getVarianceX() - 
      getObjects(victim2)[0]->getVarianceX(),
      objConstPtrVect2[1]->getVarianceX() -
      getObjects(victim2)[0]->getVarianceX());
  // tpaPtr1 object should be updated so that its position is 
  // closer to tpaPtr2's
  EXPECT_GT(distance(objConstPtrVect2[1], objConstPtrVect2[2]), 
      distance(getObjects(victim2)[0], objConstPtrVect2[2]));
  EXPECT_GT(distance(objConstPtrVect2[1], objConstPtrVect2[2]), 
      distance(getObjects(victim2)[0], objConstPtrVect2[1]));
}

TEST_F(VictimTest, eraseObjectAt)
{  
  // Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0)
  victim1->setObjects(objConstPtrVect1, 5);
  victim1->eraseObjectAt(1, 5);
  // tpa1 is erased.
  EXPECT_EQ(1, victim1->getPose().position.x);
  EXPECT_EQ(2, victim1->getPose().position.y);
  EXPECT_EQ(0, victim1->getPose().position.z);
  EXPECT_EQ(0, victim1->getSelectedObjectIndex());
  EXPECT_EQ(1, getObjects(victim1).size());
  // hole1 is erased 
  victim1->eraseObjectAt(0, 5);
  EXPECT_EQ(0, getObjects(victim1).size());
  
  // Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)
  victim2->setObjects(objConstPtrVect2, 6);
  EXPECT_EQ(0, findRepresentativeObject(victim2));
  // Erase hole1. Now tpa1 is the representative object.
  victim2->eraseObjectAt(0, 3);
  EXPECT_EQ(2, victim2->getPose().position.x);
  EXPECT_EQ(3, victim2->getPose().position.y);
  EXPECT_EQ(4, victim2->getPose().position.z);
  EXPECT_EQ(0, victim2->getSelectedObjectIndex());
  EXPECT_EQ(1, getObjects(victim2).size());
  // Erase the last object (tpa1).
  victim2->eraseObjectAt(1, 3);
  EXPECT_EQ(0, getObjects(victim2).size());
  EXPECT_EQ(-1, victim2->getSelectedObjectIndex());
}
  
}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

