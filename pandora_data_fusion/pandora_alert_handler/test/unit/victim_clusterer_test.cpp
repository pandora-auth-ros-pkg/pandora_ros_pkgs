// "Copyright [2014] <Chamzas Konstantinos>"
#include "alert_handler/victim_clusterer.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

double pi= M_PI;
typedef boost::shared_ptr<VictimClusterer> VictimClustererSharedPtr;
class VictimClustererTest : public ::testing::Test {
 
  protected:

  VictimClustererTest(): VictimClustererPtr1(new VictimClusterer(3, 3))
  {
  }
/* helper functions*/
 
// We create manually Tpa1(0, 3.87, 4) Tpa2(1, 0, 2) Hole1(-1, 0, 2) (no rotation)
  void createVariousObjects1(ObjectConstPtrVector*   ObjConstPtrVect1)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(0, 3.87, 4,  TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(1, 0, 3,  TpaPtr2);
    HolePtr HolePtr1(new Hole);
    setPose(-1, 0, 2, HolePtr1);
    ObjConstPtrVect1->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect1->push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVect1->push_back(HoleConstPtr(HolePtr1));
  } 
// We create manually Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4
  void createVariousObjects2(ObjectConstPtrVector*   ObjConstPtrVect2)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 4, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(4, 3, 2, HolePtr1);
    HolePtr HolePtr2(new Hole);
    setPose(0, 4, 2, HolePtr2);
    ObjConstPtrVect2->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect2->push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVect2->push_back(HoleConstPtr(HolePtr2));
  } 

  
//  We create manually Tpa1(0, 3.87, 4) Tpa2(1, 0, 2) Hole1(-1, 0, 2)
  void createVariousObjects3(ObjectConstPtrVectorPtr   ObjConstPtrVectPtr1)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(0, 3.87, 4,  TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(1, 0, 3,  TpaPtr2);
    HolePtr HolePtr1(new Hole);
    setPose(-1, 0, 2, HolePtr1);
    ObjConstPtrVectPtr1->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr1->push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVectPtr1->push_back(HoleConstPtr(HolePtr1));
  }
  
  void createVariousObjects4(ObjectConstPtrVectorPtr   ObjConstPtrVectPtr2)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(10, 3, 0, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(11, 5, 2, HolePtr1, pi/4);
    HolePtr HolePtr2(new Hole);
    setPose(14, 3, 3, HolePtr2);
    ObjConstPtrVectPtr2->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr2->push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVectPtr2->push_back(HoleConstPtr(HolePtr2));
  }
  
   
void setPose ( float  x, float  y, float  z, ObjectPtr Object, float yaw = 0)
  {
    // z is set zero because this test is written for 2d
    geometry_msgs::Pose pose1;
    pose1.position.x = x;
    pose1.position.y = y;
    pose1.position.z = 0;
    pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,  0,  yaw);
    Object->setPose(pose1);
  }

  
/* accesing private functions*/
  ObjectConstPtrVectorVector groupObjects(const ObjectConstPtrVectorPtr& allObjects,
    VictimClustererSharedPtr VictimClustererPtr1)
  {
    return VictimClustererPtr1->groupObjects( allObjects); 
  }
  
  geometry_msgs::Point findGroupCenterPoint(const ObjectConstPtrVector& objects, 
  VictimClustererSharedPtr VictimClustererPtr1 )
  {
    return VictimClustererPtr1->findGroupCenterPoint( objects);
  }
/* accesing private variables*/
  float getclusterRadius(VictimClustererSharedPtr VictimClustererPtr )
  {
    return VictimClustererPtr->CLUSTER_RADIUS;
  }
  float getApproachDist(VictimClustererSharedPtr VictimClustererPtr)
  {
    return VictimClustererPtr->APPROACH_DIST;
  }
  
   
  ObjectConstPtrVector getObjects(VictimPtr Victim)
  { 
  
    return Victim->objects_;
  }


/* variables */

VictimClustererSharedPtr  VictimClustererPtr1;

};
// Checks  if the Construstors behave Correctly 
TEST_F(VictimClustererTest,  Constructor)
{
  EXPECT_FLOAT_EQ( 3, getclusterRadius(VictimClustererPtr1));
  EXPECT_FLOAT_EQ( 3, getApproachDist(VictimClustererPtr1)); 
  
  VictimClustererPtr1.reset(new VictimClusterer( 2, 1));
  EXPECT_FLOAT_EQ( 2, getclusterRadius(VictimClustererPtr1));
  EXPECT_FLOAT_EQ( 1, getApproachDist(VictimClustererPtr1)); 
}

TEST_F(VictimClustererTest,  updateParams)
{
  VictimClustererPtr1->updateParams(3, 1);
  EXPECT_FLOAT_EQ( 3, getclusterRadius(VictimClustererPtr1));
  EXPECT_FLOAT_EQ( 1, getApproachDist(VictimClustererPtr1));
  
  VictimClustererPtr1->updateParams(-3, -2);
  EXPECT_FLOAT_EQ( -3, getclusterRadius(VictimClustererPtr1));
  EXPECT_FLOAT_EQ( -2, getApproachDist(VictimClustererPtr1)); 
} 

TEST_F(VictimClustererTest,  findGroupCenterPoint)
{
  ObjectConstPtrVector  ObjConstPtrVect1;
  ObjectConstPtrVector  ObjConstPtrVect2;
  geometry_msgs::Point Point1; 
  // Tpa1(0, 3.87, 4) Tpa2(1, 0, 2) Hole1(-1, 0, 2)
  createVariousObjects1((&ObjConstPtrVect1));
  Point1 = findGroupCenterPoint(ObjConstPtrVect1, VictimClustererPtr1);
  EXPECT_FLOAT_EQ( 0, Point1.x);
  EXPECT_FLOAT_EQ( 1.29, Point1.y); 
  // Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)
  createVariousObjects2((&ObjConstPtrVect2));
  Point1 = findGroupCenterPoint(ObjConstPtrVect2, VictimClustererPtr1);
  EXPECT_FLOAT_EQ( 2, Point1.x);
  EXPECT_NEAR( 3.334, Point1.y, 0.001);  
  
}

TEST_F(VictimClustererTest,  groupObjects)
{
  ObjectConstPtrVectorPtr ObjectConstPtrVectorPtr1(new ObjectConstPtrVector);
  ObjectConstPtrVectorVector GroupedObjects;
  
  createVariousObjects3(ObjectConstPtrVectorPtr1);
  createVariousObjects4(ObjectConstPtrVectorPtr1);
  
  VictimClustererPtr1->updateParams( 5, 3);
  GroupedObjects = groupObjects(ObjectConstPtrVectorPtr1, VictimClustererPtr1);
  EXPECT_EQ(2, GroupedObjects.size());
  EXPECT_EQ(3, GroupedObjects[0].size());
  EXPECT_EQ(3, GroupedObjects[1].size());
  
  VictimClustererPtr1->updateParams( 1.8, 3);
  GroupedObjects = groupObjects(ObjectConstPtrVectorPtr1, VictimClustererPtr1);
  EXPECT_EQ(6, GroupedObjects.size());
  EXPECT_EQ(1, GroupedObjects[0].size());
  EXPECT_EQ(1, GroupedObjects[1].size());
  
  VictimClustererPtr1->updateParams( 3, 3);
  GroupedObjects = groupObjects(ObjectConstPtrVectorPtr1, VictimClustererPtr1);
  EXPECT_EQ(4, GroupedObjects.size());
  EXPECT_EQ(1, GroupedObjects[0].size());
  EXPECT_EQ(2, GroupedObjects[1].size());
  EXPECT_EQ(2, GroupedObjects[2].size());
  EXPECT_EQ(1, GroupedObjects[3].size());
  
  VictimClustererPtr1->updateParams( 3.7, 3);
  GroupedObjects = groupObjects(ObjectConstPtrVectorPtr1, VictimClustererPtr1);
  EXPECT_EQ(3, GroupedObjects.size());
  EXPECT_EQ(1, GroupedObjects[0].size());
  EXPECT_EQ(2, GroupedObjects[1].size());
  EXPECT_EQ(3, GroupedObjects[2].size());
} 



TEST_F(VictimClustererTest,  createVictimList)
{
  ObjectConstPtrVectorPtr ObjectConstPtrVectorPtr1(new ObjectConstPtrVector);
  VictimPtrVector Victims;
  
  createVariousObjects3(ObjectConstPtrVectorPtr1);
  createVariousObjects4(ObjectConstPtrVectorPtr1);
  
  VictimClustererPtr1->updateParams( 5, 3);
  Victims = VictimClustererPtr1->createVictimList(ObjectConstPtrVectorPtr1);
  EXPECT_EQ(2, Victims.size());
  EXPECT_EQ(3, getObjects(Victims[0]).size());
  EXPECT_EQ(3, getObjects(Victims[1]).size());
  
  VictimClustererPtr1->updateParams( 1.8, 3);
  Victims    = VictimClustererPtr1->createVictimList(ObjectConstPtrVectorPtr1);
  EXPECT_EQ(6, Victims.size());
  EXPECT_EQ(1, getObjects(Victims[0]).size());
  EXPECT_EQ(1, getObjects(Victims[1]).size());
  
  VictimClustererPtr1->updateParams( 3, 3);
  Victims    = VictimClustererPtr1->createVictimList(ObjectConstPtrVectorPtr1);
  EXPECT_EQ(4, Victims.size());
  EXPECT_EQ(1, getObjects(Victims[0]).size());
  EXPECT_EQ(2, getObjects(Victims[1]).size());
  EXPECT_EQ(2, getObjects(Victims[2]).size());
  EXPECT_EQ(1, getObjects(Victims[3]).size());
  
  VictimClustererPtr1->updateParams( 3.7, 3);
  Victims    = VictimClustererPtr1->createVictimList(ObjectConstPtrVectorPtr1);
  EXPECT_EQ(3, Victims.size());
  EXPECT_EQ(1, getObjects(Victims[0]).size());
  EXPECT_EQ(2, getObjects(Victims[1]).size());
  EXPECT_EQ(3, getObjects(Victims[2]).size());
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

