#include "alert_handler/victim.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

double pi= M_PI;

class VictimTest : public ::testing::Test {
 
  protected:

  VictimTest()
  {
  }


  virtual void SetUp() {
    
  Victim1.reset( new Victim); 
  Victim2.reset( new Victim);
  ObjConstPtrVectPtr3.reset(new ObjectConstPtrVector);
  //create the Objectvector1 and fill it with various objects 
  //Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) (yaw=0) ApproachDist=5
  createVariousObjects1(ObjConstPtrVect1);
  //create the Objectvector2 and fill it with various objects 
  //Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4 ApproachDist=6
  createVariousObjects2( ObjConstPtrVect2);
  //create the Objectvector3 and fill it with various objects 
  //Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4 ApproachDist=6
  createVariousObjects3(ObjConstPtrVectPtr3);


  } 
  
  virtual void TearDown()
  {
    
    clear();
  }
 //* helper functions 
 
// We create manually Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) (no rotation)
  void createVariousObjects1(ObjectConstPtrVector   &ObjConstPtrVectPtr1)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 4, TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(4, 3, 2, TpaPtr2);
    HolePtr HolePtr1(new Hole);
    setPose(1, 2, 0, HolePtr1);
    ObjConstPtrVectPtr1.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr1.push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVectPtr1.push_back(HoleConstPtr(HolePtr1));
  } 
// We create manually Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4
  void createVariousObjects2(ObjectConstPtrVector   &ObjConstPtrVectPtr2)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 4, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(4, 3, 2, HolePtr1, pi/4);
    HolePtr HolePtr2(new Hole);
    setPose(0, 4, 2, HolePtr2);
    ObjConstPtrVectPtr2.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr2.push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVectPtr2.push_back(HoleConstPtr(HolePtr2));
  } 

  
// We create manually Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4
  void createVariousObjects3(ObjectConstPtrVectorPtr   ObjConstPtrVectPtr2)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 4, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(4, 3, 2, HolePtr1, pi/4);
    HolePtr HolePtr2(new Hole);
    setPose(0, 4, 2, HolePtr2);
    ObjConstPtrVectPtr3->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr3->push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVectPtr3->push_back(HoleConstPtr(HolePtr2));
  } 
  
  void setPose ( float x, float y, float z, ObjectPtr Object, float yaw=0)
  {
    //z is set zero because this test is written for 2d
    geometry_msgs::Pose pose1;
    pose1.position.x=x;
    pose1.position.y=y;
    pose1.position.z=0;
    pose1.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    Object->setPose(pose1);
  }
  
  /* accesing private functions*/
   void updatePose(VictimPtr Victim, geometry_msgs::Pose poseVic, float dist)
  {
    Victim->updatePose(poseVic, dist);
  }
  
  
  int findRepresentativeObject(VictimPtr Victim)
  {
    return Victim->findRepresentativeObject();
  }
  
  void clear()
  {
    
    Victim::lastVictimId_=0;
  }
    
  /* Accesors to private variables*/
  
  
  int* getLastVictimId(VictimPtr Victim)
  {
    return &Victim->lastVictimId_;
  }
  
  ObjectConstPtrVector getObjects(VictimPtr Victim)
  { 
  
    return Victim->objects_;
  }
 /* Variables */
 
  VictimPtr Victim1;
  VictimPtr Victim2;
  ObjectConstPtrVector  ObjConstPtrVect1;
  ObjectConstPtrVector  ObjConstPtrVect2;
  ObjectConstPtrVectorPtr  ObjConstPtrVectPtr3;
};


//~ // Checks  if the Construstors behave Correctly 
TEST_F(VictimTest, Constructor)
  {
  
  clear(); 
  Victim1.reset( new Victim); 
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(1, *getLastVictimId(Victim1));
  EXPECT_EQ(-1, Victim1->getSelectedObjectIndex());
 
    
  Victim2.reset( new Victim);
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(2, *getLastVictimId(Victim1));
  EXPECT_EQ(2, *getLastVictimId(Victim2));
  EXPECT_EQ(-1, Victim2->getSelectedObjectIndex());
  clear();
    
  }
    

TEST_F(VictimTest, isSameObject)
  {
  
  
  EXPECT_EQ(2, *getLastVictimId(Victim1));
  // MUST Be CHECKED ALSO FOR 3d WHEN SLAM BECAMES 3D
  setPose(5, 1, 0, Victim1);
  setPose(0, 1, 0, Victim2);
  EXPECT_TRUE(Victim1->isSameObject(VictimConstPtr(Victim2), 7));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2), -5));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2), -1));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2), 1));
  clear();
  }

  
TEST_F(VictimTest, updatePose)
{
   geometry_msgs::Pose poseVic;
   poseVic.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
   poseVic.position.x=1;
   poseVic.position.y=0;
   poseVic.position.z=0;
   updatePose(Victim1, poseVic, 5);
   EXPECT_EQ(1, Victim1->getPose().position.x);
   EXPECT_EQ(0, Victim1->getPose().position.y);
   EXPECT_EQ(0, Victim1->getPose().position.z);
   EXPECT_EQ(6, Victim1->getApproachPose().position.x);
   EXPECT_EQ(0, Victim1->getApproachPose().position.y);
   EXPECT_EQ(0, Victim1->getApproachPose().position.z);
   clear();
}

TEST_F(VictimTest, setObjects)
  {
  
  ObjectConstPtrVector  ObjConstPtrVect1;
  //create the Objectvector and fill it with various objects 
  //Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) (yaw=0) ApproachDist=5
  createVariousObjects1((ObjConstPtrVect1));
  Victim1->setObjects(ObjConstPtrVect1, 5);
  EXPECT_EQ(3, getObjects(Victim1).size());
  EXPECT_EQ(2, findRepresentativeObject(Victim1));
  EXPECT_EQ(1, Victim1->getPose().position.x);
  EXPECT_EQ(2, Victim1->getPose().position.y);
  EXPECT_EQ(0, Victim1->getPose().position.z);
  EXPECT_EQ(6, Victim1->getApproachPose().position.x);
  EXPECT_EQ(2, Victim1->getApproachPose().position.y);
  EXPECT_EQ(0, Victim1->getApproachPose().position.z);
  
  
  
  ObjectConstPtrVector  ObjConstPtrVect2;
  //create the Objectvector and fill it with various objects 
  //Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4 ApproachDist=6
  createVariousObjects2((ObjConstPtrVect2));
  Victim2->setObjects(ObjConstPtrVect2, 6);
  EXPECT_EQ(3, ObjConstPtrVect2.size());
  EXPECT_EQ(1, findRepresentativeObject(Victim2));
  EXPECT_EQ(4, Victim2->getPose().position.x);
  EXPECT_EQ(3, Victim2->getPose().position.y);
  EXPECT_EQ(0, Victim2->getPose().position.z);
  EXPECT_FLOAT_EQ(8.24264, Victim2->getApproachPose().position.x);
  EXPECT_FLOAT_EQ(7.24264, Victim2->getApproachPose().position.y);
  EXPECT_EQ(0, Victim2->getApproachPose().position.z);
  
  }

TEST_F(VictimTest, eraseObjectAt)
  {
    
  //Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0)
  Victim1->setObjects(ObjConstPtrVect1, 5);
  // A bigger index should erase the last element
  Victim1->eraseObjectAt(20, 5);
  // the hole is erased
  EXPECT_EQ(2, Victim1->getPose().position.x);
  EXPECT_EQ(3, Victim1->getPose().position.y);
  EXPECT_EQ(0, Victim1->getPose().position.z);
  EXPECT_EQ(0, Victim1->getSelectedObjectIndex());
  EXPECT_EQ(2, getObjects(Victim1).size());
  //the first tpa is erased 
  Victim1->eraseObjectAt(0, 5);
  EXPECT_EQ(4, Victim1->getPose().position.x);
  EXPECT_EQ(3, Victim1->getPose().position.y);
  EXPECT_EQ(0, Victim1->getPose().position.z);
  EXPECT_EQ(0, Victim1->getSelectedObjectIndex());
  EXPECT_EQ(1, getObjects(Victim1).size());
  // the last element is erased
  Victim1->eraseObjectAt(0, 5);
  EXPECT_EQ(-1, Victim1->getSelectedObjectIndex());
  EXPECT_EQ(0, getObjects(Victim1).size());
  
  
//Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)

  Victim2->setObjects(ObjConstPtrVect2, 6);
  EXPECT_EQ(1, findRepresentativeObject(Victim2));
  // erase the medium Hole
  Victim2->eraseObjectAt(1, 3);
  EXPECT_EQ(0, Victim2->getPose().position.x);
  EXPECT_EQ(4, Victim2->getPose().position.y);
  EXPECT_EQ(0, Victim2->getPose().position.z);
  EXPECT_EQ(1, Victim2->getSelectedObjectIndex());
  EXPECT_EQ(2, getObjects(Victim2).size());
  // erase the last hole
  Victim2->eraseObjectAt(1, 3);
  EXPECT_EQ(1, getObjects(Victim2).size());
  EXPECT_EQ(0, Victim2->getSelectedObjectIndex());
  // erase the last object(tpa)
  Victim2->eraseObjectAt(8, 3);
  EXPECT_EQ(0, getObjects(Victim2).size());
  EXPECT_EQ(-1, Victim2->getSelectedObjectIndex());
  
}

//create the Objectvector1 and fill it with various objects 
//Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) (yaw=0) ApproachDist=5
//createVariousObjects1((ObjConstPtrVect1));
//create the Objectvector2 and fill it with various objects 
//Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) Yaw=pi/4 ApproachDist=6
//createVariousObjects2((ObjConstPtrVect2));

TEST_F(VictimTest, SanityCheck)
{
// Objectvector1 Tpa1(2, 3, 4) Tpa2(4, 3, 2) Hole1(1, 2, 0) 

// Objectvector2 Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2) 

// ObjectvectorPtr3 Tpa1(2, 3, 4) Hole1(4, 3, 2) Hole2(0, 4, 2)

  Victim1->setObjects(ObjConstPtrVect1, 5);
  Victim1->sanityCheck(ObjConstPtrVectPtr3, 0.5, 3);
  EXPECT_EQ(1, Victim1->getObjects().size());
  EXPECT_EQ(0, Victim1->getSelectedObjectIndex());
  
  
  Victim1->setObjects(ObjConstPtrVect1, 5);
  Victim1->sanityCheck(ObjConstPtrVectPtr3, 10, 3);
  EXPECT_EQ(3, Victim1->getObjects().size());
  EXPECT_EQ(2, Victim1->getSelectedObjectIndex());
  
  Victim1->setObjects(ObjConstPtrVect1, 5);
  Victim1->sanityCheck(ObjConstPtrVectPtr3, -3, 3);
  EXPECT_EQ(0, Victim1->getObjects().size());
  EXPECT_EQ(-1, Victim1->getSelectedObjectIndex());
  
  
  Victim1->setObjects(ObjConstPtrVect1, 5);
  Victim1->sanityCheck(ObjConstPtrVectPtr3, 10, 3);
  EXPECT_EQ(3, Victim1->getObjects().size());
  EXPECT_EQ(2, Victim1->getSelectedObjectIndex());
  

}
  
}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

