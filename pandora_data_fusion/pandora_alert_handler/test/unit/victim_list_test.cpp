#include "alert_handler/victim_list.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class VictimListTest : public ::testing::Test {
 
  protected:

  VictimListTest():VictimList1(),VictimList2(2,1,1.5,1),Victim1( new Victim),
  Victim2( new Victim),Victim3( new Victim),Victim4( new Victim),
  Victim5( new Victim),Victim6( new Victim) { }


  virtual void SetUp()
   {
     
     createVariousObjects1(ObjConstPtrVect1);
     createVariousObjects2(ObjConstPtrVect2);
     createVariousObjects3(ObjConstPtrVect3);
     createVariousObjects4(ObjConstPtrVect4);
     
     fillVictim(Victim1,ObjConstPtrVect1);
     fillVictim(Victim2,ObjConstPtrVect2);
     fillVictim(Victim3,ObjConstPtrVect3);
     fillVictim(Victim4,ObjConstPtrVect4);
     
     fillVictimList(VictimList1);
    
  } 
  
  virtual void TearDown()
  {
    VictimList1.clear();
    
  }
  
  
// helper functions
void fillVictimList( VictimList &VictimList1 )
  {
    VictimList1.clear();
    VictimList1.add(Victim1);
    VictimList1.add(Victim2);
    VictimList1.add(Victim3);
  }
  
void fillVictim(VictimPtr Victim, ObjectConstPtrVector &ObjConstPtrVect)
{
  Victim->setObjects(ObjConstPtrVect, 5);
}

//Tpa1(-1, 0, 0)  Tpa2(1, 0, 0) Hole1(0 , 1, 0)  
  void createVariousObjects1(ObjectConstPtrVector   &ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(-1, 0, 0, TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(1, 0, 0, TpaPtr2);
    HolePtr HolePtr1(new Hole);
    setPose(0, 1, 0, HolePtr1);
    ObjConstPtrVect.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect.push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVect.push_back(HoleConstPtr(HolePtr1));
  }
//Tpa1(2, 3, 0) Hole1(3, 3, 0) Hole2(2, 2.5, 0) 
  void createVariousObjects2(ObjectConstPtrVector   &ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 0, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(3, 3, 0, HolePtr1);
    HolePtr HolePtr2(new Hole);
    setPose(2, 2.5, 0, HolePtr2);
    ObjConstPtrVect.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect.push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVect.push_back(HoleConstPtr(HolePtr2));
  }
//Tpa1(2.7, 3, 0) Tpa2(3, 3, 0) 
  void createVariousObjects3(ObjectConstPtrVector   &ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(3, 3, 0, TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(3, 3, 0, TpaPtr2);
    ObjConstPtrVect3.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect3.push_back(TpaConstPtr(TpaPtr2));
  } 
  
// Tpa1(3, 3, 0) Hole1(10, 3, 0) 
  void createVariousObjects4(ObjectConstPtrVector   &ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(3, 3, 0, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(10, 3, 0, HolePtr1);
    ObjConstPtrVect3.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect3.push_back(HoleConstPtr(HolePtr1));
  } 
  
  void setPose ( float x, float y, float z, ObjectPtr Object, float yaw =0)
  {
    //z is set zero because this test is written for 2d
    geometry_msgs::Pose pose1;
    pose1.position.x=x;
    pose1.position.y=y;
    pose1.position.z=0;
    pose1.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    Object->setPose(pose1);
  }
// accesing private variables
  VictimList::iterator getCurrentVictimIt(VictimList VictimList1)
  {
    return VictimList1.currentVictimIt_;
  }
 //!< The pose of the currently tracked victim when tracking was last updated 
  geometry_msgs::Pose getCurrentApproachPose(VictimList VictimList1)
  {
      return VictimList1.currentApproachPose_;
  }
  //!< A map containing correspondence of victims returned indices with ids
  std::map<int, int> getMap(VictimList VictimList1)
  {
    return VictimList1.victimIndicesMap_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getVictimsRequestAndGiven(VictimList VictimList1)
  {
     return VictimList1.victimsRequestedAndGiven_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getCurrentVictimDied(VictimList VictimList1)
  {
    return VictimList1.currentVictimDied_;
  }
  
  //!< The distance of the approach pose from the wall
  float getApproachDist(VictimList VictimList1)
  { 
     return VictimList1.APPROACH_DIST;
  }
  //!< The approach pose distance threshold for informing fsm of change 
  float getVictimUpdate(VictimList VictimList1)
  {
      return VictimList1.VICTIM_UPDATE;
  }

// variables 
VictimList VictimList1;
VictimList VictimList2;
VictimPtr Victim1;
VictimPtr Victim2;
VictimPtr Victim3;
VictimPtr Victim4;
VictimPtr Victim5;
VictimPtr Victim6;
ObjectConstPtrVector   ObjConstPtrVect1;
ObjectConstPtrVector   ObjConstPtrVect2;
ObjectConstPtrVector   ObjConstPtrVect3;
ObjectConstPtrVector   ObjConstPtrVect4;

};


TEST_F(VictimListTest,Constructor)
{
  EXPECT_FALSE(getVictimsRequestAndGiven(VictimList1));
  EXPECT_FALSE(getCurrentVictimDied( VictimList1));
  EXPECT_EQ(0.5,getApproachDist(VictimList1));
  EXPECT_EQ(0.5,getVictimUpdate(VictimList1));
  


  EXPECT_FALSE(getVictimsRequestAndGiven(VictimList2));
  EXPECT_FALSE(getCurrentVictimDied( VictimList2));
  EXPECT_EQ(1.5,getApproachDist(VictimList2));
  EXPECT_EQ(1,getVictimUpdate(VictimList2));

}


TEST_F(VictimListTest,contains)
{
  
  ASSERT_EQ(3,VictimList1.size());
  EXPECT_TRUE(VictimList1.contains(VictimConstPtr(Victim1)));
  EXPECT_TRUE(VictimList1.contains(VictimConstPtr(Victim2)));
  EXPECT_TRUE(VictimList1.contains(VictimConstPtr(Victim3)));
  EXPECT_FALSE(VictimList1.contains(VictimConstPtr(Victim4)));
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

