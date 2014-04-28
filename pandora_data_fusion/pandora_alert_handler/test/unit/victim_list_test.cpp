// "Copyright [2014] <Chamzas Konstantinos>"
#include "alert_handler/victim_list.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class VictimListTest : public ::testing::Test {
 
 protected:

  VictimListTest() : 
  victim1(new Victim), victim2( new Victim), victim3( new Victim), 
  victim4( new Victim), victim5( new Victim), victim6( new Victim) {}

  virtual void SetUp()
  {
    victimList1.setParams(0.5, 0.5, 0.5);
    victimList2.setParams(2, 1, 1.5);
     
    filterModelPtr.reset( new FilterModel );
    Victim::setHoleModel(filterModelPtr);
    Victim::setTpaModel(filterModelPtr);
    
    createVariousObjects1(&ObjConstPtrVect1);
    createVariousObjects2(&ObjConstPtrVect2);
    createVariousObjects3(&ObjConstPtrVect3);
    createVariousObjects4(&ObjConstPtrVect4);
     
    fillVictim(victim1, ObjConstPtrVect1);
    fillVictim(victim2, ObjConstPtrVect2);
    fillVictim(victim3, ObjConstPtrVect3);
    fillVictim(victim4, ObjConstPtrVect4);
     
    fillVictimListAdd(&victimList1);
    
    
  } 
  
  virtual void TearDown()
  {
    victimList1.clear();
    victimList2.clear();
    
  }
  
  
// helper functions
void fillVictimListAdd( VictimList* victimList1 )
  {
    victimList1->clear();
    victimList1->add(victim1);
    victimList1->add(victim2);
    victimList1->add(victim3);
    victimList1->add(victim4);
  }
  
void fillVictimListAddUnchanged( VictimList* victimList2 )
  {
    victimList2->clear();
    victimList2->addUnchanged(victim1);
    victimList2->addUnchanged(victim2);
    victimList2->addUnchanged(victim3);
    victimList2->addUnchanged(victim4);
    
  }

  
void fillVictim(VictimPtr Victim, ObjectConstPtrVector ObjConstPtrVect)
{
  Victim->setObjects(ObjConstPtrVect, 5);
}

// We fill the iteratorList specificcaly around victim2
void fillIteratorList(VictimList* victimListX, ObjectList<Victim>::IteratorList* iteratorListPtr )
{
    for (ObjectList<Victim>:: iterator it = victimListX->objects_.begin(); it != victimListX->objects_.end(); ++it)
    {
      if ((*it)->isSameObject(victim2, 0.5))
      {
        iteratorListPtr->push_back(it);
      }
    }
}


// Tpa1(-1, 0, 0)  Tpa2(1, 0, 0) Hole1(0 , 1, 0)  
  void createVariousObjects1(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(-1, 0, 0, tpaPtr1);
    TpaPtr tpaPtr2(new Tpa);
    setPose(1, 0, 0, tpaPtr2);
    tpaPtr2->update(tpaPtr2, filterModelPtr);
    tpaPtr2->update(tpaPtr2, filterModelPtr);
    HolePtr holePtr1(new Hole);
    setPose(0, 1, 0, holePtr1);
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr1));
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr2));
    ObjConstPtrVect->push_back(HoleConstPtr(holePtr1));
  }
// Tpa1(2, 3, 0) Hole1(3, 3, 0) Hole2(2, 2.5, 0) 
  void createVariousObjects2(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr tpaPtr1(new Tpa); 
    setPose(2, 3, 0, tpaPtr1);
    
    HolePtr holePtr1(new Hole);
    setPose(3, 3, 0, holePtr1);
    holePtr1->update(holePtr1, filterModelPtr);
    holePtr1->update(holePtr1, filterModelPtr);
    
    HolePtr holePtr2(new Hole); 
    setPose(2, 2.5, 0, holePtr2);
    
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr1)); 
    ObjConstPtrVect->push_back(HoleConstPtr(holePtr1)); 
    ObjConstPtrVect->push_back(HoleConstPtr(holePtr2));
  }
// Tpa1(2.7, 3, 0) Tpa2(3, 3, 0) 
  void createVariousObjects3(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(2.7, 3, 0, tpaPtr1);
    
    TpaPtr tpaPtr2(new Tpa);
    setPose(3, 3, 0, tpaPtr2);
    tpaPtr2->update(tpaPtr2, filterModelPtr);
    tpaPtr2->update(tpaPtr2, filterModelPtr);
    
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr1));
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr2));
  } 
  
// Tpa1(3, 3, 0) Hole1(10, 3, 0) 
  void createVariousObjects4(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr tpaPtr1(new Tpa);
    setPose(3, 3, 0, tpaPtr1);
    HolePtr holePtr1(new Hole);
    setPose(10, 3, 0, holePtr1);
    ObjConstPtrVect->push_back(TpaConstPtr(tpaPtr1));
    ObjConstPtrVect->push_back(HoleConstPtr(holePtr1));
  } 
  
  void setPose ( float x, float y, float z, ObjectPtr Object, float yaw =0)
  {
    
    geometry_msgs::Pose pose1;
    pose1.position.x = x;
    pose1.position.y = y;
    pose1.position.z = z;
    pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    Object->setPose(pose1);
    Object->initializeObjectFilter(0.5, 0.5, 0.5); 
  }
// accessors to private variables
  VictimList::iterator getCurrentVictimIt(VictimList victimList1)
  {
    return victimList1.currentVictimIt_;
  }
 //!< The pose of the currently tracked victim when tracking was last updated 
  geometry_msgs::Pose getCurrentApproachPose(VictimList victimList1)
  {
      return victimList1.currentApproachPose_;
  }
  //!< A map containing correspondence of victims returned indices with ids
  std::map<int, int> getMap(VictimList victimList1)
  {
    return victimList1.victimIndicesMap_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getVictimsRequestAndGiven(VictimList victimList1)
  {
     return victimList1.victimsRequestedAndGiven_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getCurrentVictimDied(VictimList victimList1)
  {
    return victimList1.currentVictimDied_;
  }
  
  //!< The distance of the approach pose from the wall
  float getApproachDist(VictimList victimList1)
  { 
     return victimList1.APPROACH_DIST;
  }
  //!< The approach pose distance threshold for informing fsm of change 
  float getVictimUpdate(VictimList victimList1)
  {
      return victimList1.VICTIM_UPDATE;
  }
  
// accesors to private functions/

  void updateObjects( VictimPtr victim, 
     ObjectList<Victim>::IteratorList* iteratorList, VictimList* victimList)
     {
       victimList->updateObjects( victim , *iteratorList);
     }
     
 ObjectList<Victim>::List& getObjects(VictimList* objList) 
  {
    return objList->objects_;
  }
  
  void setCurrentVictimIt( VictimList* victimList, ObjectList<Victim>::iterator *it)
  {
    victimList->currentVictimIt_ = *it;
    
  }
  


// variables 
VictimList victimList1;
VictimList victimList2;
FilterModelPtr filterModelPtr;
VictimPtr victim1;
VictimPtr victim2;
VictimPtr victim3;
VictimPtr victim4;
VictimPtr victim5;
VictimPtr victim6;
ObjectConstPtrVector   ObjConstPtrVect1;
ObjectConstPtrVector   ObjConstPtrVect2;
ObjectConstPtrVector   ObjConstPtrVect3;
ObjectConstPtrVector   ObjConstPtrVect4;
ObjectList<Victim> ::IteratorList iteratorList;

};


TEST_F(VictimListTest, Constructor)
{
  EXPECT_FALSE(getVictimsRequestAndGiven(victimList1));
  EXPECT_FALSE(getCurrentVictimDied( victimList1));
  EXPECT_EQ(0.5, getApproachDist(victimList1));
  EXPECT_EQ(0.5, getVictimUpdate(victimList1));
  


  EXPECT_FALSE(getVictimsRequestAndGiven(victimList2));
  EXPECT_FALSE(getCurrentVictimDied( victimList2));
  EXPECT_EQ(1, getApproachDist(victimList2));
  EXPECT_EQ(1.5, getVictimUpdate(victimList2));

}


TEST_F(VictimListTest, contains)
{
  
  // Victim1 Tpa(1, 0, 0) Hole(0 , 1, 0)  
  // Victim2 Tpa(2, 3, 0) Hole(3, 3, 0)
  // Victim3 Tpa(3, 3, 0)
  // Victim4 Tpa(3, 3, 0) Hole(10, 3, 0) 
  
  // victim 3 is the same as victim 2 (samePosition)
  ASSERT_EQ(3, victimList1.size());
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim1)));
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim2)));
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim3)));
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim4)));
}


TEST_F(VictimListTest , updateObjects)
{
  ObjectList<Victim>::iterator it;
  
  fillVictimListAddUnchanged(&victimList2);
  fillIteratorList(&victimList2 , &iteratorList);
  ASSERT_EQ(4, victimList2.size());
  
  // VictimList2 has all 4 victims inside
  
  it = getObjects(&victimList2).begin();
  EXPECT_EQ(victim1, *(it));
  EXPECT_EQ(victim2, *(++it));
  EXPECT_EQ(victim3, *(++it));
  EXPECT_EQ(victim4, *(++it));
  
  // Victim3 will be removed 
  updateObjects(victim2, &iteratorList, &victimList2);
  ASSERT_EQ(3, victimList2.size());
  it = getObjects(&victimList2).begin();
  EXPECT_EQ(victim1, *(it));
  EXPECT_EQ(victim2, *(++it));
  EXPECT_EQ(victim4, *(++it));
}


TEST_F(VictimListTest , ValidateCurrentObject)
{
  VictimList::iterator it;
    
  
  
  it = getObjects(&victimList1).begin();
  
  
              /* VictimList1*/
  // Victim1 Tpa(1, 0, 0) Hole(0 , 1, 0)  
  // Victim2 Tpa(2, 3, 0) Hole(3, 3, 0)
  // Victim4 Tpa(3, 3, 0) Hole(10, 3, 0)
  
  
  ASSERT_EQ(victim1, *(it));
  ASSERT_EQ(victim2, *(++it));
  ASSERT_EQ(victim4, *(++it));
  setCurrentVictimIt(&victimList1 , &it);
  
  
  // RUNTIME ERROR DONT KNOW WHY
  
  victimList1.validateCurrentObject(false);
   
  // RUNTIME ERROR DONT KNOW WHY
  
  
}



}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

