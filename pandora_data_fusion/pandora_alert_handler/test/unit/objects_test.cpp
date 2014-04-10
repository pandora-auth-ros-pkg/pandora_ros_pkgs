/*Copyright [2014] <Chamzas Konstantinos>*/ 

#include "alert_handler/objects.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class ObjectsTest : public ::testing::Test 
{
protected:

  ObjectsTest()
  {
    QrPtr2.reset(new Qr);
    HazmatPtr2.reset(new Hazmat);
    HazmatPtr3.reset(new Hazmat);
    HolePtr2.reset(new Hole);
    TpaPtr2.reset(new Tpa);
  }

// Seting up all the differnet Objects that will be used for our test cases
// The distance Between the points (0,0,0,) (4,3,2) is 5 
  
  virtual void SetUp() 
  {
     
    pose1.position.x = 0;
    pose1.position.y = 0;
    pose1.position.z = 0;
    pose2.position.x = 4;
    pose2.position.y = 3;
    pose2.position.z = 0;
    Qr1.setPose(pose1);
    QrPtr2->setPose(pose2);
    Hazmat1.setPose(pose1);
    Hazmat1.setPattern(1);
    HazmatPtr2->setPose(pose2);
    HazmatPtr2->setPattern(1);
    HazmatPtr3->setPose(pose2);
    HazmatPtr3->setPattern(0);
    Hole1.setPose(pose1);
    HolePtr2->setPose(pose2);
    Tpa1.setPose(pose1);
    TpaPtr2->setPose(pose2);
  }
  
  /* Variables */

  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;
  Object Object1;
  Qr Qr1;
  QrPtr QrPtr2;
  Hazmat Hazmat1;
  HazmatPtr HazmatPtr2;
  HazmatPtr HazmatPtr3;
  Hole Hole1;
  HolePtr HolePtr2;
  Tpa Tpa1;
  TpaPtr TpaPtr2;
};

TEST_F(ObjectsTest, Construstors)
  {
  EXPECT_EQ(0, Object1.getCounter());
  EXPECT_FALSE(Object1.getLegit());
  EXPECT_STREQ("qr", Qr1.getType().c_str());    
  EXPECT_STREQ("hazmat", Hazmat1.getType().c_str());
  EXPECT_STREQ("hole", Hole1.getType().c_str());
  EXPECT_STREQ("tpa", Tpa1.getType().c_str());
  }

// Checks  if isSameObject() behaves correctly for all possible inputs(Qr)

TEST_F(ObjectsTest, IsSameQr)
  {
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(QrPtr2), 4));
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(QrPtr2), -8));
  EXPECT_TRUE(Qr1.isSameObject( ObjectConstPtr(QrPtr2), 6));
  }
         
// Checks  if isSameObject() behaves correctly for all possible inputs(Hazmat)
     
TEST_F(ObjectsTest, IsSameHazmat)
  {
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(HazmatPtr2), 4));
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(HazmatPtr2), -8));
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(HazmatPtr3), 6));  
  EXPECT_TRUE(Hazmat1.isSameObject( ObjectConstPtr(HazmatPtr2), 6));
  }
  
// Checks  if isSameObject() behaves correctly for all possible inputs(Hole)
     
TEST_F(ObjectsTest, IsSameHole) 
  {
  EXPECT_FALSE(Hole1.isSameObject( ObjectConstPtr(HolePtr2), 4));
  EXPECT_FALSE(Hole1.isSameObject( ObjectConstPtr(HolePtr2), -8));
  EXPECT_TRUE(Hole1.isSameObject( ObjectConstPtr(HolePtr2), 6));
  }

// Checks  if isSameObject() behaves correctly for all possible inputs(Tpa)
     
TEST_F(ObjectsTest, isSameTpa) 
  {
  EXPECT_FALSE(Tpa1.isSameObject( ObjectConstPtr(TpaPtr2), 4));
  EXPECT_FALSE(Tpa1.isSameObject( ObjectConstPtr(TpaPtr2), -8));
  EXPECT_TRUE(Tpa1.isSameObject( ObjectConstPtr(TpaPtr2), 6));
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

int main(int argc, char **argv)
  {
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
  }
