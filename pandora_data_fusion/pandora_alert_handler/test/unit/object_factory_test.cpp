// "Copyright [2014] <Chamzas Konstantinos>"
#include "alert_handler/object_factory.h"
#include "gtest/gtest.h"
#include <ros/package.h>
#include "map_loader/map_loader.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

// Defining PI
double pi= M_PI;

class ObjectFactoryTest : public ::testing::Test
{ 
  protected: 

/*Constructor SetUp */

  // Loading the Map and initializing Objectfactory with a
  // very small orientation circle(The other Params are the same)
  ObjectFactoryTest(): map_type1("TEST"), MapPtr1(new Map) 
  { 
    *MapPtr1 = map_loader::loadMap(
      ros::package::getPath("pandora_alert_handler")
        +"/test/test_maps/map1.yaml");
    ObjectFactoryPtr1.reset(new ObjectFactory(
      MapPtr1, map_type1, 0.5, 1.2, 0, 0.5, 20, 0.25)); 
  }
  
  // Creating all the Different Alerts we will use
    virtual void SetUp() 
  { 
    createHoleDirVector();
    createHazmatAlertVector();
    createQrAlertVector();
    createTpaDir();
  }
  
/* Helper Functions*/

  // Creating 3 HoleAlerts two on Walls and one very High
  void createHoleDirVector()
  {
    HoleDirVect1.header.seq = 1; 
    HoleDirVect1.header.frame_id = "Maria";
    HoleDir1.yaw = -pi/4;
    HoleDir1.pitch = pi/6; 
    HoleDir1.probability = 4;
    HoleDir1.holeId = 1; 
    HoleDirVect1.holesDirections.push_back(HoleDir1); 
    HoleDir2.yaw = -pi/2;
    HoleDir2.pitch = 0; 
    HoleDir2.probability = 4; 
    HoleDir2.holeId = 2;
    HoleDirVect1.holesDirections.push_back(HoleDir2);
    HoleDir3.yaw = 0; 
    HoleDir3.pitch = pi/2; 
    HoleDir3.probability = 4; 
    HoleDir3.holeId = 3;
    HoleDirVect1.holesDirections.push_back(HoleDir3);
  }
  
  // Creating 3 QrAlerts two on Walls and one very High
  void createQrAlertVector()
  {
   
    QrAlert1.yaw = -pi/4;
    QrAlert1.pitch = pi/6;
    QrAlert1.QRcontent = 
      "No one can make you feel inferior without your consent."; 
    QrAlertVect1.qrAlerts.push_back(QrAlert1); 
    QrAlert2.yaw = -pi/2;
    QrAlert2.pitch = 0; 
    QrAlert2.QRcontent =
      "Let him who would enjoy a good future waste none of his present."; 
    QrAlertVect1.qrAlerts.push_back(QrAlert2);
    QrAlert3.yaw = 0; 
    QrAlert3.pitch = pi/2;
    QrAlert3.QRcontent =
      "Live as if you were to die tomorrow.Learn as if you were to live forever.";
    QrAlertVect1.qrAlerts.push_back(QrAlert3);
  }
  
  // Creating 3 HazmatAlerts two on Walls and one very High
  void createHazmatAlertVector()
  {
    HazmatAlert1.yaw = -pi/4;
    HazmatAlert1.pitch = pi/6; 
    HazmatAlert1.patternType = 1; 
    HazmatAlertVect1.hazmatAlerts.push_back(HazmatAlert1); 
    HazmatAlert2.yaw = -pi/2;
    HazmatAlert2.pitch = 0; 
    HazmatAlert2.patternType = 2;
    HazmatAlertVect1.hazmatAlerts.push_back(HazmatAlert2);
    HazmatAlert3.yaw = 0;  
    HazmatAlert3.pitch = pi/2; 
    HazmatAlert3.patternType = 3;
    HazmatAlertVect1.hazmatAlerts.push_back(HazmatAlert3);
  }
  // Creating 2 TpaAlerts one on Wall and one very High
  void createTpaDir()
  {
  TpaDir1.yaw = -pi/4;
  TpaDir1.pitch =  pi/6;
  TpaDir2.yaw = 0;
  TpaDir2.pitch =  pi/2;
  }

/* variables */

  MapPtr MapPtr1;
  const std::string map_type1;
  ObjectFactoryPtr ObjectFactoryPtr1;
  vision_communications::HolesDirectionsVectorMsg HoleDirVect1;
  vision_communications::HoleDirectionMsg HoleDir1; 
  vision_communications::HoleDirectionMsg HoleDir2; 
  vision_communications::HoleDirectionMsg HoleDir3;
  vision_communications::QRAlertsVectorMsg QrAlertVect1; 
  vision_communications::QRAlertMsg QrAlert1;
  vision_communications::QRAlertMsg QrAlert2;
  vision_communications::QRAlertMsg QrAlert3;
  vision_communications::HazmatAlertsVectorMsg HazmatAlertVect1;
  vision_communications::HazmatAlertMsg HazmatAlert1;
  vision_communications::HazmatAlertMsg HazmatAlert2;
  vision_communications::HazmatAlertMsg HazmatAlert3;
  data_fusion_communications::ThermalDirectionAlertMsg  TpaDir1;
  data_fusion_communications::ThermalDirectionAlertMsg  TpaDir2;
};

// The tf that is used is created in mock object tfFinder
// With origin (5,5,0.3) and (roll,pitch,yaw)=(0,0,0)

/*Test  Cases*/

TEST_F(ObjectFactoryTest, makeHoles)
  { 
    HolePtrVectorPtr holesVectorPtr(new HolePtrVector);
    holesVectorPtr = ObjectFactoryPtr1-> makeHoles(HoleDirVect1);
    EXPECT_EQ(2, holesVectorPtr->size());
    
    HolePtr HolePtr1 = (*holesVectorPtr)[0];
    EXPECT_EQ(1, HolePtr1->getHoleId()); 
    EXPECT_NEAR(5.76, HolePtr1->getPose().position.x, 0.1);
    EXPECT_NEAR(5.76, HolePtr1->getPose().position.y, 0.1);
    EXPECT_NEAR(0.9205, HolePtr1->getPose().position.z, 0.01);
    EXPECT_NEAR(0, HolePtr1->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, HolePtr1->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, HolePtr1->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, HolePtr1->getPose().orientation.w, 0.1);
    
    HolePtr HolePtr2 = (*holesVectorPtr)[1];
    EXPECT_EQ(2, HolePtr2->getHoleId());
    EXPECT_NEAR(5, HolePtr2->getPose().position.x, 0.1);
    EXPECT_NEAR(5.76, HolePtr2->getPose().position.y, 0.1);
    EXPECT_NEAR(0.3, HolePtr2->getPose().position.z, 0.01);
    EXPECT_NEAR(0, HolePtr2->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, HolePtr2->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, HolePtr2->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, HolePtr2->getPose().orientation.w, 0.1);
  } 
  
TEST_F(ObjectFactoryTest, makeQrs)
  {
    QrPtrVectorPtr qrsVectorPtr( new QrPtrVector);
    qrsVectorPtr = ObjectFactoryPtr1-> makeQrs(QrAlertVect1);
    EXPECT_EQ( 2, qrsVectorPtr->size());
    
    QrPtr QrPtr1 = (*qrsVectorPtr)[0];
    EXPECT_STREQ( "No one can make you feel inferior without your consent."
    , QrPtr1->getContent().c_str()); 
    EXPECT_NEAR(5.76, QrPtr1->getPose().position.x, 0.1);
    EXPECT_NEAR(5.76, QrPtr1->getPose().position.y, 0.1);
    EXPECT_NEAR(0.9205, QrPtr1->getPose().position.z, 0.01);
    EXPECT_NEAR(0, QrPtr1->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, QrPtr1->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, QrPtr1->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, QrPtr1->getPose().orientation.w, 0.1);
    
    QrPtr QrPtr2=(*qrsVectorPtr)[1];
    EXPECT_STREQ("Let him who would enjoy a good future waste none of his present."
    , QrPtr2->getContent().c_str());
    EXPECT_NEAR(5, QrPtr2->getPose().position.x, 0.01);
    EXPECT_NEAR(5.76, QrPtr2->getPose().position.y, 0.01);
    EXPECT_NEAR(0.3, QrPtr2->getPose().position.z, 0.01);
    EXPECT_NEAR(0, QrPtr2->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, QrPtr2->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, QrPtr2->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, QrPtr2->getPose().orientation.w, 0.1);
  } 
  
TEST_F(ObjectFactoryTest, makeHazmats)
  { 
    HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
    hazmatsVectorPtr = ObjectFactoryPtr1->makeHazmats(HazmatAlertVect1);
    EXPECT_EQ( 2, hazmatsVectorPtr->size());
    
    HazmatPtr HazmatPtr1 = (*hazmatsVectorPtr)[0];
    EXPECT_EQ(1, HazmatPtr1->getPattern()); 
    EXPECT_NEAR(5.75, HazmatPtr1->getPose().position.x, 0.01);
    EXPECT_NEAR(5.75, HazmatPtr1->getPose().position.y, 0.01);
    EXPECT_NEAR(0.9205, HazmatPtr1->getPose().position.z, 0.01);
    EXPECT_NEAR(0, HazmatPtr1->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, HazmatPtr1->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, HazmatPtr1->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, HazmatPtr1->getPose().orientation.w, 0.1);
    HazmatPtr HazmatPtr2 = (*hazmatsVectorPtr)[1];
    EXPECT_EQ(2, HazmatPtr2->getPattern());
    EXPECT_NEAR(5, HazmatPtr2->getPose().position.x, 0.01);
    EXPECT_NEAR(5.76, HazmatPtr2->getPose().position.y, 0.01);
    EXPECT_NEAR(0.3, HazmatPtr2->getPose().position.z, 0.01);
    EXPECT_NEAR(0, HazmatPtr2->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, HazmatPtr2->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, HazmatPtr2->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711, HazmatPtr2->getPose().orientation.w, 0.1);
  } 
  
  //  A vector is retunred although tpa will always be alone :(
  TEST_F(ObjectFactoryTest, makeTpas)
  { 
    TpaPtrVectorPtr tpasVectorPtr( new TpaPtrVector );
    tpasVectorPtr = ObjectFactoryPtr1->makeTpas(TpaDir1);
    EXPECT_EQ(1, tpasVectorPtr->size());
    TpaPtr TpaPtr1 = (*tpasVectorPtr)[0];
    EXPECT_NEAR(5.75, TpaPtr1->getPose().position.x, 0.01);
    EXPECT_NEAR(5.75, TpaPtr1->getPose().position.y, 0.01);
    EXPECT_NEAR(0.9205, TpaPtr1->getPose().position.z, 0.01);
    EXPECT_NEAR(0, TpaPtr1->getPose().orientation.x, 0.1);
    EXPECT_NEAR(0, TpaPtr1->getPose().orientation.y, 0.1);
    EXPECT_NEAR(-0.70711, TpaPtr1->getPose().orientation.z, 0.1);
    EXPECT_NEAR(+0.70711 , TpaPtr1->getPose().orientation.w, 0.1);
    tpasVectorPtr = ObjectFactoryPtr1->makeTpas(TpaDir2);
  }
  
}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

int main(int argc, char **argv) 
{
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

