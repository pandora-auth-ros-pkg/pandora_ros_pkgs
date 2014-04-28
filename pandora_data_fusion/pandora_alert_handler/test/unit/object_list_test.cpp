// "Copyright [2014] <Pandora_Software_Testing_Team>" 

#include <cstdlib>
#include <ctime>
#include "alert_handler/object_list.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class ObjectListTest : public testing::Test 
{
 protected:
 
  /* Constructor/Destructor */

  ObjectListTest() :
  object1(new Object), object2(new Object), object3(new Object),
  object4(new Object), object5(new Object), object6(new Object),
  object7(new Object), object8(new Object), object9(new Object),
  object10(new Object), object11(new Object) {}

  /* SetUp/TearDown definitions */

  virtual void SetUp() 
  {
    objectList.setParams(1, 0.5);
    objectList2.setParams(5, 10.699, 0.5, 0.4, 0.3, 0.1, 0.15, 0.20);

    std::srand(std::time(0));
    
    seed = time(NULL);
    
    
    pose1.position.x = -0.5;
    pose1.position.y = 0;
    pose1.position.z = 0;
    object1->setPose(pose1); 
    object1->setId(1);
    
    pose2.position.x = 0;
    pose2.position.y = 0;
    pose2.position.z = 0;      
    object2->setPose(pose2);
    object2->setId(2);
    
    
    pose3.position.x = 0.5;
    pose3.position.y = 0;
    pose3.position.z = 0;      
    object3->setPose(pose3);
    object3->setId(3);

    pose4.position.x = -1;
    pose4.position.y = 0;
    pose4.position.z = 0;      
    object4->setPose(pose4);
    object4->setId(4);
    
    
    pose5.position.x = -0.125;
    pose5.position.y = -0.125;
    pose5.position.z = 0;      
    object5->setPose(pose5);
    object5->setId(5);
    
    pose6.position.x = 0;
    pose6.position.y = 0.4;
    pose6.position.z = 0;      
    object6->setPose(pose6);
    object6->setId(6);
    
    pose7.position.x = static_cast<double>
     (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
    pose7.position.y = 
      static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
    pose7.position.z = 
      static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;      
    object7->setPose(pose7);
    object7->setId(7);
    
    
    pose8.position.x = 0.125;
    pose8.position.y = 0.125;
    pose8.position.z = 0;      
    object8->setPose(pose8);
    object8->setId(8);
    
    pose9.position.x = 0.4;
    pose9.position.y = 0.1;
    pose9.position.z = 0;
    object9->setPose(pose9);
    object9->setId(9);
    
    pose10.position.x = 0.6;
    pose10.position.y = 0.2;
    pose10.position.z = 0.1;
    object10->setPose(pose10);
    object10->setId(10);
    
    pose11.position.x = -0.75;
    pose11.position.y = 0;
    pose11.position.z = 0;
    object11->setPose(pose11);
    object11->setId(11);
    
  }

  /* Function to fill objectList.objects_ */

  void fillList(ObjectList<Object>* objList) 
  {
    getObjects(objList).clear();
    getObjects(objList).push_back(object1);
    getObjects(objList).push_back(object2);
    getObjects(objList).push_back(object3);
  }

  void fillListRandom(ObjectList<Object>* objList, int n) 
  {
    getObjects(objList).clear();

    for(int i = 0; i < n; ++i) 
    {
      geometry_msgs::Pose pose;
      pose.position.x =
        static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
      pose.position.y=
        static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
      pose.position.z =
        static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;   
      ObjectPtr object(new Object);
      object->setPose(pose);
      getObjects(objList).push_back(object);
    }
  }

  //!< Spawns Objects In a fixed radius Around the objectX
  ObjectPtr ObjectSpawner(ObjectPtr objectX, float radius)
  {
    ObjectPtr closeObject(new Object);
    double angle1( static_cast<double> ((rand_r(&seed) % 314)) / 100);
    double angle2( static_cast<double> ((rand_r(&seed) % 314)) / 200);
    
    geometry_msgs::Pose pose;
    pose.position.x = (cos(angle2) * radius) * sin(angle1) + 
      objectX->getPose().position.x;
    pose.position.y = (cos(angle2) * radius) * cos(angle1) + 
      objectX->getPose().position.y;
    pose.position.z = sin(angle2) * radius  + objectX->getPose().position.z;

    closeObject->setPose(pose);
     
    return closeObject;
  }
  
  
  //!< Returns distance between 2 objects.
  float distance(const ObjectConstPtr& object1, const ObjectConstPtr& object2)
  {
    float x_ = object1->getPose().position.x - object2->getPose().position.x;
    float y_ = object1->getPose().position.y - object2->getPose().position.y;
    float z_ = object1->getPose().position.z - object2->getPose().position.z;
    return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
  }
  
  void setPose(float x, float y, float z, ObjectPtr object)
  {
    geometry_msgs::Pose pose1;
    pose1.position.x = x;
    pose1.position.y = y;
    pose1.position.z = z;
    object->setPose(pose1);
  }  

  /* Accessors for private methods/members of ObjectList */

  bool isAnExistingObject(
      ObjectList<Object>* objList, const ObjectConstPtr& object, 
      ObjectList<Object>::IteratorList* iteratorListPtr)
  {
    return objList->isAnExistingObject(object, iteratorListPtr);
  }


  int* id(ObjectList<Object>* objList) 
  {
    return &(objList->id_);
  }

  ObjectList<Object>::List& getObjects(ObjectList<Object>* objList) 
  {
    return objList->objects_;
  }
  
  // get and set the params

  float* DIST_THRESHOLD(ObjectList<Object>* objList) 
  {
    return &(objList->DIST_THRESHOLD);
  }

  float* X_VAR_THRES(ObjectList<Object>* objList) 
  {
    return &(objList->X_VAR_THRES);
  }

  float* Y_VAR_THRES(ObjectList<Object>* objList) 
  {
    return &(objList->Y_VAR_THRES);
  }

  float* Z_VAR_THRES(ObjectList<Object>* objList) 
  {
    return &(objList->Z_VAR_THRES);
  }

  float* PRIOR_X_SD(ObjectList<Object>* objList) 
  {
    return &(objList->PRIOR_X_SD);
  }

  float* PRIOR_Y_SD(ObjectList<Object>* objList) 
  {
    return &(objList->PRIOR_Y_SD);
  }

  float* PRIOR_Z_SD(ObjectList<Object>* objList) 
  {
    return &(objList->PRIOR_Z_SD);
  }

  /* Variables */
  unsigned int  seed;
  ObjectList<Object> objectList;
  ObjectList<Object> objectList2;
  ObjectPtr object1;
  ObjectPtr object2;
  ObjectPtr object3;
  ObjectPtr object4;
  ObjectPtr object5;
  ObjectPtr object6;
  ObjectPtr object7;
  ObjectPtr object8;
  ObjectPtr object9;
  ObjectPtr object10;
  ObjectPtr object11;
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;
  geometry_msgs::Pose pose3;
  geometry_msgs::Pose pose4;
  geometry_msgs::Pose pose5;
  geometry_msgs::Pose pose6;
  geometry_msgs::Pose pose7;
  geometry_msgs::Pose pose8;
  geometry_msgs::Pose pose9;
  geometry_msgs::Pose pose10;
  geometry_msgs::Pose pose11;

};

TEST_F(ObjectListTest, Constructor) 
{
  EXPECT_EQ( 0u , objectList.size() );
  EXPECT_EQ( 0u , *id(&objectList) );
  EXPECT_NEAR( 0.5 , *DIST_THRESHOLD(&objectList) , 0.0001 );

  EXPECT_EQ( 0u , objectList2.size() );
  EXPECT_EQ( 0u , *id(&objectList2) );
  EXPECT_NEAR( 0.5 , *X_VAR_THRES(&objectList2) , 0.0001 );
  EXPECT_NEAR( 0.4 , *Y_VAR_THRES(&objectList2) , 0.0001 );
  EXPECT_NEAR( 0.3 , *Z_VAR_THRES(&objectList2) , 0.0001 );
  EXPECT_NEAR( 0.1 , *PRIOR_X_SD(&objectList2) , 0.0001 );
  EXPECT_NEAR( 0.15 , *PRIOR_Y_SD(&objectList2) , 0.0001 );
  EXPECT_NEAR( 0.20 , *PRIOR_Z_SD(&objectList2) , 0.0001 );
}

TEST_F(ObjectListTest, IsAnExistingObject) 
{  
  fillList(&objectList);
  ObjectList<Object>::IteratorList iteratorList;
  ObjectList<Object>::IteratorList::const_iterator it;
  
  
  ASSERT_EQ( 0.5, *DIST_THRESHOLD(&objectList) );

  // It shouldn't find that Object4 already exists.
  // Object4 won't correlate with Object1 because dist equals DIST_THRES!

  EXPECT_FALSE( isAnExistingObject(&objectList, object4, &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  iteratorList.clear();

  // It should find that object5  exists in 2 places.

  EXPECT_TRUE( isAnExistingObject(&objectList, object5, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 2u , iteratorList.size() );
  EXPECT_EQ( object1, **(it++) );
  EXPECT_EQ( object2, **(it) );

  iteratorList.clear();

  // It should find that object6 exists only  in 1 place( same as object 3).
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_TRUE( isAnExistingObject(&objectList, object6, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 1u , iteratorList.size() );
  EXPECT_EQ( object2 , **it );
  EXPECT_EQ( object2 , **it );

  iteratorList.clear();

  // Changed distance must find Object 6 in  3 places( object1 object2 object3).
  (*DIST_THRESHOLD(&objectList)) = 1; 
  EXPECT_EQ( 1u , *DIST_THRESHOLD(&objectList) );
  EXPECT_TRUE( isAnExistingObject(&objectList, object6, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 3u , iteratorList.size() );
  EXPECT_EQ( object1 , **(it++) );
  EXPECT_EQ( object2 , **(it++) );
  EXPECT_EQ( object3 , **it );

  iteratorList.clear();

  // Or it doesn't find it.
  (*DIST_THRESHOLD(&objectList)) = 0.2;

  EXPECT_FALSE( isAnExistingObject(&objectList , object6 , &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_EQ( 0u , iteratorList.size() );

  iteratorList.clear();

  // Zero Distance Threshold makes impossible same Object recognition.
  (*DIST_THRESHOLD(&objectList)) = 0;
  fillListRandom(&objectList, 10000);

  EXPECT_FALSE( isAnExistingObject(&objectList , object7 , &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_EQ( 0u, iteratorList.size() );

  iteratorList.clear();

  // Maximum (Infinite) Distance Threshold makes impossible Object distinction.
  (*DIST_THRESHOLD(&objectList)) = FLT_MAX;

  EXPECT_TRUE( isAnExistingObject(&objectList, object7 , &iteratorList) );
  EXPECT_FALSE( iteratorList.empty() );
  EXPECT_EQ( objectList.size() , iteratorList.size() );

  iteratorList.clear();
}


TEST_F(ObjectListTest, AddManually) 
{ 
  ObjectList<Object>::const_iterator_vers_ref it = getObjects(&objectList).begin();
  ASSERT_EQ( 0u, objectList.size() );
  ASSERT_EQ( 0.5, *DIST_THRESHOLD(&objectList) );
  
  // Add (0.4,0.1,0)
  EXPECT_TRUE( objectList.add(object9) );
  ASSERT_EQ( 1, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9, *it );
  EXPECT_FALSE( object9->getLegit() );

  // Add (0.5, 0, 0) Object 3 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object3) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_FALSE( object9->getLegit() );

  // Add (0.125, 0.125, 0) Object 8 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object8) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );

  // Add (0.6, 0.2, 0.1) Object 10 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object10) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );
  
  // Add (0.6, 0.2, 0.1) Object 10 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object10) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );
  
  // Add (0.125, 0.125, 0) Object 9 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object9) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );
  
  // Add (0.125, 0.125, 0) Object 8 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object8) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );
  
  // Add (0.125, 0.125, 0) Object 9 will not be Added Same as Object 9
  EXPECT_FALSE( objectList.add(object9) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object9 , *it );
  EXPECT_TRUE( object9->getLegit() );
}

TEST_F(ObjectListTest, AddRandomly) 
{ 
  ObjectList<Object>::const_iterator_vers_ref it = getObjects(&objectList).begin();
  ASSERT_EQ( 0u, objectList.size() );
  ASSERT_EQ( 0.5, *DIST_THRESHOLD(&objectList) );
  
  EXPECT_TRUE( objectList.add(object2) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.4)) );
  ASSERT_EQ(1, objectList.size());
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object2, *it );
  EXPECT_FALSE( object2->getLegit() );
  
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.1)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.3)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.1)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.3)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object2, 0.1)) );
  EXPECT_TRUE( object2->getLegit());
}

TEST_F(ObjectListTest, AddTwoObjects) 
{ 
  ObjectList<Object>::const_iterator_vers_ref it = getObjects(&objectList).begin();
  ASSERT_EQ( 0u, objectList.size() );
  ASSERT_EQ( 0.5, *DIST_THRESHOLD(&objectList) );
  
  EXPECT_TRUE( objectList.add(object1) );
  ASSERT_EQ(1, objectList.size());
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object1, *it );
  EXPECT_FALSE( object1->getLegit() );
  
  
  EXPECT_TRUE( objectList.add(object4) );
  ASSERT_EQ(2, objectList.size());
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object4, *(++it) );
  EXPECT_FALSE( object4->getLegit() );
  
  // Object  1(-0,5,0,0) Object 4 (-1 ,0,0) Object11 (-0.75 ,0, 0)
  // This will spawn Objects in the middle so both Objects should became legit 
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.1)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.3)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.1)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.3)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.2)) );
  EXPECT_FALSE( objectList.add(ObjectSpawner(object11, 0.1)) );
  
  // They should have moved closer to each other
  EXPECT_LT(distance(object1, object4), 0.5);
  
  // They should become Legit!
  EXPECT_TRUE( object4->getLegit() );
  EXPECT_TRUE( object1->getLegit() );
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

