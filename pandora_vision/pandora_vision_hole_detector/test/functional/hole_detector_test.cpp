#include <ros/ros.h>

#include <gtest/gtest.h>

#include "vision_communications/HolesDirectionsVectorMsg.h"

class HoleDetectorTest : public ::testing::Test
{
 protected:

  HoleDetectorTest() : nh_("test")
  {  
    std::string topic;

    if (nh_.getParam("subscribed_topic_names/holeDirection", topic))
    {
      holeDirectionSubscriber_ = nh_.subscribe(topic, 
        1, &HoleDetectorTest::holeDirectionAlertCallback, this);
    }
    else
    {
      ROS_FATAL("holeDirection topic name param not found");
      ROS_BREAK();
    }
  }

  virtual void SetUp()
  {
    ros::WallTime begin = ros::WallTime::now();

    while(ros::WallTime::now() - begin <= ros::WallDuration(10))
    {
      ros::spinOnce();
      ros::WallDuration d(0.20);
      d.sleep();
    }
  }

  void holeDirectionAlertCallback(
    const vision_communications::HolesDirectionsVectorMsg& msg)
  {
    holeOutput_.push_back(msg);
  }
  
  ros::NodeHandle nh_;
  ros::Subscriber holeDirectionSubscriber_;
  std::vector<vision_communications::HolesDirectionsVectorMsg> holeOutput_;
  
};

TEST_F(HoleDetectorTest, detectionIsFunctional)
{
  // Make assertions here according to what you would expect holeOutput_ to contain.
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hole_detector_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
