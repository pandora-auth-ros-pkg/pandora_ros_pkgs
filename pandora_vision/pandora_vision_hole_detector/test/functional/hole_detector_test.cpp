#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <gtest/gtest.h>

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "node_tests_msgs/ReplayBagsAction.h"

typedef actionlib::SimpleActionClient<node_tests_msgs::ReplayBagsAction> Replayer;
class HoleDetectorTest : public ::testing::Test
{
 protected:

  HoleDetectorTest() : nh_("test"), replayer_("/test/replay_bags", true)
  {
    replayer_.waitForServer();

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
    node_tests_msgs::ReplayBagsGoal goal;
    goal.start = true;
    replayer_.sendGoal(goal);

    ros::WallTime begin = ros::WallTime::now();

    while(ros::WallTime::now() - begin <= ros::WallDuration(15))
    {
      ros::spinOnce();
      ros::WallDuration d(0.20);
      d.sleep();
      actionlib::SimpleClientGoalState state = replayer_.getState();
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
        break;
      if(state == actionlib::SimpleClientGoalState::LOST || 
          state == actionlib::SimpleClientGoalState::REJECTED || 
          state == actionlib::SimpleClientGoalState::ABORTED ||
          state == actionlib::SimpleClientGoalState::PREEMPTED)
        ROS_BREAK();
    }
  }

  void holeDirectionAlertCallback(
    const vision_communications::HolesDirectionsVectorMsg& msg)
  {
    holeOutput_.push_back(msg);
  }
  
  ros::NodeHandle nh_;
  Replayer replayer_;
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
