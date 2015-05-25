#include <ros/ros.h>
#include "pandora_explorer/frontier_goal_selector.h"



void update_thread(pandora_explorer::FrontierGoalSelector* frontier_selector)
{
  ros::Rate rate(1.0);

  while (ros::ok()) {
    ROS_INFO("Updating frontiers");

    geometry_msgs::PoseStamped goal;
    frontier_selector->findNextGoal(&goal);
    
    rate.sleep();
  }

}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "frontier_node", ros::init_options::NoSigintHandler);

  pandora_explorer::FrontierGoalSelector* frontier_selector = new pandora_explorer::FrontierGoalSelector("explore");

  boost::thread* frontier_update_thread;

  frontier_update_thread = new boost::thread(boost::bind(&update_thread, frontier_selector));

  ros::spin();

  frontier_update_thread->interrupt();
  frontier_update_thread->join();
  
  return 0;
}


