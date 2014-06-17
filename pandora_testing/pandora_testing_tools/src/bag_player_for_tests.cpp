#include "rosbag/player.h"
#include "boost/program_options.hpp"
#include "node_tests/bags_option_parser.h"
#include "rosgraph_msgs/Clock.h"
#include <time.h>
#include "ros/time.h"
#include "pandora_testing_tools/ReplayBagsAction.h"
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<pandora_testing_tools::ReplayBagsAction> actionServer; 

/**
 * The stub class 
 */
class bagPlayerForTests 
{
  rosbag::PlayerOptions _opts;
  ros::NodeHandle _nh;
  actionServer _ReplayBagsActionServer;
  bool _playedBags; 
  
public:
  bagPlayerForTests(int, char**);
  
  void executeCb(const pandora_testing_tools::ReplayBagsGoalConstPtr &goal);

};


/**
 * Constructor
 */
bagPlayerForTests::bagPlayerForTests(int argc, char** argv) :
  _nh("/test"),
  _ReplayBagsActionServer(_nh, "bag_player",
    boost::bind(&bagPlayerForTests::executeCb, this, _1), false),
  _playedBags(false) {
  
  _opts = parseOptions(argc, argv);
    
    _ReplayBagsActionServer.start();
    
    ros::spin();

    //~ ros::Rate loop_rate(10);    
    //~ while(ros::ok() && !_playedBags){
      //~ ros::spinOnce();
      //~ loop_rate.sleep();
    //~ }
}

void bagPlayerForTests::executeCb(const pandora_testing_tools::ReplayBagsGoalConstPtr &goal)
{
  ROS_INFO("BAG_REPLAY STARTED");

  rosbag::Player player(_opts);
  try {
      player.publish();
    }
    catch (std::runtime_error& e) {
      ROS_FATAL("%s", e.what());
      _ReplayBagsActionServer.setAborted();
      return ;
    }
    
    pandora_testing_tools::ReplayBagsResult result; 
    result.ended = true;
    ROS_INFO("BAG_REPLAY COMPLETED");
  _ReplayBagsActionServer.setSucceeded(result);
  
  ros::shutdown();
  
}

/**
 *  main 
 */
int main(int argc,char **argv)
{
  
  for(int i=0;i<argc;i++){
    printf("%s\n",argv[i]);
  }

  ros::init(argc,argv,"bag_test_player");
  bagPlayerForTests bagPlayer(argc,argv);
  return 0;
}

