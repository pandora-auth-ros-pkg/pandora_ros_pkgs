// "Copyright [year] <Copyright Owner>"

#include "alert_handler/alert_handler.h"

int main (int argc, char **argv) {

  ros::init(argc, argv, "alert_handler", ros::init_options::NoSigintHandler);
  AlertHandler alertHandler;
  ROS_INFO("Beginning Alert Handler node");
  ros::spin();
  // ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  // spinner.spin(); // spin
  return 0;
}
