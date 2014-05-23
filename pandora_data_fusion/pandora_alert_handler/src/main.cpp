// "Copyright [year] <Copyright Owner>"

#include <ros/console.h>

#include "alert_handler/alert_handler.h"

using pandora_data_fusion::pandora_alert_handler::AlertHandler;

int main (int argc, char** argv)
{
  ros::init(argc, argv, "alert_handler", ros::init_options::NoSigintHandler);
  if(argc == 1 && !strcmp(argv[0], "--debug"))
  {
    if( ros::console::set_logger_level(
          ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
      ros::console::notifyLoggerLevelsChanged();
    }
  }
  AlertHandler alertHandler("/data_fusion/alert_handler");
  ROS_INFO("Beginning Alert Handler node");
  ros::spin();
  // ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  // spinner.spin(); // spin
  return 0;
}
