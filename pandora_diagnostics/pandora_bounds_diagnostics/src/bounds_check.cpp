#include "ros/ros.h"

#include "bounds_diagnostics/controllers_and_sensors/overall_controllers_checker.h"

int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "MsgDiagnListener");
  
  ros::NodeHandle nh;

  overallControllersChecker controllersMsgCheck;
  
  ros::spin();

  return 0;
  
}
