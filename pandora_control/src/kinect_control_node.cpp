#include <std_msgs/Float64.h>
#include "ros/ros.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect_control_node");
	
	ros::NodeHandle nh;
	ros::Publisher kinect_pitch_publisher = nh.advertise<std_msgs::Float64>("/kinect_pitch_joint_position_controller/command", 5);
	ros::Publisher kinect_yaw_publisher = nh.advertise<std_msgs::Float64>("/kinect_yaw_joint_position_controller/command", 5);
	std_msgs::Float64 str;
	while(true){
		str.data = 0;
		kinect_pitch_publisher.publish(str);
		kinect_yaw_publisher.publish(str);
		sleep(2);
		str.data = 0.5;
		kinect_yaw_publisher.publish(str);
		sleep(2);
		str.data = 0.5;
		kinect_pitch_publisher.publish(str);
		sleep(2);
		str.data = 0;
		kinect_yaw_publisher.publish(str);
		sleep(2);
		str.data = -0.5;
		kinect_yaw_publisher.publish(str);
		sleep(2);
		str.data = 0;
		kinect_pitch_publisher.publish(str);
		sleep(2);
	}
}
