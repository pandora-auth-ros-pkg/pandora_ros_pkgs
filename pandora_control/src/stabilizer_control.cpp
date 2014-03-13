#include "stabilizer_control.h"

StabilizerController::StabilizerController(void){
	std::string compassTopic;
	if (nh.hasParam("compassTopic")) {
		nh.getParam("compassTopic", compassTopic);
		ROS_DEBUG("[NavigationController]: Got parameter compassTopic : %s" , compassTopic.c_str());
	}
	else {
		ROS_WARN("[NavigationController] : Parameter compassTopic not found. Using Default");
		compassTopic = "/sensors/imu" ;
	}
	_compassSubscriber = nh.subscribe(compassTopic, 1, &StabilizerController::serveImuMessage, this);
	
	_laser_roll_publisher = nh.advertise<std_msgs::Float64>("/laser_roll_joint_position_controller/command", 5);
	_laser_pitch_publisher = nh.advertise<std_msgs::Float64>("/laser_pitch_joint_position_controller/command", 5);
}
	
void StabilizerController::serveImuMessage(const sensor_msgs::ImuConstPtr& msg){
	
	double compassYaw;	
	double compassPitch;	
	double compassRoll;	
	std_msgs::Float64 str;
	
	tf::Matrix3x3 matrix(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
	
	matrix.getRPY(compassRoll, compassPitch, compassYaw);
	
	str.data = -compassRoll;
	_laser_roll_publisher.publish(str);
	str.data = -compassPitch;
	_laser_pitch_publisher.publish(str);

}
