#ifndef STABILIZER_CONTROL_H
#define STABILIZER_CONTROL_H

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include "ros/ros.h"
#include <tf/tf.h>

class StabilizerController{
	
	private:
	
		ros::NodeHandle nh;
		ros::Subscriber _compassSubscriber;
		
		ros::Publisher _laser_roll_publisher;
		ros::Publisher _laser_pitch_publisher;
		
		void serveImuMessage(const sensor_msgs::ImuConstPtr& msg);
		
	public:
		
		StabilizerController(void);
};

#endif
