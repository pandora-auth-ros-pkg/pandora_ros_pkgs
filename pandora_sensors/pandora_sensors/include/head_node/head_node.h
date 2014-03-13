#ifndef HEADNODE_H
#define HEADNODE_H

#include "ros/ros.h"

#include <sys/time.h>

#include "controllers_and_sensors_communications/headIrMsg.h"
#include "controllers_and_sensors_communications/co2Msg.h"
#include "controllers_and_sensors_communications/headSonarMsg.h"
#include "controllers_and_sensors_communications/mlxTempMsg.h"

// servo
#include <actionlib/server/simple_action_server.h>
#include <controllers_and_sensors_communications/controlServoAction.h>
// end of servo

// gripper
#include "controllers_and_sensors_communications/gripperControlSrv.h"
// end of gripper

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#define UPDATE_OFFSET 	0
#define IR_OFFSET 	4
#define CO2_OFFSET 	8
#define SONAR_OFFSET 	12
#define STAT_OFFSET 	16
#define STAT_OFFSET2 	20
#define STAT_OFFSET3 	24
#define STAT_OFFSET4 	28
#define STAT_OFFSET5 	46
#define BS2_OFFSET	50

#define GRIP_OFF 	0
#define GRIP_CLOSE 	1
#define GRIP_OPEN 	2
#define MOV_SERVO 	3
#define ANGLE_MAX 	180

#define PACKET_SIZE 	64

class Head {
	
		ros::NodeHandle m_rosNode;
		double minFreqHeadIr, maxFreqHeadIr;

		ros::Publisher m_Co2Publisher;
		
		ros::Publisher m_HeadIrPublisher;
		
		ros::Publisher m_HeadSonarPublisher;
		
		ros::Publisher m_MlxPublisher;
		// servo
		ros::Time currentTimeStamp;
	       	ros::Time previousTimeStamp;
		controllers_and_sensors_communications::controlServoResult result_;
		actionlib::SimpleActionServer<controllers_and_sensors_communications::controlServoAction> m_actionServerHead;
		std::string action_name_;
		bool controlHeadCallback(const controllers_and_sensors_communications::controlServoGoalConstPtr &goal);
		// end of servo
		
		// gripper
		ros::ServiceServer m_GripperControl;
		bool controlGripper(controllers_and_sensors_communications::gripperControlSrv::Request &req, controllers_and_sensors_communications::gripperControlSrv::Response &res);
		// end of gripper
				
		int calculateIr(float voltage);
		
		int32_t m_distanceHead;
		int sonarMeasurements;
		int sonarOverall;

		unsigned char *buf;
		int count;
		unsigned int up2date;
		
		int fd;
	
	public:
	
		Head();
	
		int processData();
	
};


#endif
