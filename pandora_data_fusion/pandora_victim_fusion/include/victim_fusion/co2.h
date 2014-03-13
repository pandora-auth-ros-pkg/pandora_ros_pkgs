#ifndef CO2_H
#define CO2_H

#include "ros/ros.h"
#include "controllers_and_sensors_communications/co2Msg.h"

class CO2Sensor
{
	private:
		int _ppm;			//!< Parts per million CO2 in air
		ros::Time _timestamp;	//!< Timestamp of measurement

		double _OPTIMAL_PPM;
		float _RELIABILITY;
		
	public:	
		CO2Sensor();
		
		void loadParams(std::string state);
		void handleCallback(const controllers_and_sensors_communications::co2Msg& msg);
		float calculateProbability() const;		
};

#endif
