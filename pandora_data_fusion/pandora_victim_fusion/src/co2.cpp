#include "victim_fusion/co2.h"

CO2Sensor::CO2Sensor()
{
	loadParams("defaults");
}

void CO2Sensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/co2/"+state+"/optimal_ppm", _OPTIMAL_PPM);
	ros::param::get("/victim_fusion/co2/"+state+"/reliability", _RELIABILITY);
}

void CO2Sensor::handleCallback(const controllers_and_sensors_communications::co2Msg& msg)
{
	_ppm = msg.ppm;
	_timestamp = msg.header.stamp;	
}

float CO2Sensor::calculateProbability() const 
{
	ros::Duration duration = ros::Time::now()-_timestamp;
	double timeConfidence=1.0-1.0/(1+exp(-0.6*(duration.toSec()-7.0)));
	return timeConfidence*_RELIABILITY*(1.0/(1.0+exp(-0.01*(_ppm - _OPTIMAL_PPM))));
}
