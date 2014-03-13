#include "victim_fusion/mlx.h"

MLXSensor::MLXSensor()
{
	ros::NodeHandle nh;
	_mlxPublisher = nh.advertise<data_fusion_communications::ThermalDirectionAlertMsg>("mlx_direction_alert", 1);
	loadParams("defaults");
}

void MLXSensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/mlx/"+state+"/reliability", _RELIABILITY);
	ros::param::get("/victim_fusion/mlx/"+state+"/mean_value", _MEAN_VAL);
	ros::param::get("/victim_fusion/mlx/"+state+"/std_deviation", _STD_DEV);
}

void MLXSensor::handleCallback(const controllers_and_sensors_communications::mlxTempMsg& msg)
{	
	_timestamp = msg.header.stamp;
	_frame = msg.header.frame_id;
	
	_tempBuffer[0].push_back(msg.mlxTemp[0]);
	_tempBuffer[1].push_back(msg.mlxTemp[1]);
	
	// calculate mean for last 10 msgs
	float meanTemp0 = calculateMeanVal(_tempBuffer[0], 10);
	float meanTemp1 = calculateMeanVal(_tempBuffer[1], 10);
	
	// keep the the maximum 
	if (meanTemp0 > meanTemp1)
		_maxTemp = meanTemp0;
	else
		_maxTemp = meanTemp1;
		
	// keep only 10 values
	if (_tempBuffer[0].size() > 10)
		_tempBuffer[0].erase(_tempBuffer[0].begin());
	
	if (_tempBuffer[1].size() > 10)
		_tempBuffer[1].erase(_tempBuffer[1].begin());	
}


float MLXSensor::calculateProbability() const
{	
	float MLXTimeConfidence, probability;
	
	ros::Duration duration = ros::Time::now()-_timestamp;
	
	MLXTimeConfidence = 1.0-1.0/(1+exp(-0.6*(duration.toSec()-7.0)));
	probability = MLXTimeConfidence*_RELIABILITY*exp(-(1.0/2.0)*pow(((_maxTemp-_MEAN_VAL)/_STD_DEV),2)); 
		
	return probability;
}

void MLXSensor::publishMessage(float probability) const
{
	data_fusion_communications::ThermalDirectionAlertMsg msg;
	
	msg.header.stamp = _timestamp;
	msg.header.frame_id = _frame;
	msg.yaw = 0;
	msg.pitch = 0;
	msg.probability = probability;
	
	_mlxPublisher.publish(msg);
}


float MLXSensor::calculateMeanVal(std::vector<float> tempBuffer, int count)
{
	float mean = 0;
	for(unsigned int i=0;i<std::min((int)tempBuffer.size(),count);i++)
		mean += tempBuffer[i];
	mean /= std::min((int)tempBuffer.size(),count);
	return mean;
}
