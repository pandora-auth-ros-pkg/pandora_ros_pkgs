#include "victim_fusion/tpa.h"

#define D_PI 3.14159265359

TPASensor::TPASensor()
{
	ros::NodeHandle nh;
	_tpaPublisher = nh.advertise<data_fusion_communications::ThermalDirectionAlertMsg>("tpa_direction_alert", 1);
	loadParams("defaults");
}

void TPASensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/tpa/"+state+"/reliability", _RELIABILITY);
	ros::param::get("/victim_fusion/tpa/"+state+"/mean_value", _MEAN_VAL);
	ros::param::get("/victim_fusion/tpa/"+state+"/std_deviation", _STD_DEV);
}

void TPASensor::handleCallback(const controllers_and_sensors_communications::tpaMsg& msg)
{
	_index = msg.id;
	_tpaAmbient[_index] = msg.ambientTemp;
	
	for(unsigned int i=0;i<msg.pixelTemp.size();i++) {
		_tpa[_index][i] = msg.pixelTemp[i];
	}
	
	_timestamp[_index] = msg.header.stamp;
}

float TPASensor::calculateProbability() const
{
	ros::Duration duration = ros::Time::now()-_timestamp[_index];
	
	double tpaTimeConfidence = 1.0-1.0/(1+exp(-4.0*(duration.toSec()-1.0)));
	int maxId = maxIndex(_index);
	float xTpa = _tpa[_index][maxId];
	float probability = _RELIABILITY*(exp(-(1.0/2.0)*pow(((xTpa-_MEAN_VAL)/_STD_DEV),2)));
	
	return probability;	
}

float TPASensor::calculateAngle() const
{
	float angle[3] = {0, D_PI/4.0, -D_PI/4.0};
	return angle[_index] + 41.0/2.0*D_PI/180.0-(41.0/8.0*(maxIndex(7-_index))*D_PI/180.0);	
}

void TPASensor::publishMessage(float probability) const
{
	data_fusion_communications::ThermalDirectionAlertMsg msg;
	
	msg.header.stamp = _timestamp[_index];
	msg.header.frame_id = "/thermal";
	msg.yaw = calculateAngle();
	msg.pitch = 0; //  tha allaksei otan mpoun kai ta katakorifa tpa
	msg.probability = probability;
	
	_tpaPublisher.publish(msg);	
}

int TPASensor::maxIndex(int id) const
{
	unsigned int maxTpaIndex = 1000;
	float maxTpaVal = -1000.0;
	
	for(int i=1;i<7;i++) {
		float temp=(_tpa[id][i-1]+_tpa[id][i]+_tpa[id][i+1])/3.0;
		if(temp > maxTpaVal) {
			maxTpaVal=temp;
			maxTpaIndex=i;
		}
	}
	return maxTpaIndex;
}
