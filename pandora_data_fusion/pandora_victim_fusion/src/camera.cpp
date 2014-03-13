#include "victim_fusion/camera.h"


//---------------HOLE--------------------//
HoleSensor::HoleSensor()
{
	ros::NodeHandle nh;
	_holePublisher = nh.advertise<vision_communications::HolesDirectionsVectorMsg>("hole_direction_alert", 1);
	loadParams("defaults");
}

void HoleSensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/hole/"+state+"/reliability", _RELIABILITY);
}

void HoleSensor::handleCallback(const vision_communications::HolesDirectionsVectorMsg& msg)
{
	_timestamp = msg.header.stamp;
	_frame = msg.header.frame_id;
	
	_holes = msg.holesDirections;
}

std::vector<float> HoleSensor::calculateProbability() const
{
	std::vector<float> probabilities;
		
	for (int i = 0; i < _holes.size(); i++) {
		probabilities.push_back(_RELIABILITY * _holes[i].probability);
	}
		
	return probabilities;
}

void HoleSensor::publishMessage(std::map<int, float> holesToPublish) const 
{
	vision_communications::HolesDirectionsVectorMsg msg;
	
	std::vector<vision_communications::HoleDirectionMsg> holesFiltered;
	
	msg.header.stamp = _timestamp;
	msg.header.frame_id = _frame;
	
	for (std::map<int,float>::iterator it=holesToPublish.begin(); it!=holesToPublish.end(); it++) {
		vision_communications::HoleDirectionMsg oneHole(_holes[it->first]);
		oneHole.probability = it->second;
		holesFiltered.push_back(oneHole);
	}
	
	if (!holesFiltered.empty()) {
		msg.holesDirections = holesFiltered;
		_holePublisher.publish(msg);
	}
}


//-------------STEREO_HOLE----------------//
StereoHoleSensor::StereoHoleSensor()
{
	ros::NodeHandle nh;
	_stereoHolePublisher = nh.advertise<vision_communications::HolesPositionsVectorMsg>("hole_position_alert", 1);
	loadParams("defaults");
}

void StereoHoleSensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/stereo_hole/"+state+"/reliability", _RELIABILITY);
}

void StereoHoleSensor::handleCallback(const vision_communications::HolesPositionsVectorMsg& msg)
{
	_timestamp = msg.header.stamp;
	_frame = msg.header.frame_id;
	
	_holes = msg.holesPositions;
}

std::vector<float> StereoHoleSensor::calculateProbability() const
{
	std::vector<float> probabilities;
		
	for (int i = 0; i < _holes.size(); i++) {
		probabilities.push_back(_RELIABILITY * _holes[i].probability);
	}
		
	return probabilities;
}

void StereoHoleSensor::publishMessage(std::map<int, float> holesToPublish) const 
{
	vision_communications::HolesPositionsVectorMsg msg;
	
	std::vector<vision_communications::HolePositionMsg> holesFiltered;
	
	msg.header.stamp = _timestamp;
	msg.header.frame_id = _frame;
	
	for (std::map<int,float>::iterator it=holesToPublish.begin(); it!=holesToPublish.end(); it++) {
		vision_communications::HolePositionMsg oneHole(_holes[it->first]);
		oneHole.probability = it->second;
		holesFiltered.push_back(oneHole);
	}
	
	if (!holesFiltered.empty()) {
		msg.holesPositions = holesFiltered;
		_stereoHolePublisher.publish(msg);
	}
}


//---------------FACE--------------------//
FaceSensor::FaceSensor()
{
	ros::NodeHandle nh;
	_facePublisher = nh.advertise<vision_communications::FaceDirectionMsg>("face_direction_alert", 1);
	loadParams("defaults");
}

void FaceSensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/face/"+state+"/reliability", _RELIABILITY);
}

void FaceSensor::handleCallback(const vision_communications::FaceDirectionMsg& msg)
{
	_timestamp = msg.header.stamp;
	_frame = msg.header.frame_id;
	_yaw = msg.yaw;
	_pitch = msg.pitch;
	_probability = msg.probability;
}

float FaceSensor::calculateProbability() const
{
	ros::Duration duration = ros::Time::now()-_timestamp;
	double timeConfidence=1.0-1.0/(1+exp(-0.6*(duration.toSec()-7.0)));
	return timeConfidence * _RELIABILITY * _probability;
}

void FaceSensor::publishMessage(float probability) const
{
	vision_communications::FaceDirectionMsg msg;
	
	msg.header.stamp = _timestamp;
	msg.header.frame_id = _frame;
	msg.yaw = _yaw;
	msg.pitch = _pitch;
	msg.probability = probability;
	
	_facePublisher.publish(msg);
}


//---------------MOTION--------------------//
MotionSensor::MotionSensor()
{
	loadParams("defaults");
}

void MotionSensor::loadParams(std::string state)
{
	ros::param::get("/victim_fusion/motion/"+state+"/reliability", _RELIABILITY);
}

void MotionSensor::handleCallback(const vision_communications::MotionMsg& msg)
{
	_timestamp = msg.header.stamp;
	_frame = msg.header.frame_id;
	_probability = msg.probability;
}

float MotionSensor::calculateProbability() const
{
	ros::Duration duration = ros::Time::now()-_timestamp;
	double timeConfidence = 1.0-1.0/(1+exp(-0.6*(duration.toSec()-7.0)));
	return timeConfidence * _RELIABILITY * _probability;
}
