#ifndef CAMERA_H
#define CAMERA_H

#include "ros/ros.h"

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/HolesPositionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/MotionMsg.h"


//---------------HOLE--------------------//
class HoleSensor
{
	private:
		ros::Time _timestamp;
		std::string _frame;
		
		std::vector<vision_communications::HoleDirectionMsg> _holes;
		
		ros::Publisher _holePublisher;
		
		//params 
		float _RELIABILITY;
		
	public:
		HoleSensor();
		
		void loadParams(std::string state);
		void handleCallback(const vision_communications::HolesDirectionsVectorMsg& msg);
		std::vector<float> calculateProbability() const;
		void publishMessage(std::map<int, float> holesToProbabilities) const;
};


//-------------STEREO_HOLE----------------//
class StereoHoleSensor
{
	private:
		ros::Time _timestamp;
		std::string _frame;
		
		std::vector<vision_communications::HolePositionMsg> _holes;
		
		ros::Publisher _stereoHolePublisher;
		
		//params 
		float _RELIABILITY;
		
	public:
		StereoHoleSensor();
		
		void loadParams(std::string state);
		void handleCallback(const vision_communications::HolesPositionsVectorMsg& msg);
		std::vector<float> calculateProbability() const;
		void publishMessage(std::map<int, float> holesToProbabilities) const;
};

//---------------FACE--------------------//
class FaceSensor
{
	private:
		ros::Time _timestamp;
		std::string _frame;
		float _yaw;
		float _pitch;
		float _probability;
				
		ros::Publisher _facePublisher;
		
		//params 
		float _RELIABILITY;
		
	public:
		FaceSensor();
		
		void loadParams(std::string state);
		void handleCallback(const vision_communications::FaceDirectionMsg& msg);
		float calculateProbability() const;
		void publishMessage(float probability) const;
};


//---------------MOTION--------------------//
class MotionSensor
{
	private:
		ros::Time _timestamp;
		std::string _frame;
		float _probability;
						
		//params 
		float _RELIABILITY;
		
	public:
		MotionSensor();
		
		void loadParams(std::string state);
		void handleCallback(const vision_communications::MotionMsg& msg);
		float calculateProbability() const;
};

#endif
