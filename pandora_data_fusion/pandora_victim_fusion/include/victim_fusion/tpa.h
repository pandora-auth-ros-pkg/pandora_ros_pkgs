#ifndef TPA_H
#define TPA_H

#include "ros/ros.h"

#include "controllers_and_sensors_communications/tpaMsg.h"
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"

class TPASensor
{
	private:
		int _tpa[3][8];		//!< Tpa temperatures
		int _tpaAmbient[3];	//!< Tpa ambient temperature
		ros::Time _timestamp[3];	//!< Timestamp of measurement
		int _index; //!< Latest TPA valid
		
		ros::Publisher _tpaPublisher;
		
		// params 
		double _RELIABILITY;
		double _MEAN_VAL;
		double _STD_DEV;
		
		
	public:
		TPASensor();
		
		void loadParams(std::string state);
		void handleCallback(const controllers_and_sensors_communications::tpaMsg& msg);
		float calculateProbability() const;
		float calculateAngle() const;
		void publishMessage(float probability) const;
		
	private:
		int maxIndex(int id) const;
		
};

#endif
