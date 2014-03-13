#ifndef MLX_H
#define MLX_H

#include "ros/ros.h"

#include "controllers_and_sensors_communications/mlxTempMsg.h"
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"

class MLXSensor
{
	private:
		float _temperature[2];	//!< MLX temperatures, [0]->Arm high,[1]->Arm low
		float _maxTemp;
		ros::Time _timestamp;		//!< Timestamp of the measurement
		std::string _frame;
		std::vector<float> _tempBuffer[2];	//!< Buffer for mean filtering
		
		ros::Publisher _mlxPublisher;
			
		double _RELIABILITY;			//!< Pre-defined MLX reliability. Initialised from parameter server.
		float _STD_DEV, _MEAN_VAL;
		
	public:
		MLXSensor();
		
		void loadParams(std::string state);
		void handleCallback(const controllers_and_sensors_communications::mlxTempMsg& msg);
		float calculateProbability() const;
		void publishMessage(float probability) const;
		
	private:
		float calculateMeanVal(std::vector<float> tempBuffer, int count);
		
};

#endif
