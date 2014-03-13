#ifndef SENSORS_CONTAINER_H
#define SENSORS_CONTAINER_H

#include "co2.h"
#include "camera.h"
#include "mlx.h"
#include "tpa.h"

struct SensorsContainer
{
	CO2Sensor co2Sensor;			//!< Object of CO2 container
	MLXSensor mlxSensor;			//!< Object of MLX container
	TPASensor tpaSensor;			//!< Object of TPA container	
	HoleSensor holeSensor;
	FaceSensor faceSensor;
	MotionSensor motionSensor;	
	StereoHoleSensor stereoHoleSensor;
};

#endif
