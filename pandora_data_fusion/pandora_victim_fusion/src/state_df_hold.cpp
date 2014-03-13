#include "victim_fusion/state.h"

StateDfHold::StateDfHold(const SensorsContainer& container) : VictimFusionState(container)
{
	ros::NodeHandle nh;
	_dataFusionGlobalPublisher = nh.advertise<data_fusion_communications::FusionGlobalMsg>("global_probabilities", 1);
	_victimVerificationPublisher = nh.advertise<data_fusion_communications::VictimVerificationMsg>("victim_verification", 1);
	
	if (!ros::param::get("/victim_fusion/df_hold/co2", _CO2_THRESHOLD))
		_CO2_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/df_hold/motion", _MOTION_THRESHOLD))
		_MOTION_THRESHOLD = 0.5;
	
	if (!ros::param::get("/victim_fusion/df_hold/face", _FACE_THRESHOLD))
		_FACE_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/df_hold/mlx", _MLX_THRESHOLD))
		_MLX_THRESHOLD = 0.5;
}

void StateDfHold::fuse(SENSOR latestAlert)
{
	data_fusion_communications::VictimVerificationMsg victimMsg;
	data_fusion_communications::FusionGlobalMsg globalMsg;
	
	float co2True = sensorsContainer.co2Sensor.calculateProbability();
	float mlxTrue = sensorsContainer.mlxSensor.calculateProbability();
	float faceTrue = sensorsContainer.faceSensor.calculateProbability();
	float motionTrue = sensorsContainer.motionSensor.calculateProbability();
	
	float probability = 0;
	
	if (co2True > _CO2_THRESHOLD) {
		victimMsg.sensorIds.push_back(victimMsg.CO2);
		probability += co2True;
	}
	if (mlxTrue > _MLX_THRESHOLD) {
		victimMsg.sensorIds.push_back(victimMsg.MLX);
		probability += mlxTrue;
	}
	if (faceTrue > _FACE_THRESHOLD) {
		victimMsg.sensorIds.push_back(victimMsg.FACE);
		probability += faceTrue;
	}
	if (motionTrue > _MOTION_THRESHOLD) {
		victimMsg.sensorIds.push_back(victimMsg.MOTION);
		probability += motionTrue;
	}
	
	
	if (!victimMsg.sensorIds.empty()) {
		
		victimMsg.probability = probability/victimMsg.sensorIds.size();
		_victimVerificationPublisher.publish(victimMsg);
	}	
	
	globalMsg.co2 = co2True;
	globalMsg.mlx = mlxTrue;
	globalMsg.motion = motionTrue;
	globalMsg.face = faceTrue;
	_dataFusionGlobalPublisher.publish(globalMsg);
}

std::set<std::string> StateDfHold::topicsToSubscribe()
{
	std::set<std::string> topicsToOpen;
	topicsToOpen.insert("/sensors/co2");
	topicsToOpen.insert("/sensors/mlx");
	topicsToOpen.insert("/vision/face_direction");
	topicsToOpen.insert("/vision/motion");
	
	return topicsToOpen;
}
