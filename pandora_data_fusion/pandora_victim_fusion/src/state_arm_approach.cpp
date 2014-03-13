#include "victim_fusion/state.h"

StateArmApproach::StateArmApproach(const SensorsContainer& container) : VictimFusionState(container)
{
	if (!ros::param::get("/victim_fusion/arm_approach/face", _FACE_THRESHOLD))
		_FACE_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/arm_approach/mlx", _MLX_THRESHOLD))
		_MLX_THRESHOLD = 0.5;
}

void StateArmApproach::fuse(SENSOR latestAlert)
{
	if(latestAlert==_MLX) {
		float victimTrue = sensorsContainer.mlxSensor.calculateProbability();
		
		if (victimTrue > _MLX_THRESHOLD) {
			sensorsContainer.mlxSensor.publishMessage(victimTrue);
			return;
		}
	}
	else if(latestAlert==_FACE) {
		float victimTrue = sensorsContainer.faceSensor.calculateProbability();
		
		if (victimTrue > _FACE_THRESHOLD) {
			sensorsContainer.faceSensor.publishMessage(victimTrue);
			return;
		}		
	}
}

std::set<std::string> StateArmApproach::topicsToSubscribe()
{
	std::set<std::string> topicsToOpen;
	topicsToOpen.insert("/sensors/mlx");
	topicsToOpen.insert("/vision/face_direction");
	
	return topicsToOpen;
}
