#include "victim_fusion/state.h"

StateExplorationOrIdentification::StateExplorationOrIdentification(const SensorsContainer& container) : VictimFusionState(container)
{
	if (!ros::param::get("/victim_fusion/exploration_ientification/tpa", _TPA_THRESHOLD))
		_TPA_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/exploration_ientification/mlx", _MLX_THRESHOLD))
		_MLX_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/exploration_ientification/hole", _HOLE_THRESHOLD))
		_HOLE_THRESHOLD = 0.5;
		
	if (!ros::param::get("/victim_fusion/exploration_ientification/stereo_hole", _STEREO_HOLE_THRESHOLD))
		_STEREO_HOLE_THRESHOLD = 0.5;		
	
}

void StateExplorationOrIdentification::fuse(SENSOR latestAlert)
{
	if (latestAlert == _MLX) {
		
		float mlxTrue = sensorsContainer.mlxSensor.calculateProbability();
		
		if (mlxTrue > _MLX_THRESHOLD) {
			sensorsContainer.mlxSensor.publishMessage(mlxTrue);
			return;
		}
	}
	else if (latestAlert == _TPA) {
		
		float tpaTrue = sensorsContainer.tpaSensor.calculateProbability();
		
		if (tpaTrue > _TPA_THRESHOLD) {
			sensorsContainer.tpaSensor.publishMessage(tpaTrue);
			return;
		}
	}
	else if (latestAlert ==_HOLE) {
		
		std::vector<float> holesTrue (sensorsContainer.holeSensor.calculateProbability());
		
		std::map<int, float> holesToPublish;
		
		for (int i = 0; i < holesTrue.size(); i++) {
						
			if (holesTrue[i] > _HOLE_THRESHOLD) {
				holesToPublish.insert(std::pair<int, float>(i,holesTrue[i]));
				ROS_INFO_NAMED("exploration_identification", "[VICTIM_FUSION %d] Got hole, probability: %f",__LINE__, holesTrue[i]);
			}			
		}
		
		sensorsContainer.holeSensor.publishMessage(holesToPublish);
	}
	else if (latestAlert ==_STEREO_HOLE) {
		
		std::vector<float> holesTrue (sensorsContainer.stereoHoleSensor.calculateProbability());
		
		std::map<int, float> holesToPublish;
		
		for (int i = 0; i < holesTrue.size(); i++) {
						
			if (holesTrue[i] > _STEREO_HOLE_THRESHOLD) {
				holesToPublish.insert(std::pair<int, float>(i,holesTrue[i]));
				ROS_INFO_NAMED("exploration_identification", "[VICTIM_FUSION %d] Got hole from stereo, probability: %f",__LINE__, holesTrue[i]);
			}			
		}
		
		sensorsContainer.stereoHoleSensor.publishMessage(holesToPublish);
	}
}

std::set<std::string> StateExplorationOrIdentification::topicsToSubscribe()
{
	std::set<std::string> topicsToOpen;
	topicsToOpen.insert("/sensors/thermal");
	topicsToOpen.insert("/sensors/mlx");
	topicsToOpen.insert("/vision/hole_direction");
	topicsToOpen.insert("/stereo/hole_position");
	
	return topicsToOpen;
}




