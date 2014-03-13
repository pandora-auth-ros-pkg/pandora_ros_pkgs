#include "victim_fusion/state.h"

StateAnyOther::StateAnyOther(const SensorsContainer& container) : VictimFusionState(container)
{
	
}

void StateAnyOther::fuse(SENSOR latestAlert)
{
	ROS_DEBUG("[DATA_FUSION] Not good state, latestAlert %d",latestAlert);
	return;
}

std::set<std::string> StateAnyOther::topicsToSubscribe()
{
	// return an empty set to shutdown all subscribers
	std::set<std::string> topicsToOpen;
	topicsToOpen.clear();
	return topicsToOpen;
}
