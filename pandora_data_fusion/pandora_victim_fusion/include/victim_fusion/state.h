#ifndef STATE_H
#define STATE_H

#include "victim_fusion.h"
#include "data_fusion_communications/VictimVerificationMsg.h"
#include "data_fusion_communications/FusionGlobalMsg.h"


class VictimFusionState
{
	protected:
		const SensorsContainer& sensorsContainer;
		
	public:
		VictimFusionState(const SensorsContainer& sensorsContainer);
		virtual void fuse(SENSOR latestAlert) = 0;
		virtual std::set<std::string> topicsToSubscribe() = 0;
};


class StateExplorationOrIdentification : public VictimFusionState 
{
	private:
		//params
		float _TPA_THRESHOLD, _MLX_THRESHOLD, _HOLE_THRESHOLD, _STEREO_HOLE_THRESHOLD;
	public:
		StateExplorationOrIdentification(const SensorsContainer& sensorsContainer);
		virtual void fuse(SENSOR latestAlert);
		virtual std::set<std::string> topicsToSubscribe();
};


class StateArmApproach : public VictimFusionState
{
	private:
		//params
		float _MLX_THRESHOLD, _FACE_THRESHOLD;
	public:
		StateArmApproach(const SensorsContainer& sensorsContainer);
		virtual void fuse(SENSOR latestAlert);
		virtual std::set<std::string> topicsToSubscribe();
};

class StateDfHold : public VictimFusionState
{
	private:
		ros::Publisher _dataFusionGlobalPublisher;
		ros::Publisher _victimVerificationPublisher;
		
		//params
		float _MLX_THRESHOLD, _FACE_THRESHOLD, _CO2_THRESHOLD, _MOTION_THRESHOLD;
		
	public:
		StateDfHold(const SensorsContainer& sensorsContainer);
		virtual void fuse(SENSOR latestAlert);
		virtual std::set<std::string> topicsToSubscribe();
};

class StateAnyOther : public VictimFusionState
{
	public:
		StateAnyOther(const SensorsContainer& sensorsContainer);
		virtual void fuse(SENSOR latestAlert);
		virtual std::set<std::string> topicsToSubscribe();
};

#endif
