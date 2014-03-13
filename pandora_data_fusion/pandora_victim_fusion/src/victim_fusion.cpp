#include "victim_fusion/victim_fusion.h"
#include "victim_fusion/state.h"

VictimFusion::VictimFusion(void)
{		
	startTransition(state_manager_communications::robotModeMsg::MODE_OFF); 
	clientInitialize();
}

void VictimFusion::serveTpa(const controllers_and_sensors_communications::tpaMsg& msg)
{
	sensorsContainer.tpaSensor.handleCallback(msg);	
	statePtr->fuse(_TPA);
}

void VictimFusion::serveCO2(const controllers_and_sensors_communications::co2Msg& msg)
{ 
	sensorsContainer.co2Sensor.handleCallback(msg);
	statePtr->fuse(_CO2);
}

void VictimFusion::serveMlx(const controllers_and_sensors_communications::mlxTempMsg& msg)
{
	sensorsContainer.mlxSensor.handleCallback(msg);
	statePtr->fuse(_MLX);
}

void VictimFusion::serveHoleDirection(const vision_communications::HolesDirectionsVectorMsg& msg)
{
	sensorsContainer.holeSensor.handleCallback(msg);
	statePtr->fuse(_HOLE);
}

void VictimFusion::serveHolePosition(const vision_communications::HolesPositionsVectorMsg& msg)
{
	sensorsContainer.stereoHoleSensor.handleCallback(msg);
	statePtr->fuse(_STEREO_HOLE);
}

void VictimFusion::serveFaceDirection(const vision_communications::FaceDirectionMsg& msg)
{
	sensorsContainer.faceSensor.handleCallback(msg);
	statePtr->fuse(_FACE);
}

void VictimFusion::serveMotion(const vision_communications::MotionMsg& msg)
{
	sensorsContainer.motionSensor.handleCallback(msg);
	statePtr->fuse(_MOTION);
	
}

void VictimFusion::startTransition(int newState)
{

	switch(newState){
		
		case state_manager_communications::robotModeMsg::MODE_EXPLORATION:
			loadParams("exploration_identification");
			statePtr.reset( new StateExplorationOrIdentification(sensorsContainer) );
			break;
			
		case state_manager_communications::robotModeMsg::MODE_IDENTIFICATION:
			loadParams("exploration_identification");
			statePtr.reset( new StateExplorationOrIdentification(sensorsContainer) );
			break;
			
		case state_manager_communications::robotModeMsg::MODE_ARM_APPROACH:
			loadParams("arm_approach");
			statePtr.reset( new StateArmApproach(sensorsContainer) );
			break;
			
		case state_manager_communications::robotModeMsg::MODE_DF_HOLD:
			loadParams("df_hold");
			statePtr.reset( new StateDfHold(sensorsContainer) );
			break;
			
		case state_manager_communications::robotModeMsg::MODE_TERMINATING:
			ROS_ERROR("[DATA_FUSION] Terminating node");
			exit(0);
			break;
			
		default:
			loadParams("defaults");
			statePtr.reset( new StateAnyOther(sensorsContainer) );
			break;
	}
	
	toggleSubscribers();
	
	transitionComplete(newState);
}

void VictimFusion::toggleSubscribers()
{
	std::set<std::string> subscribersToOpen = statePtr->topicsToSubscribe();
	
	// toggle /sensors/tpa
	if (subscribersToOpen.find("/sensors/thermal")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/sensors/thermal")!=openSubscribers.end())) {
			TpaSubscriber = n.subscribe("/sensors/thermal", 1, &VictimFusion::serveTpa,this);
		}
	} 
	else {
		if ((openSubscribers.find("/sensors/thermal")!=openSubscribers.end())) {
			TpaSubscriber.shutdown();
		}
	}
	
	// toggle /sensors/mlx
	if (subscribersToOpen.find("/sensors/mlx")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/sensors/mlx")!=openSubscribers.end())) {
			MlxSubscriber = n.subscribe("/sensors/mlx", 1, &VictimFusion::serveMlx,this);
		}
	} 
	else {
		if ((openSubscribers.find("/sensors/mlx")!=openSubscribers.end())) {
			MlxSubscriber.shutdown();
		}
	}
	
	// toggle /sensors/co2
	if (subscribersToOpen.find("/sensors/co2")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/sensors/co2")!=openSubscribers.end())) {
			CO2Subscriber = n.subscribe("/sensors/co2", 1, &VictimFusion::serveCO2,this);
		}
	} 
	else {
		if ((openSubscribers.find("/sensors/co2")!=openSubscribers.end())) {
			CO2Subscriber.shutdown();
		}
	}
	
	// toggle /vision/hole_direction
	if (subscribersToOpen.find("/vision/hole_direction")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/vision/hole_direction")!=openSubscribers.end())) {
			HoleDirectionSubscriber = n.subscribe("/vision/hole_direction", 1, &VictimFusion::serveHoleDirection,this);
		}
	} 
	else {
		if ((openSubscribers.find("/vision/hole_direction")!=openSubscribers.end())) {
			HoleDirectionSubscriber.shutdown();
		}
	}
	
	// toggle /stereo/hole_position
	if (subscribersToOpen.find("/stereo/hole_position")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/stereo/hole_position")!=openSubscribers.end())) {
			HolePositionSubscriber = n.subscribe("/stereo/hole_position", 1, &VictimFusion::serveHolePosition,this);
		}
	} 
	else {
		if ((openSubscribers.find("/stereo/hole_position")!=openSubscribers.end())) {
			HolePositionSubscriber.shutdown();
		}
	}
	
	// toggle /vision/face_direction
	if (subscribersToOpen.find("/vision/face_direction")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/vision/face_direction")!=openSubscribers.end())) {
			FaceDirectionSubscriber = n.subscribe("/vision/face_direction", 1, &VictimFusion::serveFaceDirection,this);
		}
	} 
	else {
		if ((openSubscribers.find("/vision/face_direction")!=openSubscribers.end())) {
			FaceDirectionSubscriber.shutdown();
		}
	}
	
	// toggle /vision/motion
	if (subscribersToOpen.find("/vision/motion")!=subscribersToOpen.end()) {
		if (!(openSubscribers.find("/vision/motion")!=openSubscribers.end())) {
			MotionSubscriber = n.subscribe("/vision/motion", 1, &VictimFusion::serveMotion,this);
		}
	} 
	else {
		if ((openSubscribers.find("/vision/motion")!=openSubscribers.end())) {
			MotionSubscriber.shutdown();
		}
	}
	
	openSubscribers = subscribersToOpen;
}


void VictimFusion::completeTransition(void)
{
	
}

void VictimFusion::loadParams(std::string state)
{
	sensorsContainer.mlxSensor.loadParams(state);
	sensorsContainer.tpaSensor.loadParams(state);
	sensorsContainer.co2Sensor.loadParams(state);
	sensorsContainer.holeSensor.loadParams(state);
	sensorsContainer.faceSensor.loadParams(state);
	sensorsContainer.motionSensor.loadParams(state);
	sensorsContainer.stereoHoleSensor.loadParams(state);
}

