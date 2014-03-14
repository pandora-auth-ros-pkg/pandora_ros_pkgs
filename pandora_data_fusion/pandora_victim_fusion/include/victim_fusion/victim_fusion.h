#ifndef DATA_FUSION_H
#define DATA_FUSION_H

#include "ros/ros.h"
#include "state_manager/state_client.h"
#include "enums.h"
#include "sensors_container.h"

class VictimFusionState;	//!< Forward declaration of class DataFusionState

typedef boost::shared_ptr<VictimFusionState> StatePtr;

class VictimFusion : public StateClient 
{
	private:

		ros::NodeHandle n;					//!< ROS node handle
		ros::Subscriber CO2Subscriber;		//!< CO2 subscriber
		ros::Subscriber TpaSubscriber;		//!< Tpa subscriber
		ros::Subscriber MlxSubscriber;		//!< Mlx subscriber
		ros::Subscriber HoleDirectionSubscriber;
		ros::Subscriber HolePositionSubscriber;
		ros::Subscriber FaceDirectionSubscriber;
		ros::Subscriber MotionSubscriber;
		
		std::set<std::string> openSubscribers;
		
		SensorsContainer sensorsContainer; 
		
		StatePtr statePtr;		//!<	Current state of robot

	public:
		
		VictimFusion(void);
		
		void serveCO2(const controllers_and_sensors_communications::co2Msg& msg);
		
		void serveTpa(const controllers_and_sensors_communications::tpaMsg& msg);
		
		void serveMlx(const controllers_and_sensors_communications::mlxTempMsg& msg);
		
		void serveHoleDirection(const vision_communications::HolesDirectionsVectorMsg& msg);
		
		void serveHolePosition(const vision_communications::HolesPositionsVectorMsg& msg);
		
		void serveFaceDirection(const vision_communications::FaceDirectionMsg& msg);
		
		void serveMotion(const vision_communications::MotionMsg& msg);
		
		void startTransition(int newState);
		
		void completeTransition(void);
				
		void toggleSubscribers();
		
		void loadParams(std::string state);
};


#endif
