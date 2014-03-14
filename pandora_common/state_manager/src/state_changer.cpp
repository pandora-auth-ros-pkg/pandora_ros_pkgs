/** 
 * File Description: Instatiation of StateManager for changing robot states
 * 
 * Author: Software Architecture Team
 * 
 * Date: 15 April 2011
 * 
 * Change History: -
 * 
 */

#include "state_manager/state_client.h"

int main(int argc, char** argv){
	ros::init(argc,argv,"stateClient");
	StateClient client(false);
	ros::Duration w(.1);
	w.sleep();
	
	ROS_INFO("Requested to change in state %s ", argv[1]);

	client.transitionToState(atoi(argv[1]));
	ros::spin();
}
