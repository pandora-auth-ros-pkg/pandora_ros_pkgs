#include "victim_fusion/victim_fusion.h"

int main (int argc, char **argv)
{
	ros::init(argc,argv,"victim_fusion",ros::init_options::NoSigintHandler);
	VictimFusion victimFusion;
	ROS_INFO("Beginning Victim Fusion node");
	ros::spin();
	return 0;
}
