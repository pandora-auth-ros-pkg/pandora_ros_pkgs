#include "stabilizer_control.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stabilizer_control_node");
	StabilizerController stabilizerController;
	ros::spin();
}
