/**
 * File Description: Instantiation of MutexGuard
 * 
 * Author: Software Architecture Team
 *
 */

#include "mutex_guard.h"

int main(int argc, char** argv) {
	std::string name = "mutex";
	name += argv[1];
	ros::init(argc,argv,name);	
	MutexGuard g(argv[1]);
	ros::spin();
}
