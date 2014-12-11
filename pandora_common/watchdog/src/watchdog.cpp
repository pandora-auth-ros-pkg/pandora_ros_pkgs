/** 
 * File Description: Watchdog Definition
 * 
 * Contents: Methods that start, stop and reset the watchdog timer
 * Methods: reset, start, stop
 * 
 * Author: Software Architecture Team
 * 
 * Date: 2 May 2011
 * 
 * Change History: -
 * 
 */

#include "watchdog.h"

Watchdog::Watchdog(std::string wdtName, ros::Duration duration) {
	_name = ros::this_node::getName() + "/" + wdtName;
	_wdtCounter = duration;
	_started = false;
	
	_watchdogResetPublisher = _nh.advertise<watchdog_msgs::watchdogResetMsg>
								("/robot/watchdog", 100);
	
	int retries = 0;
	while (!ros::service::waitForService("/robot/watchdogRegistry", ros::Duration(.2)) && ros::ok()) {
		retries++;
		if (retries > 10)
			ROS_ERROR("Couldn't find service watchdogRegistry.");
		ros::spinOnce();
	}
	_registerService = _nh.serviceClient<watchdog_msgs::watchdogSrv>
						("/robot/watchdogRegistry");
}

Watchdog::~Watchdog() {
	if (_started) {
		ROS_ERROR("Watchdog %s is being destructed, but has not been stopped", _name.c_str());
		stop();
	}
}

void Watchdog::start() {
	if (_started) return;
	watchdog_msgs::watchdogSrv srv;
	srv.request.watchdogName = _name;
	srv.request.type = watchdog_msgs::watchdogSrv::Request::TYPE_START;
	srv.request.timeoutDuration = _wdtCounter;
	if (_registerService.call(srv)) {
		_started = true;
		ROS_INFO("Watchdog started");
	} else {
		ROS_ERROR("Failed to contact watchdog service");
	}
}

void Watchdog::stop() {
	if (!_started) return;
	watchdog_msgs::watchdogSrv srv;
	srv.request.watchdogName = _name;
	srv.request.type = watchdog_msgs::watchdogSrv::Request::TYPE_STOP;
	if (_registerService.call(srv)) {
		_started = false;
		ROS_INFO("Watchdog %s stopped",_name.c_str());
	} else {
		ROS_ERROR("Failed to contact watchdog service for %s", _name.c_str());
	}	
}

void Watchdog::reset() {
		watchdog_msgs::watchdogResetMsg msg;
		msg.watchdogName = _name;
		msg.header.stamp = ros::Time::now();
		_watchdogResetPublisher.publish(msg);
}
