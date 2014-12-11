/** 
 * File Description: Watchdog Monitor Implementation
 * 
 * Methods: checkForTimeouts, registerWdtService, registerWdtService,
 * 			receiveReset, WatchdogMonitor
 * 
 * Author: Software Architecture Team
 * 
 * Date: 2 May 2011
 * 
 * Change History: -
 * 
 */

#include "watchdog_monitor.h"

WatchdogMonitor::WatchdogMonitor(float checkFrequency) {
	_resetSubscriber = _nh.subscribe("/robot/watchdog", 1000, 
						&WatchdogMonitor::receiveReset, this);
						
	_registerWDTService = _nh.advertiseService("/robot/watchdogRegistry", 
						&WatchdogMonitor::registerWdtService, this);
							
	_timeoutsPublisher = _nh.advertise<watchdog_msgs::wdtTimeoutNotificationMsg>
						("/robot/watchdogResets", 1000);
						
	_wdtTimer = _nh.createTimer(ros::Duration(checkFrequency),
						&WatchdogMonitor::checkForTimeouts, this);
}

void WatchdogMonitor::checkForTimeouts(const ros::TimerEvent&) {
	std::map<std::string, wdtInformation>::iterator currentWdt;
	
	for (currentWdt=_activeWdt.begin(); currentWdt != _activeWdt.end(); currentWdt++ ) {
		wdtInformation info = (*currentWdt).second;
		
		if (info.lastReset + info.wdtDuration < ros::Time::now()) {
			ROS_ERROR("Watchdog %s timedout",(*currentWdt).first.c_str());
			watchdog_msgs::wdtTimeoutNotificationMsg msg;
			msg.lastReset = info.lastReset;
			msg.wdtName = (*currentWdt).first;
			msg.nodeName = msg.wdtName.substr(0,msg.wdtName.rfind("/"));
			_timeoutsPublisher.publish(msg);
		}
	}
}

bool WatchdogMonitor::registerWdtService(watchdog_msgs::watchdogSrv::Request& rq, 
										 watchdog_msgs::watchdogSrv::Response&) {
											 
	if (rq.type == watchdog_msgs::watchdogSrv::Request::TYPE_START) {
		if (_activeWdt.count(rq.watchdogName) > 0) return false; //! WDT already exists
		wdtInformation info;
		info.wdtDuration = rq.timeoutDuration;
		info.lastReset = ros::Time::now();
		_activeWdt[rq.watchdogName] = info;
		return true;
	} else {
		std::map<std::string, wdtInformation>::iterator wdtToBeStopped;
		wdtToBeStopped = _activeWdt.find(rq.watchdogName);
		if (_activeWdt.count(rq.watchdogName) == 0) return false; //! WDT does not exist
		_activeWdt.erase(wdtToBeStopped);		
		return true;
	}
}

void WatchdogMonitor::receiveReset(const watchdog_msgs::watchdogResetMsgConstPtr& msg) {
	_activeWdt[msg->watchdogName].lastReset = msg->header.stamp;
}

int main(int argc, char **argv) {
	ros::init(argc,argv,"watchdogMonitor");
	
	WatchdogMonitor wdt(1);
	ros::spin();
}
