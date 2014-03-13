/** 
 * File Description: Watchdog Monitor Definition
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

#ifndef WATCHDOG_MONITOR_H
#define WATCHDOG_MONITOR_H


#include "ros/ros.h"
#include "watchdog_communications/watchdogSrv.h"
#include "watchdog_communications/watchdogResetMsg.h"
#include "watchdog_communications/wdtTimeoutNotificationMsg.h"

struct wdtInformation {
	ros::Duration wdtDuration;
	ros::Time lastReset;	
};

class WatchdogMonitor {
	
	private:
		/**
		 * The ROS node handle.
		 */
		ros::NodeHandle _nh;
	
		/**
		 * A subscriber to the watchdog reset topic.
		 */
		ros::Subscriber _resetSubscriber;
		
		/**
		 * A publisher for the timeouts.
		 */
		ros::Publisher _timeoutsPublisher;
		
		/**
		 * A service for registering WDT.
		 */
		ros::ServiceServer _registerWDTService;
		
		/**
		 * A map for storing the wdt information.
		 */
		std::map<std::string, wdtInformation> _activeWdt;
		
		/**
		 * The actual timer watching the wdts
		 */
		ros::Timer _wdtTimer;		
		
		/**
		 * The timer callback that informs for timeouts
		 */
		void checkForTimeouts(const ros::TimerEvent&);
		
		/**
		 * Register the WDT service callback
		 */
		bool registerWdtService(watchdog_communications::watchdogSrv::Request&, 
								watchdog_communications::watchdogSrv::Response&);
		
		/**
		 * Callback for receiving resets.
		 */
		void receiveReset(const watchdog_communications::watchdogResetMsgConstPtr&);
		
	public:
		/**
		 * Costructor
		 * @param checkFrequency: the frequency to check for resets
		 */
		WatchdogMonitor(float checkFrequency);
};

#endif
