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

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "ros/ros.h"
#include "watchdog_communications/watchdogSrv.h"
#include "watchdog_communications/watchdogResetMsg.h"



/**
 * A watchdog class for creating a watchdog.
 */
class Watchdog {
	private: 
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Watchdog Reset Publisher.
		 */
		ros::Publisher _watchdogResetPublisher;
		
		/**
		 * Service for starting and stopping watchdog.
		 */
		ros::ServiceClient _registerService;
	
		/**
		 * The name of the watchdog.
		 */
		std::string _name;
		
		/**
		 * The duration to count before reseting the watchdog.
		 */
		ros::Duration _wdtCounter;
		
		/**
		 * A boolean variable indicating if the wdt is enabled.
		 */
		bool _started;
		
		/**
		 * ROS Service Server
		 */
		ros::ServiceServer _timeoutSrv;

	public:
		/**
		 * Constructor.
		 * @param watchDogName the name of the watchdog
		 * @param duration the duration of the timer until reset
		 */
		Watchdog(std::string watchDogName, ros::Duration duration);

		/**
		 * Destructor for throwing error if watchdog is being destructed but
		 * watchdog is still running.
		 */
		~Watchdog();
		
		/**
		 * Reset the watchdog and start timing again.
		 */
		void reset();
	
		/**
		 * Starting the wdt.
		 */
		void start();
		
		/**
		 * Stop the wdt timer.
		 */
		void stop();
};

#endif
