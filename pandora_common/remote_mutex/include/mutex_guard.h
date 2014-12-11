/**
 * File Description: Definition of MutexGuard. Class MutexGuard 
 * 					 impements a remote mutex guard. This class implements
 * 					 the server-side
 * 
 * Contents: tryLock, unlock, serviceCallback, getStatus, getHolder
 * 
 * Author: Software Architecture Team
 *
 */

#ifndef MUTEX_GUARD_H
#define MUTEX_GUARD_H

#include "ros/ros.h"
#include "remote_mutex_msgs/mutexSrv.h"

class MutexGuard {
	private:
		/**
		 * The ROS node handle
		 */
		ros::NodeHandle _nh;
	
		/**
		 * The service server.
		 */
		 ros::ServiceServer _mutexServer;
		 
		/**
		 * The mutex name.
		 */
		 std::string _name;
		  
		 /**
		  * The current mutex holder.
		  */
		 std::string holderName;
		  
		 /**
		  * The guarded variable.
		  */
		 bool locked;
		   
		 /**
		  * Try to lock the mutex.
		  * @return true if we succeeded in locking the mutex
		  */
		 bool tryLock(std::string holder);
		   
		 /**
		  * The ROS Service callback for the mutex.
		  */
		 bool serviceCallback(remote_mutex_msgs::mutexSrv::Request &, remote_mutex_msgs::mutexSrv::Response &);
		   
		 /**
		  * Try to unlock the mutex for the given holder.
		  * @param holder: the mutex holder.
		  * @return true if the mutex has been unlocked sucessfully
		  */
		 bool unlock(std::string holder);
		   
	public:
		
		 /**
		  * The Constructor.
		  * @param mutexName
		  */
		 MutexGuard(std::string mutexName);
		
		 /**
		  * Gets the mutex status.
		  * @return true if the mutex is locked.
		  */
		 bool getStatus();
		
		 /**
		  * Returns the current mutex holder.
		  * @return the name of the holder, or an empty string if 
		  * noone is holding the mutex.
		  */
		 std::string getHolder();
};
#endif
