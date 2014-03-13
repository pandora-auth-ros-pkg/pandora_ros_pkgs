/**
 * File Description: Implementation of MutexGuard - Server Side
 * 
 * Contents: tryLock, unlock, serviceCallback, getStatus, getHolder 
 * 
 * Author: Software Architecture Team
 *
 */

#include "mutex_guard.h"

MutexGuard::MutexGuard(std::string mutexName) {
	holderName = "";
	locked = false;
	_name = "/mutexes/" + mutexName;
	_mutexServer = _nh.advertiseService(_name,&MutexGuard::serviceCallback,this);
}

bool MutexGuard::tryLock(std::string holder) {
	if (locked) {
		 ROS_INFO("[MutexGuard] %s tried to lock %s but failed.",holder.c_str(),_name.c_str());
		 return false;
	 }
	locked = true;
	holderName = holder;
	ROS_INFO("[MutexGuard] %s sucessfully locked mutex %s.",holder.c_str(),_name.c_str());
	return true;
}

bool MutexGuard::unlock(std::string holder) {
	if (!locked) {
		 ROS_ERROR("[MutexGuard] Node %s tried to unlock already unlocked mutex %s",
		 holder.c_str(),_name.c_str());
		 return false;		 
	 }
	if (holder != holderName) {
		 ROS_ERROR("[MutexGuard] %s tried to unlock %s mutex that does not hold",
		 holder.c_str(),_name.c_str());
		 return false;
	 }
	holderName = "";
	locked = false;
	return true;	
}

bool MutexGuard::getStatus() {
	return locked;
}

std::string MutexGuard::getHolder() {
	return holderName;
}

bool MutexGuard::serviceCallback(remote_mutex_communications::mutexSrv::Request& rq, 
				 remote_mutex_communications::mutexSrv::Response& rs) {
	switch (rq.requestType) {
		case remote_mutex_communications::mutexSrv::Request::TYPE_LOCK :
			if (tryLock(rq.requestor)) {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_LOCKED;
			} else {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_UNLOCKED;
			}
			break;
		case remote_mutex_communications::mutexSrv::Request::TYPE_UNLOCK :
			if (unlock(rq.requestor)) {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_UNLOCKED;
			} else {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_LOCKED;
			}
			break;
		case remote_mutex_communications::mutexSrv::Request::TYPE_POLL :	
			if (getStatus()) {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_LOCKED;
			} else {
				rs.status = remote_mutex_communications::mutexSrv::Response::STATUS_UNLOCKED;
			}		
			break;
		default:
			return false;
	}
	
	return true;
}
