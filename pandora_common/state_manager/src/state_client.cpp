/** 
 * File Description: State Manager - Client Implementation
 * 
 * Contents: Methods for implementing nodes as clients for changing
 * 			 states of operation 	
 * Methods:	 serverStateInformation, clientRegister, startTransition,
 * 			 transitionComplete, completeTransition, transitionToState
 * 		 
 * Author: Software Architecture Team
 * 
 * Date: 15 April 2011
 * 
 * Change History: -
 */


#include "state_manager/state_client.h"

StateClient::StateClient (bool doRegister) : _nh() {
	name = ros::this_node::getName();
	_acknowledgePublisher = _nh.advertise<state_manager_msgs::RobotModeMsg>
	("/robot/state/server", 5, true);
	
	_stateSubscriber = _nh.subscribe("/robot/state/clients",10, 
	&StateClient::serverStateInformation, this);		
	
	if (doRegister)
		clientRegister();
}

void StateClient::clientRegister()
{
	while (!ros::service::waitForService("/robot/state/register", 
	ros::Duration(.1)) && ros::ok()) {
		
		ROS_ERROR("[%s] Couldn't find service /robot/state/register", name.c_str());
		ros::spinOnce();
	}
	_registerServiceClient = _nh.serviceClient<state_manager_msgs::RegisterNodeSrv>
	("/robot/state/register");
	
	state_manager_msgs::RegisterNodeSrv rq;
	rq.request.nodeName = name;
	rq.request.type = state_manager_msgs::RegisterNodeSrvRequest::TYPE_STARTED;
	
	while (!_registerServiceClient.call(rq) && ros::ok())
		ROS_ERROR("[%s] Failed to register node. Retrying...", name.c_str());
}


void StateClient::clientInitialize()
{
	while (!ros::service::waitForService("/robot/state/register", 
	ros::Duration(.1)) && ros::ok()) {
		
		ROS_ERROR("[%s] Couldn't find service /robot/state/register", name.c_str());
		ros::spinOnce();
	}
	_registerServiceClient = _nh.serviceClient<state_manager_msgs::RegisterNodeSrv>
	("/robot/state/register");
	
	state_manager_msgs::RegisterNodeSrv rq;
	rq.request.nodeName = name;
	rq.request.type = state_manager_msgs::RegisterNodeSrvRequest::TYPE_INITIALIZED;
	
	while (!_registerServiceClient.call(rq) && ros::ok())
		ROS_ERROR("[%s] Failed to register node. Retrying...", name.c_str());
}

void StateClient::startTransition(int newState) {

	ROS_INFO("[%s] Starting Transition to state %i",name.c_str(), newState);	
	transitionComplete(newState);
}

void StateClient::transitionComplete(int newState){

	ROS_INFO("[%s] Node Transition to state %i Completed",name.c_str(), newState);	

	state_manager_msgs::RobotModeMsg msg;
	msg.nodeName = name;
	msg.mode = newState;
	msg.type = state_manager_msgs::RobotModeMsg::TYPE_ACK;
	_acknowledgePublisher.publish(msg);
}

void StateClient::completeTransition(){
	ROS_INFO("[%s] System Transitioned, starting work", name.c_str());
}

void StateClient::transitionToState(int newState){

	ROS_INFO("[%s] Requesting transition to state %i",name.c_str(), newState);	
	state_manager_msgs::RobotModeMsg msg;
	msg.nodeName = name;
	msg.mode = newState;
	msg.type = state_manager_msgs::RobotModeMsg::TYPE_REQUEST;
	_acknowledgePublisher.publish(msg);
	
}

void StateClient::serverStateInformation(const state_manager_msgs::RobotModeMsgConstPtr& msg) {
	ROS_INFO("[%s] Received new information from state server",name.c_str());
	
	if (msg->type == state_manager_msgs::RobotModeMsg::TYPE_TRANSITION) {
		startTransition(msg->mode);
		
	} else if (msg->type == state_manager_msgs::RobotModeMsg::TYPE_START) {
		completeTransition();
		
	} else {
		ROS_ERROR("[%s] StateClient received a new state command, that is not understandable",
		name.c_str());
	}
}


