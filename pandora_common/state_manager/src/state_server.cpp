/** 
 * File Description: State Manager - Server Implementation
 * 
 * Contents: Methods for implementing nodes state changes
 * Methods: watchdog, setStateNames, clientStateInformation, registerNode,
 * 			 registerNodeTransition, nodeListed, performNodeCensus
 * 		 
 * Author: Software Architecture Team
 * 
 * Date: 15 April 2011
 * 
 * Change History: -
 */

#include "state_manager/state_server.h"

StateServer::StateServer() : 
			_nh(),	
			_stateChangeActionServer(_nh, "/robot/state/change", 
				boost::bind(&StateServer::stateChangeExecuteCb, this, _1), false)

{
	setStateNames();
	
	
	//Create watchdog
	_watchdog = _nh.createTimer(ros::Duration(15),&StateServer::watchdog,this);
	_timer = _nh.createTimer(ros::Duration(1),&StateServer::publishDiagnostics,this);
	_watchdog.stop();
	_transitioning = false;
	
	//Subcriber and Publisher declaration
	_statePublisher = _nh.advertise<state_manager_communications::robotModeMsg>
						("/robot/state/clients", 5, true);	
						
	_registerService = _nh.advertiseService("/robot/state/register", 
						&StateServer::registerNode, this);
						
	_acknowledgeSubscriber = _nh.subscribe("/robot/state/server", 100,  
						&StateServer::clientStateInformation, this);
	
	_updater.setHardwareID("none");
	_updater.add("System State", this, &StateServer::stateDiagnostic);
	_updater.add("Nodes Initialization", this, &StateServer::initDiagnostic);
	
	ros::Duration w(10); //! Wait 10 seconds for all nodes to start.
	w.sleep();
	_previousState = 0;
	_currentState = 0;
	_numOfNodesAcked = 0;
		
	_stateChangeActionServer.start();
}

void StateServer::watchdog(const ros::TimerEvent&) {
	ROS_ERROR("State Transition Timeout");
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).currentState != _currentState)
			ROS_ERROR("Node %s timeout", nodeNames.at(i).nodeName.c_str());
	}
	sendStart(_currentState);
}

void StateServer::setStateNames() {
	stateNames[state_manager_communications::robotModeMsg::MODE_OFF] = "MODE_OFF";
	stateNames[state_manager_communications::robotModeMsg::MODE_EXPLORATION] = "MODE_EXPLORATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_IDENTIFICATION] = "MODE_IDENTIFICATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_APPROACH] = "MODE_ARM_APPROACH";
	stateNames[state_manager_communications::robotModeMsg::MODE_DF_HOLD] = "MODE_DF_HOLD";
	stateNames[state_manager_communications::robotModeMsg::MODE_SEMI_AUTONOMOUS] = "MODE_SEMI_AUTONOMOUS";
	stateNames[state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION] = "MODE_TELEOPERATED_LOCOMOTION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_TELEOPERATION] = "MODE_ARM_TELEOPERATION";
	stateNames[state_manager_communications::robotModeMsg::MODE_ARM_TELEOPERATION_TUCK] = "MODE_ARM_TELEOPERATION_TUCK";
	stateNames[state_manager_communications::robotModeMsg::MODE_TERMINATING] = " MODE_TERMINATING";
}

void StateServer::clientStateInformation(const state_manager_communications::robotModeMsgConstPtr& msg) {
	ROS_INFO("Received new msg from state client %s", msg->nodeName.c_str());
	
	if (msg->type ==  state_manager_communications::robotModeMsg::TYPE_REQUEST) {
		ROS_INFO("%s requested state transition to %i", msg->nodeName.c_str(),msg->mode);
		sendTransitionRequest(msg->mode);
		
	} else if (msg->type == state_manager_communications::robotModeMsg::TYPE_ACK) {
		registerNodeTransition(msg);
		
	} else {
		ROS_ERROR("State Server received a state message, but it was not understandable.");
	}
}

bool StateServer::registerNode(state_manager_communications::registerNodeSrv::Request& rq,
								state_manager_communications::registerNodeSrv::Response &rs) {
	
	if (rq.type == state_manager_communications::registerNodeSrvRequest::TYPE_STARTED){
	
		if (!nodeListed(rq.nodeName)) {
			stateNode node;
			node.nodeName = rq.nodeName;
			ROS_INFO("Node %s has registered as STARTED in state manager", node.nodeName.c_str());
			node.currentState = 0;
			node.previousState = 0;
			node.isInit = false;
			nodeNames.push_back(node);
		} else {
			ROS_ERROR("Node %s tried to register in state manager, but is already registerd", 
			rq.nodeName.c_str());
		}
		
	}
	else if (rq.type == state_manager_communications::registerNodeSrvRequest::TYPE_INITIALIZED) {
		if (nodeListed(rq.nodeName)) {
			ROS_INFO("Node %s has checked in as INITIALIZED in state manager", rq.nodeName.c_str());
			nodeNames.at( nodeIndex(rq.nodeName) ).isInit = true; 
		} 
		else {
			ROS_ERROR("Node %s tried to check in as INITIALIZED in state manager , but is not registerd", 
			rq.nodeName.c_str());
		}
		
	} 
	else {
		ROS_ERROR("State Server received a node registry message, but it was not understandable.");
	}
	
	return true;
	
}

void StateServer::registerNodeTransition(const state_manager_communications::robotModeMsgConstPtr& msg) {
	if (msg->mode != _currentState){
		ROS_ERROR("Received and ACK from %s, but it transitioned to a wrong state", 
		msg->nodeName.c_str()); 
		return;
	}
	if (!nodeListed(msg->nodeName)) {
		ROS_ERROR("Received an ACK from an unknown node %s",msg->nodeName.c_str());
		return;
	}
	ROS_INFO("%s transitioned sucessfully",msg->nodeName.c_str());
	_numOfNodesAcked++;
	
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).nodeName == msg->nodeName){
			nodeNames.at(i).previousState = nodeNames.at(i).currentState;
			nodeNames.at(i).currentState = _currentState;
			continue;
		}
	}
	if (_numOfNodesAcked >= nodeNames.size()) {
		sendStart(_currentState);
		ROS_INFO("System Successfully transitioned to %i",_currentState);
	}	
}

int StateServer::nodeIndex(std::string name) {
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if (nodeNames.at(i).nodeName == name)
			return i;
	}
	return -1;
}

bool StateServer::nodeListed(std::string name) {
	return nodeIndex(name)!=-1 ? true : false ;  
}




void StateServer::performNodeCensus() {
	nodeNames.clear();
	
	//First retrieve all nodes
	ros::V_string nodes;
	ros::master::getNodes (nodes);
	
	
	for (unsigned int i = 0; i < nodes.size(); i++){
		stateNode node;
		node.nodeName = nodes.at(i);
		if (node.nodeName == "/rosout" || node.nodeName == ros::this_node::getName())
			continue;
		ROS_INFO("%s",node.nodeName.c_str());
		node.currentState = 0;
		nodeNames.push_back(node);	
	}
}

void StateServer::sendTransitionRequest(int newState) {
	state_manager_communications::robotModeMsg msg;
	ROS_INFO("Changing system state to %i", newState);
	_previousState = _currentState;
	_currentState = newState;
	msg.nodeName = "";
	msg.mode = newState;
	msg.type = state_manager_communications::robotModeMsg::TYPE_TRANSITION;
	_statePublisher.publish(msg);	
	_numOfNodesAcked = 0;
	ROS_INFO("Informed all state clients for state change");
	_watchdog.start();
	_transitioning = true;
}

void StateServer::sendStart(int newState) {
	state_manager_communications::robotModeMsg msg;
	_watchdog.stop(); //Disable watchdog
	msg.nodeName = "";
	msg.mode = newState;
	_numOfNodesAcked = 0;
	msg.type = state_manager_communications::robotModeMsg::TYPE_START;
	_statePublisher.publish(msg);
	_transitioning = false;
	statusNode();	
	
	{
        boost::lock_guard<boost::mutex> lock(_mut);
        systemTransitioned = true;
    }
    _cond.notify_one();
	
	
}

void StateServer::statusNode() {
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		ROS_INFO("Node %s is at state %i", nodeNames.at(i).nodeName.c_str(), 
		nodeNames.at(i).currentState);	
	}
}

std::string StateServer::getStateName(int state){
	if (state < NUM_OF_STATES) 
		return stateNames[state];
	else
		return "INVALID";
}

void StateServer::stateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    bool allOk = true;
    if(!_transitioning) {
		for (unsigned int i = 0; i < nodeNames.size(); i++) {
			if (nodeNames.at(i).currentState != _currentState) {
				allOk = false;
			}
		}
	}
	if (allOk && getStateName(_currentState) != "INVALID") {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, 
		getStateName(_currentState));
	}
	else {
		std::string timedOutNodes;
		for (unsigned int i = 0; i < nodeNames.size(); i++) {
			if (nodeNames.at(i).currentState != _currentState) {
				timedOutNodes = timedOutNodes + " " + nodeNames.at(i).nodeName.c_str();
			}
		}
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, 
		"Timed Out Nodes:" + timedOutNodes);
	}
	stat.add("System Current State ", getStateName(_currentState));
	stat.add("System Previous State ", getStateName(_previousState));
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		stat.add(nodeNames.at(i).nodeName, getStateName(nodeNames.at(i).currentState) + 
		"\t(" + getStateName(nodeNames.at(i).previousState)+")");
	}
}

void StateServer::initDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    bool allOk = true;
    
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		if ( !nodeNames.at(i).isInit ) {
			allOk = false;
		}
	}
	
	if (allOk) {
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, 
		"All nodes have been initialiazed");
	}
	else {
		std::string notInitiatedNodes;
		for (unsigned int i = 0; i < nodeNames.size(); i++) {
			if ( !nodeNames.at(i).isInit ) {
				notInitiatedNodes = notInitiatedNodes + " " + nodeNames.at(i).nodeName.c_str();
			}
		}
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, 
		"Not initialized Nodes:" + notInitiatedNodes);
	}
	for (unsigned int i = 0; i < nodeNames.size(); i++) {
		std::string nodeInitializationStatus;
		if (nodeNames.at(i).isInit){
			nodeInitializationStatus = "Initialized" ;
		}
		else{
			nodeInitializationStatus = "Started" ;
		}
		stat.add(nodeNames.at(i).nodeName, nodeInitializationStatus );
	}
}

void StateServer::publishDiagnostics(const ros::TimerEvent&) {
	_updater.update();
}

void StateServer::stateChangeExecuteCb(const state_manager_communications::RobotModeGoalConstPtr &goal){
		
	ROS_INFO("%s requested state transition to %i", goal->modeMsg.nodeName.c_str(),goal->modeMsg.mode);
	
	sendTransitionRequest(goal->modeMsg.mode);
	
	ros::Duration dur(0.1);
	
	boost::unique_lock<boost::mutex> lock(_mut);
	systemTransitioned = false;
    while(!systemTransitioned)
    {
        _cond.wait(lock);
    }
	
	_stateChangeActionServer.setSucceeded();
	
	
}


int main(int argc, char** argv) {
	ros::init(argc, argv,"StateServer");
	StateServer server;
	ros::spin();
}
