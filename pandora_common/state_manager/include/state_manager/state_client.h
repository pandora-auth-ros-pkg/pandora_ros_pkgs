/** 
 * File Description: State Manager - Client Definition
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

#ifndef STATE_CLIENT
#define STATE_CLIENT

#include "ros/ros.h"
#include "state_manager_msgs/RobotModeMsg.h"
#include "state_manager_msgs/RegisterNodeSrv.h"

class StateClient {
	private:
		/**
		 * The ROS Node Handle
		 */
		ros::NodeHandle _nh;
		
		/**
		 * Publisher to the state topic
		 */
		ros::Publisher _acknowledgePublisher;
		
		/**
		 * Subscriber to the state topic
		 */
		ros::Subscriber _stateSubscriber;
		
		/**
		 * Callback for receiving state info from server.
		 */
		void serverStateInformation(const state_manager_msgs::RobotModeMsgConstPtr&);
		
		void clientRegister();
		
		/**
		 * The node name.
		 */
		std::string name;
		
		/**
		 * Client for registering
		 */
		ros::ServiceClient _registerServiceClient;
		
		/**
		 * True to register to server.
		 */
		bool registerToServer;
		
	
	protected: 
	
		/**
	   * Check in node as Initialized
	   * @param newState the new state
	   */
		void clientInitialize();
	
		/**
	   * Start the client transition.
	   * @param newState the new state
	   */
		virtual void startTransition(int newState);
		
		/**
		 * Update the system that we have finished transitioning.
		 * @param newState the state at which the client has transitioned.
		 */
		void transitionComplete(int newState);
	
	public:
	
		/**
		 * Constructor.
		 */	
		StateClient (bool doRegister = true);
		
		/**
		 * Update the client that the system has tranisitioned.
		 */
		virtual void completeTransition();
	
		/**
		 * Ask the system to change the transition/
		 * @param newState the state to transition to
		 */
		void transitionToState(int newState);
	
	
};

#endif
