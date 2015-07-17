/**
 * File Description: State Manager - Server Definition
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
 *
 */

#ifndef STATE_SERVER
#define STATE_SERVER

#include "ros/ros.h"
#include "state_manager_msgs/RobotModeMsg.h"
#include "state_manager_msgs/RegisterNodeSrv.h"
#include "state_manager_msgs/GetStateInfo.h"
#include "state_manager_msgs/ChangeMode.h"
#include "diagnostic_updater/diagnostic_updater.h"

#include <actionlib/server/simple_action_server.h>
#include "state_manager_msgs/RobotModeAction.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

#define NUM_OF_STATES 11	//! The defined robot states.

/**
 * State Information Internal Structure.
 */

typedef actionlib::SimpleActionServer<state_manager_msgs::RobotModeAction> RobotModeActionServer;


struct stateNode {
  std::string nodeName;
  int currentState;
  int previousState;
  bool isInit;
};

class StateServer {
  private:
    /**
     * The ROS Node Handle.
     */
    ros::NodeHandle _nh;

    /**
     * Publisher to the state topic.
     */
    ros::Publisher _statePublisher;

    /**
     * Subscriber to the state topic.
     */
    ros::Subscriber _acknowledgeSubscriber;

    /**
     * Register service.
     */
    ros::ServiceServer _registerService;

    /**
     * State info service.
     */
    ros::ServiceServer _stateInfoService;

    /**
     * Service to change the current mode.
     */
    ros::ServiceServer _changeModeService;

    /**
     * Timer watchdog.
     */
    ros::Timer _watchdog;

    /**
     * Timer for monitoring.
     */
    ros::Timer _timer;

    RobotModeActionServer _stateChangeActionServer;

    boost::mutex _mut;
    boost::condition_variable _cond;

    /**
     * The diagnostic updater for posting on runtime_monitor.
     */
    diagnostic_updater::Updater _updater;

    /**
     * Variable indicating that the system is transitioning.
     */
    bool _transitioning;

    /**
     * Client State information getter (callback).
     */
    void clientStateInformation(const state_manager_msgs::RobotModeMsgConstPtr&);

    /**
     * Register node to a new state.
     */
    void registerNodeTransition(const state_manager_msgs::RobotModeMsgConstPtr& msg);

    /**
     * Register a node.
     */
    bool registerNode(state_manager_msgs::RegisterNodeSrv::Request& req,
                      state_manager_msgs::RegisterNodeSrv::Response& res);

    /**
     * Returns the current or the previous state.
     */
    bool getStateInfo(state_manager_msgs::GetStateInfo::Request& req,
                      state_manager_msgs::GetStateInfo::Response& res);

    /**
     * Change the robot state
     */
    bool changeMode(state_manager_msgs::ChangeMode::Request& req,
                    state_manager_msgs::ChangeMode::Response& res);
    /**
     * Send a transition request to all nodes.
     */
    void sendTransitionRequest(int newState);

    /**
     * Send the start msg to nodes.
     */
    void sendStart(int newState);

    /**
     * The current state.
     */
    int _currentState;

    /**
     * The previous state.
     */
    int _previousState;

    /**
     * The number of nodes that Ack'ed.
     */
    unsigned int _numOfNodesAcked;

    /**
     * A vector of the subscribed node names.
     */
    std::vector<stateNode> nodeNames;

    /**
     * The names of the states.
     */
    std::string stateNames[NUM_OF_STATES];

    /**
     * Get the state the system is.
     */
    inline std::string getStateName(int state);

    /**
     * Perform a census of all running nodes.
     */
    void performNodeCensus();

    /**
     * Get index of node in nodeNames.
     * @param name the name of the node being searched.
     */
    int nodeIndex(std::string name);

    /**
     * Checks if a node is listed.
     * @param name the name of the node being searched.
     */
    bool nodeListed(std::string name);

    /**
     * Watchdog timer waiting for system transition.
     */
    void watchdog(const ros::TimerEvent&);

    /**
     * Giving info about nodes and current state.
     */
    void statusNode();

    /**
     * Setting the name of state.
     */
    void setStateNames();

    /**
     * Monitors the behavior of state manager.
     */
    void stateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

    /**
     * Monitors the initialization of the nodes.
     */
    void initDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&);

    /**
     * Displays the diagnostic messages.
     */
    void publishDiagnostics(const ros::TimerEvent&);

    void stateChangeExecuteCb(const state_manager_msgs::RobotModeGoalConstPtr &goal);

    bool systemTransitioned;

  public:

    /**
     * Constructor.
     */
    StateServer();

};

#endif
