/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Software Architecture team
 * Maintainer:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef STATE_MANAGER_STATE_CLIENT_NODELET_H
#define STATE_MANAGER_STATE_CLIENT_NODELET_H

#include <string>
#include <map>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "state_manager_msgs/RobotModeMsg.h"
#include "state_manager_msgs/RegisterNodeSrv.h"

namespace state_manager
{

  class StateClientNodelet : public nodelet::Nodelet
  {
   public:
    static int ROBOT_STATES(const std::string& X) {
      static std::map<std::string, int>* m = NULL;
      static bool initialize = true;
      static boost::mutex mtx;

      mtx.lock();
      if (initialize)
      {
        initialize = false;
        m = new std::map<std::string, int>;
        m->insert(std::make_pair("OFF", 0));
        m->insert(std::make_pair("START_AUTONOMOUS", 1));
        m->insert(std::make_pair("EXPLORATION_RESCUE", 2));
        m->insert(std::make_pair("IDENTIFICATION", 3));
        m->insert(std::make_pair("SENSOR_HOLD", 4));
        m->insert(std::make_pair("SEMI_AUTONOMOUS", 5));
        m->insert(std::make_pair("TELEOPERATED_LOCOMOTION", 6));
        m->insert(std::make_pair("SENSOR_TEST", 7));
        m->insert(std::make_pair("EXPLORATION_MAPPING", 8));
        m->insert(std::make_pair("TERMINATING", 9));
      }
      mtx.unlock();

      return (*m)[X];
    }

    static std::string ROBOT_STATES(int X) {
      static std::map<int, std::string>* m = NULL;
      static bool initialize = true;
      static boost::mutex mtx;

      mtx.lock();
      if (initialize)
      {
        initialize = false;
        m = new std::map<int, std::string>;
        m->insert(std::make_pair(0, "OFF"));
        m->insert(std::make_pair(1, "START_AUTONOMOUS"));
        m->insert(std::make_pair(2, "EXPLORATION_RESCUE"));
        m->insert(std::make_pair(3, "IDENTIFICATION"));
        m->insert(std::make_pair(4, "SENSOR_HOLD"));
        m->insert(std::make_pair(5, "SEMI_AUTONOMOUS"));
        m->insert(std::make_pair(6, "TELEOPERATED_LOCOMOTION"));
        m->insert(std::make_pair(7, "SENSOR_TEST"));
        m->insert(std::make_pair(8, "EXPLORATION_MAPPING"));
        m->insert(std::make_pair(9, "TERMINATING"));
      }
      mtx.unlock();

      return (*m)[X];
    }

   public:
    /**
    * Constructor.
    */
    StateClientNodelet();
    virtual
    ~StateClientNodelet();

    virtual void
    onInit();

    ros::NodeHandle&
    getPublicNh();

    ros::NodeHandle&
    getPrivateNh();

    const std::string&
    getNodeName();

    void
    clientRegister();

    /**
    * Check in node as Initialized
    * @param newState the new state
    */
    void
    clientInitialize();

    /**
    * Start the client transition.
    * @param newState the new state
    */
    virtual void
    startTransition(int newState) = 0;

    /**
    * Update the system that we have finished transitioning.
    * @param newState the state at which the client has transitioned.
    */
    void
    transitionComplete(int newState);

    /**
    * Update the client that the system has tranisitioned.
    */
    virtual void
    completeTransition() = 0;

    /**
    * Ask the system to change the transition/
    * @param newState the state to transition to
    */
    void
    transitionToState(int newState);

   private:
    /**
    * Callback for receiving state info from server.
    */
    void
    serverStateInformation(const state_manager_msgs::RobotModeMsgConstPtr&);

   private:
    /**
    * The ROS Node Handle
    */
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    /**
    * The node name.
    */
    std::string node_name_;

    /**
    * Publisher to the state topic
    */
    ros::Publisher acknowledgePublisher_;
    /**
    * Subscriber to the state topic
    */
    ros::Subscriber stateSubscriber_;
    /**
    * Client for registering
    */
    ros::ServiceClient registerServiceClient_;
    /**
    * True to register to server.
    */
    bool registerToServer_;
  };

}  // namespace state_manager

#endif  // STATE_MANAGER_STATE_CLIENT_NODELET_H
