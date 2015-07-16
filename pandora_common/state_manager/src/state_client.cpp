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

#include <string>
#include <boost/algorithm/string.hpp>

#include "state_manager/state_client.h"

namespace state_manager
{

  StateClient::
  StateClient(bool doRegister) : nh_(""), private_nh_("~")
  {
    node_name_ = boost::to_upper_copy<std::string>(private_nh_.getNamespace());

    acknowledgePublisher_ = nh_.advertise<state_manager_msgs::RobotModeMsg>
    ("/robot/state/server", 2, true);

    stateSubscriber_ = nh_.subscribe("/robot/state/clients", 2,
      &StateClient::serverStateInformation, this);

    if (doRegister)
      clientRegister();
  }

  StateClient::
  ~StateClient() {}

  ros::NodeHandle&
  StateClient::
  getPublicNodeHandle()
  {
    return nh_;
  }

  ros::NodeHandle&
  StateClient::
  getPrivateNodeHandle()
  {
    return private_nh_;
  }

  std::string
  StateClient::
  getName()
  {
    return node_name_;
  }

  void
  StateClient::
  clientRegister()
  {
    while (!ros::service::waitForService("/robot/state/register",
          ros::Duration(.1)) && ros::ok())
    {
      ROS_ERROR("[%s] Couldn't find service /robot/state/register", node_name_.c_str());
      ros::spinOnce();
    }
    registerServiceClient_ = nh_.
      serviceClient<state_manager_msgs::RegisterNodeSrv>("/robot/state/register");

    state_manager_msgs::RegisterNodeSrv rq;
    rq.request.nodeName = node_name_;
    rq.request.type = state_manager_msgs::RegisterNodeSrvRequest::TYPE_STARTED;

    while (!registerServiceClient_.call(rq) && ros::ok())
      ROS_ERROR("[%s] Failed to register node. Retrying...", node_name_.c_str());
  }

  void
  StateClient::
  clientInitialize()
  {
    while (!ros::service::waitForService("/robot/state/register",
          ros::Duration(.1)) && ros::ok())
    {
      ROS_ERROR("[%s] Couldn't find service /robot/state/register", node_name_.c_str());
      ros::spinOnce();
    }
    registerServiceClient_ = nh_.
      serviceClient<state_manager_msgs::RegisterNodeSrv>("/robot/state/register");

    state_manager_msgs::RegisterNodeSrv rq;
    rq.request.nodeName = node_name_;
    rq.request.type = state_manager_msgs::RegisterNodeSrvRequest::TYPE_INITIALIZED;

    while (!registerServiceClient_.call(rq) && ros::ok())
      ROS_ERROR("[%s] Failed to register node. Retrying...", node_name_.c_str());
  }

  void
  StateClient::
  transitionComplete(int newState)
  {
    ROS_INFO("[%s] transitioned to state %s",
        node_name_.c_str(), ROBOT_STATES(newState).c_str());

    state_manager_msgs::RobotModeMsg msg;
    msg.nodeName = node_name_;
    msg.mode = newState;
    msg.type = state_manager_msgs::RobotModeMsg::TYPE_ACK;
    acknowledgePublisher_.publish(msg);
  }

  void
  StateClient::
  transitionToState(int newState)
  {
    ROS_INFO("[%s] Requesting transition to state %i", node_name_.c_str(), newState);
    state_manager_msgs::RobotModeMsg msg;
    msg.nodeName = node_name_;
    msg.mode = newState;
    msg.type = state_manager_msgs::RobotModeMsg::TYPE_REQUEST;
    acknowledgePublisher_.publish(msg);
  }

  void
  StateClient::
  serverStateInformation(const state_manager_msgs::RobotModeMsgConstPtr& msg)
  {
    // ROS_INFO("[%s] Received new information from state server", node_name_.c_str());
    if (msg->type == state_manager_msgs::RobotModeMsg::TYPE_TRANSITION)
    {
      this->startTransition(msg->mode);
    }
    else if (msg->type == state_manager_msgs::RobotModeMsg::TYPE_START)
    {
      this->completeTransition();
    }
    else
    {
      ROS_ERROR("[%s] StateClient received a new state command, that is not understandable",
      node_name_.c_str());
    }
  }

}  // namespace state_manager
