/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* File Description: Implemetation of interface_tester
* 
* Contents: checkForService, checkForTopicPublishing, checkForSubscribed,
*        checkForSubscribedNode, checkForNodePublishing, 
*        checkForNodeService, checkForNode, checkForMaster, 
*        checkForActionServer, checkForActionNodeServer,
*        checkForActionClient, checkForActionNodeClient
* 
* Author: Software Architecture Team
* 
*/

#include "interface_tester/interface_tester.h"

bool InterfaceTester::checkForService(std::string serviceName) {
  return ros::service::exists(serviceName,false);  
}

bool InterfaceTester::checkForTopicPublishing(
    std::string topicName,std::string topicType) {
  ros::master::V_TopicInfo topicList;
  ros::master::getTopics(topicList);
  
  bool found = false;
  for (unsigned int i = 0; i < topicList.size(); i++) {
    ros::master::TopicInfo currentInfo;
    currentInfo = topicList.at(i);
    
    if (currentInfo.name ==  topicName && currentInfo.datatype == topicType){
      found = true;
      break;
    }
  }
  
  return found;  
}

bool InterfaceTester::checkForSubscribed(std::string topic) {
  XmlRpc::XmlRpcValue args, result, payload;
  ros::NodeHandle nh;
  args[0] = "/";

  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    ROS_ERROR("Error: Could not connect to master or other problem");
    return false;
  }
  
  
  for (int j =0; j< payload[1].size();j++){
    if (topic == std::string(payload[1][j][0]))
      return true;
  }
    
  return false;
}

bool InterfaceTester::checkForSubscribedNode(
  std::string topic, std::string node) {
  
  XmlRpc::XmlRpcValue args, result, payload;
  ros::NodeHandle nh;
  args[0] = "/";

  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    ROS_ERROR("Error: Could not connect to master or other problem");
    return false;
  }
  
  for (int j =0; j< payload[1].size();j++){
    if (std::string(payload[1][j][0]).find(topic)==0) {
      for (int l=0; l< payload[1][j][1].size();l++){
        if(std::string(payload[1][j][1][l]).find(node)==0)
          return true;        
      }
    }
  }
    
  return false;
}

bool InterfaceTester::checkForNodePublishing(
    std::string topic,std::string node) {
  XmlRpc::XmlRpcValue args, result, payload;
  ros::NodeHandle nh;
  args[0] = "/";

  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    ROS_ERROR("Error: Could not connect to master or other problem");
    return false;
  }
  
  for (int j =0; j< payload[0].size();j++){
    if (std::string(payload[0][j][0])==topic) {
      for (int l=0; l< payload[0][j][1].size();l++){
        if(std::string(payload[0][j][1][l]).find(node)==0)
          return true;        
      }
    }
  }
    
  return false;
}

bool InterfaceTester::checkForNodeService(
    std::string serviceName,std::string node) {
  XmlRpc::XmlRpcValue args, result, payload;
  ros::NodeHandle nh;
  args[0] = "/";

  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    ROS_ERROR("Error: Could not connect to master or other problem");
    return false;
  }
  
  
  for (int j =0; j< payload[2].size();j++){
    if (std::string(payload[2][j][0])==serviceName) {        
      for (int l=0; l< payload[2][j][1].size();l++)
        if(std::string(payload[2][j][1][l]).find(node)==0)
          return true;        
    }
  }
    
  return false;
}


bool InterfaceTester::checkForNode(std::string nodeName) {
  
  ros::V_string nodes;
  ros::master::getNodes (nodes);
  
  bool found = false;
  for (unsigned int i = 0; i < nodes.size(); i++){
    if (nodes.at(i) == nodeName){
      found = true;
      break;
    }
  }
  
  return found;
}

bool InterfaceTester::checkForMaster() {
  return ros::master::check();
}

bool InterfaceTester::checkForActionServer(
    std::string actionName, std::string type){
  return checkForTopicPublishing(actionName+"/feedback",type+"ActionFeedback") 
    && checkForTopicPublishing(actionName+"/status",
                                      "actionlib_msgs/GoalStatusArray") 
    && checkForTopicPublishing(actionName+"/result",type+"ActionResult") 
    && checkForSubscribed(actionName+"/goal") 
    && checkForSubscribed(actionName+"/cancel");
}

bool InterfaceTester::checkForActionNodeServer(
    std::string actionName, std::string nodeName) {
  return checkForNodePublishing(actionName+"/feedback",nodeName) 
    && checkForNodePublishing(actionName+"/status",nodeName) 
    && checkForNodePublishing(actionName+"/result",nodeName) 
    && checkForSubscribedNode(actionName+"/goal",nodeName) 
    && checkForSubscribedNode(actionName+"/cancel",nodeName);
}

bool InterfaceTester::checkForActionClient(
    std::string actionName, std::string type) {
  return checkForTopicPublishing(actionName+"/goal",type+"ActionGoal") && 
    checkForTopicPublishing(actionName+"/cancel","actionlib_msgs/GoalID") &&
    checkForSubscribed(actionName+"/feedback") &&
    checkForSubscribed(actionName+"/status") &&
    checkForSubscribed(actionName+"/result");    
}

bool InterfaceTester::checkForActionNodeClient(
    std::string actionName, std::string nodeName) {
  return checkForNodePublishing(actionName+"/goal", nodeName) && 
    checkForNodePublishing(actionName+"/cancel", nodeName) &&
    checkForSubscribedNode(actionName+"/feedback", nodeName) &&
    checkForSubscribedNode(actionName+"/status", nodeName) &&
    checkForSubscribedNode(actionName+"/result", nodeName);  
  
}

bool InterfaceTester::checkForTF(std::string parent, std::string child) {
  tf::TransformListener _listener;
  return _listener.waitForTransform(
    parent, child, ros::Time(0), ros::Duration(1));
}
