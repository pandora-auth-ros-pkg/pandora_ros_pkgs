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
* File Description: Definition of interface_tester. The class contains 
*            methods for testing publishers, subscribers, service
*            servers, action servers etc
* 
* Contents: checkForService, checkForTopicPublishing, checkForSubscribed,
*        checkForSubscribedNode, checkForNodePublishing, 
*        checkForNodeService, checkForNode, checkForMaster, 
*        checkForActionServer, checkForActionNodeServer,
*        checkForActionClient, checkForActionNodeClient, checkForTF
* 
* Author: Software Architecture Team
* 
*/

#ifndef INTERFACETESTER_H
#define INTERFACETESTER_H
#include "ros/ros.h"
#include <tf/transform_listener.h>

class InterfaceTester {
  private:
    
    //Constructor
    InterfaceTester() {};
  
  public:    
    /**
     * Tests if a service has been advertised.
     * @param serviceName the name of the service to test against
     * @return true if the service exists
     */
    static bool checkForService(std::string serviceName);
    
    /**
     * Test if a topic exisits.
     * @param topicName the name of the topic
     * @param topicType the type of the topic
     * @return true if topic exists
     */
    static bool checkForTopicPublishing(std::string topicName,std::string topicType);
    
    /**
     * Checks if a specific node is publishing at a node.
     * @param topicName the name of the topic
     * @param nodeName a string that is the beginning (or whole) of the node name
     * @return true if the node publishes on the given topic
     */
    static bool checkForNodePublishing(std::string topicName,std::string nodeName);
    
    /**
     * Checks if a specific node offers a specific service.
     * @param serviceName the name of the topic
     * @param nodeName a string that is the beginning (or whole) of the node name
     * @return true if the node publishes on the given topic
     */
    static bool checkForNodeService(std::string serviceName,std::string nodeName);
    
    /**
     * Checks if a node is running.
     * @param nodeName the name of the node
     * @return true if the node seems to be running
     */
    static bool checkForNode(std::string nodeName);
    
    /**
     * Checks for an action server.
     * @param actionName the action name
     * @param type the type of the action
     * @return true if action exists
     */
    static bool checkForActionServer(std::string actionName, std::string type);
    
    /**
     * Checks for an action server.
     * @param actionName the action name
     * @param nodeName the name of the node acting as an action server
     * @return true if action exists
     */
    static bool checkForActionNodeServer(std::string actionName, std::string nodeName);
    
    /**
     * Checks for an action client.
     * @param actionName the action name
     * @param type the action type
     * @return true if action exists
     */
    static bool checkForActionClient(std::string actionName, std::string type);
    
    /**
     * Checks for an node action client.
     * @param actionName the action name
     * @param nodeName the node acting as client
     * @return true if action exists
     */
    static bool checkForActionNodeClient(std::string actionName, std::string nodeName);
    
    /**
     * A simple wrapper for checking if master is running
     * @return true if master is running
     */
    static bool checkForMaster();
    
    /**
     * Checks if any message subscriber.
     * @param topic to check if any node is subscribed
     * @return true if subscribed succesfully
     */
    static bool checkForSubscribed(std::string topic);
    
    /**
     * Check if a specific node is subscribed at a specific topic.
     * @param topic the topic name
     * @param  a string that is the beginning (or whole) of the node name
     * @return true if the node is subscribed to the topic
     */
    static bool checkForSubscribedNode(std::string topic, std::string node);  
    
    /**
     * Check if a specific node is subscribed at a specific topic.
     * @param parent the frame's parent name
     * @param  child the frame's child name
     * @return true if the tranform is published on /tf
     */
    static bool checkForTF(std::string parent, std::string child);
};

#endif 
