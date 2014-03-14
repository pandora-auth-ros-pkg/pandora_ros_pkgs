/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Wim Meeussen */

#include "tf/tf.h"
#include "tf/transform_listener.h"
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "interface_tester/tf_monitor.h"

using namespace tf;
using namespace ros;
using namespace std;


namespace interface_tester{


TFMonitor::TFMonitor()
  {
    subscriber_tf_ = node_.subscribe<tf::tfMessage>(
      "tf", 100, boost::bind(&TFMonitor::callback, this, _1));
    subscriber_tf_message_ = node_.subscribe<tf::tfMessage>(
      "tf_message", 100, boost::bind(&TFMonitor::callback, this, _1));
    checkInactiveTimer = node_.createTimer(
      ros::Duration(0.1),&TFMonitor::checkInactiveTimerCallback,this);
  }

void TFMonitor::callback(const tf::tfMessageConstPtr& msg_ptr)
  {
    const tf::tfMessage& message = *msg_ptr;
    //Lookup the authority 
    std::string authority;
    std::map<std::string, std::string>* msg_header_map =
      message.__connection_header.get();
    std::map<std::string, std::string>::iterator it =
      msg_header_map->find("callerid");
    if (it == msg_header_map->end())
    {
      ROS_WARN("Message recieved without callerid");
      authority = "no callerid";
    }
    else
    {
      authority = it->second;
    }

    for (unsigned int i = 0; i < message.transforms.size(); i++)
    {
      frame_authority_map[message.transforms[i].child_frame_id] = authority;
      time_map[message.transforms[i].child_frame_id] = ros::Time::now();
    } 

  }

  void TFMonitor::outputNodesFrameInfo()
  {
    std::map<std::string, std::string>::iterator iter ;
    
    if (frame_authority_map.begin() == frame_authority_map.end() ) {
        ROS_INFO(" map is empty");
    }
            
    for(iter = frame_authority_map.begin(); iter != 
      frame_authority_map.end()  ; ++iter){
      ROS_INFO("Node %s publishing frame %s",
        iter->second.c_str() , iter->first.c_str());
    }
  }
  
  void TFMonitor::checkInactiveTimerCallback(const ros::TimerEvent&) {
        
    std::map<std::string, std::string>::iterator iter ;
    
    ros::Time now = ros::Time::now();
    
    iter= frame_authority_map.begin();
    
    while (iter != frame_authority_map.end()) {
      if ( now - time_map[iter->first]  > ros::Duration(0.3) ) {
        frame_authority_map.erase(iter++);
      } else {
        ++iter;
      }
    }
  
  }
  
  bool TFMonitor::checkForPublishedTF(std::string frame, std::string node){
    if (frame_authority_map.count(frame) == 0) {
      return false;
    }
    if (frame_authority_map[frame].compare(node) == 0) {
      return true;
    }
    return false; 
  }
  
}
  
  

