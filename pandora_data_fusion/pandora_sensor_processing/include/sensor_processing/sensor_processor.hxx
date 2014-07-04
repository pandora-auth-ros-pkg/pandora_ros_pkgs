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
 * Authors: 
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "pandora_sensor_processing/SensorProcessingConfig.h"

namespace pandora_sensor_processing
{

  template <class DerivedProcessor> 
    SensorProcessor<DerivedProcessor>::SensorProcessor(const std::string& ns,
        const std::string& sensorType)
    : nh_(ns), sensorType_(sensorType)
    {
      working_ = false;
      name_ = boost::to_upper_copy(ros::this_node::getName());

      // Get subscriber topic name.
      if (!nh_.getParam("subscribed_topic_names/raw_input", subscriberTopic_))
      {
        ROS_FATAL("[%s] %s raw input topic name param not found", 
            name_.c_str(), sensorType_.c_str());
        ROS_BREAK();
      }

      // Get publisher topic name and register a publisher.
      if (nh_.getParam("published_topic_names/alert_output", publisherTopic_))
      {
        alertPublisher_ = nh_.advertise<pandora_common_msgs::GeneralAlertMsg>
          (publisherTopic_, 5);
      }
      else
      {
        ROS_FATAL("[%s] %s alert output topic name param not found", 
            name_.c_str(), sensorType_.c_str());
        ROS_BREAK();
      }

      // Set callback for dynamic reconfiguration server.
      dynReconfServer_.setCallback(
          boost::bind(
            &DerivedProcessor::dynamicReconfigCallback, 
            static_cast<DerivedProcessor*>(this), _1, _2));

      // Get sensor's behaviour according to each robot state.
      if (!nh_.getParam("states/exploration", EXPLORATION_STATE))
      {
        ROS_FATAL("[%s] mode in exploration state param not found",
            name_.c_str());
        ROS_BREAK();
      }
      if (!nh_.getParam("states/identification", IDENTIFICATION_STATE))
      {
        ROS_FATAL("[%s] mode in identification state param not found",
            name_.c_str());
        ROS_BREAK();
      }
      if (!nh_.getParam("states/sensor_hold", SENSOR_HOLD_STATE))
      {
        ROS_FATAL("[%s] mode in sensor hold state param not found",
            name_.c_str());
        ROS_BREAK();
      }

      clientInitialize();
    }

  template <class DerivedProcessor>
    void SensorProcessor<DerivedProcessor>::completeTransition()
    {}

  template <class DerivedProcessor>
    void SensorProcessor<DerivedProcessor>::publishAlert()
    {
      ROS_DEBUG_NAMED("SENSOR_PROCESSING", "[%s] Publishing alert.", name_.c_str());
      alertPublisher_.publish(alert_);
    }

  template <class DerivedProcessor>
    void SensorProcessor<DerivedProcessor>::startTransition(int newState)
    {
      switch (newState)
      {
        case state_manager_communications::robotModeMsg::MODE_EXPLORATION_RESCUE:
          ROS_INFO("[%s] Entering Exploration mode.", name_.c_str());
          toggleSubscriber(EXPLORATION_STATE);
          break;
        case state_manager_communications::robotModeMsg::MODE_IDENTIFICATION:
          ROS_INFO("[%s] Entering Identification mode.", name_.c_str());
          toggleSubscriber(IDENTIFICATION_STATE);
          break;
        case state_manager_communications::robotModeMsg::MODE_SENSOR_HOLD:
          ROS_ERROR("[%s] Terminating node.", name_.c_str());
          toggleSubscriber(SENSOR_HOLD_STATE);
          break;
        case state_manager_communications::robotModeMsg::MODE_SEMI_AUTONOMOUS:
        case state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION:
        case state_manager_communications::robotModeMsg::MODE_EXPLORATION_MAPPING:
          toggleSubscriber(false);
          break;
        case state_manager_communications::robotModeMsg::MODE_SENSOR_TEST:
          toggleSubscriber(true);
          break;
        case state_manager_communications::robotModeMsg::MODE_TERMINATING:
          ROS_ERROR("[%s] Terminating node.", name_.c_str());
          exit(0);
          break;
        default:
          break;
      }

      transitionComplete(newState);
    }

  template <class DerivedProcessor>
    void SensorProcessor<DerivedProcessor>::toggleSubscriber(bool toWork)
    {
      if (toWork && !working_)
      {
        sensorSubscriber_ = nh_.subscribe(subscriberTopic_, 1, 
            &DerivedProcessor::sensorCallback, static_cast<DerivedProcessor*>(this));
        working_ = true;
      }
      else if (!toWork && working_)
      {
        sensorSubscriber_.shutdown();
        working_ = false;
      }
    }

}  // namespace pandora_sensor_processing

