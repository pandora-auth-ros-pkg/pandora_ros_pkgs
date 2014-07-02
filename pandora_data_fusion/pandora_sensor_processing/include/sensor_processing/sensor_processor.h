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

#ifndef SENSOR_PROCESSING_SENSOR_PROCESSOR_H
#define SENSOR_PROCESSING_SENSOR_PROCESSOR_H

#ifndef BOOST_NO_DEFAULTED_FUNCTIONS
#define BOOST_NO_DEFAULTED_FUNCTIONS
#endif

#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include "state_manager/state_client.h"
#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "pandora_sensor_processing/SensorProcessingConfig.h"

namespace pandora_sensor_processing
{

  /**
   * @brief Abstraction layer that is responsible for communication over ROS
   * with other nodes and processor organization through StateClient.
   */
  template <class DerivedProcessor>
    class SensorProcessor 
    : public StateClient, private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         * @param sensorType [std::string const&] Name of sensor
         * @param toggle [bool] if subscribe topic is to be toggled
         * @param open [bool] if subscribe topic is to be opened by construction
         */
        SensorProcessor(const std::string& ns,
            const std::string& sensorType, bool toggle = true, bool open = true);

        void startTransition(int newState);

        void completeTransition() {}

        /**
         * @brief getter for general alert message alert_
         * @return pandora_common_msgs::GeneralAlertMsg alert_
         */
        pandora_common_msgs::GeneralAlertMsg getAlert() const
        {
          return alert_;
        }

      protected:
        /**
         * @brief Delegates to alertPublisher_.
         * @return void
         */
        void publishAlert();

      private:
        void toggleSubscriber();

      protected:
        pandora_common_msgs::GeneralAlertMsg alert_;

        std::string name_;

      private:
        ros::NodeHandle nh_;

        std::string sensorType_;

        ros::Publisher alertPublisher_;
        std::string publisherTopic_;

        ros::Subscriber sensorSubscriber_;
        std::string subscriberTopic_;
        bool toggle_;
        bool open_;
        bool opened_;

        dynamic_reconfigure::Server< SensorProcessingConfig >
          dynReconfServer_;
    };

  template <class DerivedProcessor> 
    SensorProcessor<DerivedProcessor>::SensorProcessor(const std::string& ns,
        const std::string& sensorType, bool toggle, bool open)
    : nh_(ns), sensorType_(sensorType), open_(open), toggle_(toggle), opened_(false)
    {
      name_ = boost::to_upper_copy(ros::this_node::getName());

      if (!nh_.getParam("subscribed_topic_names/" + sensorType_ + "_raw", subscriberTopic_))
      {
        ROS_FATAL("[%s] %s_raw topic name param not found.", 
            name_.c_str(), sensorType_.c_str());
        ROS_BREAK();
      }

      if (open_)
      {
        sensorSubscriber_ = nh_.subscribe(subscriberTopic_, 1, 
            &DerivedProcessor::sensorCallback, static_cast<DerivedProcessor*>(this));
        opened_ = true;
      }

      if (nh_.getParam("published_topic_names/" + sensorType_ + "_alert", publisherTopic_))
      {
        alertPublisher_ = nh_.advertise<pandora_common_msgs::GeneralAlertMsg>
          (publisherTopic_, 5);
      }
      else
      {
        ROS_FATAL("[%s] %s_alert topic name param not found.", 
            name_.c_str(), sensorType_.c_str());
        ROS_BREAK();
      }

      dynReconfServer_.setCallback(
          boost::bind(
            &DerivedProcessor::dynamicReconfigCallback, 
            static_cast<DerivedProcessor*>(this), _1, _2));

      clientInitialize();
    }

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
        case state_manager_communications::robotModeMsg::MODE_EXPLORATION:
          ROS_INFO("[%s] Entering Exploration mode.", name_.c_str());
          open_ = false;
          break;
        case state_manager_communications::robotModeMsg::MODE_IDENTIFICATION:
          ROS_INFO("[%s] Entering Identification mode.", name_.c_str());
          open_ = true;
          break;
        case state_manager_communications::robotModeMsg::MODE_TERMINATING:
          ROS_ERROR("[%s] Terminating node.", name_.c_str());
          exit(0);
          break;
        case state_manager_communications::robotModeMsg::MODE_OFF:
          ROS_INFO("[%s] Node is off.", name_.c_str());
          break;
        default:
          break;
      }

      if (toggle_)
        toggleSubscriber();
    }

  template <class DerivedProcessor>
    void SensorProcessor<DerivedProcessor>::toggleSubscriber()
    {
      if (open_ && !opened_)
      {
        sensorSubscriber_ = nh_.subscribe(subscriberTopic_, 1, 
            &DerivedProcessor::sensorCallback, static_cast<DerivedProcessor*>(this));
        opened_ = true;
      }
      else if (!open_ && opened_)
      {
        sensorSubscriber_.shutdown();
        opened_ = false;
      }
    }

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_SENSOR_PROCESSOR_H
