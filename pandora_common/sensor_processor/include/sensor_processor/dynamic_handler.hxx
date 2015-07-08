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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX
#define SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX

#include <string>
#include <vector>

#include <ros/ros.h>

#include "state_manager_msgs/RobotModeMsg.h"

#include "sensor_processor/abstract_processor.h"
#include "sensor_processor/dynamic_handler.h"

namespace sensor_processor
{

  DynamicHandler::
  DynamicHandler() {}

  DynamicHandler::
  ~DynamicHandler() {}

  void
  DynamicHandler::
  onInit()
  {
    Handler::onInit();

    processor_loader_ptr_.reset( new pluginlib::
        ClassLoader<AbstractProcessor>("sensor_processor", "sensor_processor::AbstractProcessor") );

    currentState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;
    previousState_ = state_manager_msgs::RobotModeMsg::MODE_OFF;

    int ii;

    activeStates_.clear();
    XmlRpc::XmlRpcValue active_states;
    if (!private_nh_.getParam("active_states", active_states))
    {
      ROS_FATAL("[%s] Cound not find active robot states", name_.c_str());
      ROS_BREAK();
    }
    ROS_ASSERT(active_states.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (ii = 0; ii < active_states.size(); ++ii) {
      ROS_ASSERT(active_states[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
      activeStates_.push_back(static_cast<std::string>(active_states[ii]));
    }

    bool load;
    private_nh_.param("load_processors", load, false);

    if (load)
    {
      int jj;

      for (ii = 0; ii < activeStates_.size(); ++ii) {
        XmlRpc::XmlRpcValue state_processors;
        if (!private_nh_.getParam("state_processors/"+activeStates_[ii], state_processors))
        {
          ROS_FATAL("[%s] Cound not find processor per state", name_.c_str());
          ROS_BREAK();
        }
        ROS_ASSERT(state_processors.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(state_processors.size() == 3);
        boost::array<std::string, 3> processors_of_state;
        for (jj = 0; jj < 3; ++jj) {
          ROS_ASSERT(state_processors[jj].getType() == XmlRpc::XmlRpcValue::TypeString);
          processors_of_state[jj] = static_cast<std::string>(state_processors[jj]);
        }
        state_to_processor_map_.insert(std::make_pair(
              ROBOT_STATES(activeStates_[ii]), processors_of_state));
      }
    }
  }

  template <class PreProcessor>
  void
  DynamicHandler::
  loadPreProcessor(const std::string& processor_name)
  {
    this->preProcPtr_.reset( new PreProcessor );
    this->preProcPtr_->initialize(processor_name, this);
  }

  void
  DynamicHandler::
  loadPreProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    previousPreProcessorType_ = processor_type;
    loadProcessor(this->preProcPtr_, processor_name, processor_type);
  }

  void
  DynamicHandler::
  checkAndLoadPreProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    if (previousPreProcessorType_ == processor_type)
      return;
    loadPreProcessor(processor_name, processor_type);
  }

  void
  DynamicHandler::
  unloadPreProcessor()
  {
    this->preProcPtr_.reset();
  }

  template <class Processor>
  void
  DynamicHandler::
  loadProcessor(const std::string& processor_name)
  {
    this->processorPtr_.reset( new Processor );
    this->processorPtr_->initialize(processor_name, this);
  }

  void
  DynamicHandler::
  loadProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    previousProcessorType_ = processor_type;
    loadProcessor(this->processorPtr_, processor_name, processor_type);
  }

  void
  DynamicHandler::
  checkAndLoadProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    if (previousProcessorType_ == processor_type)
      return;
    loadProcessor(processor_name, processor_type);
  }

  void
  DynamicHandler::
  unloadProcessor()
  {
    this->processorPtr_.reset();
  }

  template <class PostProcessor>
  void
  DynamicHandler::
  loadPostProcessor(const std::string& processor_name)
  {
    this->postProcPtr_.reset( new PostProcessor );
    this->postProcPtr_->initialize(processor_name, this);
  }

  void
  DynamicHandler::
  loadPostProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    previousPostProcessorType_ = processor_type;
    loadProcessor(this->postProcPtr_, processor_name, processor_type);
  }

  void
  DynamicHandler::
  checkAndLoadPostProcessor(const std::string& processor_name, const std::string& processor_type)
  {
    if (previousPostProcessorType_ == processor_type)
      return;
    loadPostProcessor(processor_name, processor_type);
  }

  void
  DynamicHandler::
  unloadPostProcessor()
  {
    this->postProcPtr_.reset();
  }

  void
  DynamicHandler::
  startTransition(int newState)
  {
    this->previousState_ = this->currentState_;
    this->currentState_ = newState;

    bool previouslyOff = true;
    bool currentlyOn = false;

    for (int ii = 0; ii < activeStates_.size(); ii++) {
      previouslyOff = (previouslyOff && this->previousState_ != ROBOT_STATES(activeStates_[ii]));
      currentlyOn = (currentlyOn || this->currentState_ == ROBOT_STATES(activeStates_[ii]));
    }

    if (!previouslyOff && !currentlyOn)
    {
      unloadPreProcessor();
      previousPreProcessorType_ = "";
      unloadProcessor();
      previousProcessorType_ = "";
      unloadPostProcessor();
      previousPostProcessorType_ = "";
    }
    else if (previouslyOff && currentlyOn)
    {
      loadPreProcessor("~preprocessor", state_to_processor_map_[this->currentState_][0]);
      loadProcessor("~processor", state_to_processor_map_[this->currentState_][1]);
      loadPostProcessor("~postprocessor", state_to_processor_map_[this->currentState_][2]);
    }
    else
    {
      checkAndLoadPreProcessor("~preprocessor", state_to_processor_map_[this->currentState_][0]);
      checkAndLoadProcessor("~processor", state_to_processor_map_[this->currentState_][1]);
      checkAndLoadPostProcessor("~postprocessor", state_to_processor_map_[this->currentState_][2]);
    }

    if (this->currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      unloadPreProcessor();
      unloadProcessor();
      unloadPostProcessor();

      ROS_INFO("[%s] Terminating", name_.c_str());
      ros::shutdown();
      return;
    }

    transitionComplete(this->currentState_);
  }

  void
  DynamicHandler::
  completeTransition() {}

  void
  DynamicHandler::
  loadProcessor(AbstractProcessorPtr& processorPtr,
                const std::string& processor_name, const std::string& processor_type)
  {
    try
    {
      processorPtr = processor_loader_ptr_->createInstance(processor_type);
      processorPtr->initialize(processor_name, this);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("[%s] Failed to create a %s processor, are you sure it is properly"
                " registered and that the containing library is built? "
                "Exception: %s", name_.c_str(), processor_type.c_str(), ex.what());
      ROS_BREAK();
    }
  }

}  // namespace sensor_processor

#endif  // SENSOR_PROCESSOR_DYNAMIC_HANDLER_HXX
