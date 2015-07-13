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
 * Authors: Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include <string>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Empty.h>

#include "sensor_msgs/Image.h"
#include "pandora_vision_msgs/EnhancedImage.h"
#include "sensor_processor/ProcessorLogInfo.h"
#include "pandora_vision_msgs/RegionOfInterest.h"

#include "thermal_node/utils/message_conversions.h"
#include "thermal_node/utils/noise_elimination.h"
#include "thermal_node/thermal_cropper.h"

PLUGINLIB_EXPORT_CLASS(pandora_vision::pandora_vision_hole::thermal::ThermalCropper, nodelet::Nodelet)

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_hole
{
namespace thermal
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  ThermalCropper::
  ThermalCropper() {}

  /**
    @brief Default destructor
    @return void
   **/
  ThermalCropper::
  ~ThermalCropper(void)
  {
    NODELET_INFO("[%s] Terminated", nodeName_.c_str());
  }


  void
  ThermalCropper::
  onInit()
  {
    state_manager::StateClientNodelet::onInit();

    // Take NodeHandlers from nodelet manager
    nh_ = this->getPublicNh();
    private_nh_ = this->getPrivateNh();
    nodeName_ = this->getNodeName();

    // Acquire the names of topics which the thermal_cropper node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the thermal poi published by thermal node
    thermalRoiSubscriber_ = nh_.subscribe(thermalRoiTopic_, 1,
      &ThermalCropper::inputThermalRoiCallback, this);
    isThermalAvailable_ = false;
    // Subscribe to rgb and depth images published by synchronizer node.
    enhancedImageSubscriber_ = nh_.subscribe(enhancedImageTopic_, 1,
      &ThermalCropper::inputEnhancedImageCallback, this);
    isEnhancedImageAvailable_ = false;

    // Set the initial on/off state of the Hole Detector package to off
    isOn_ = false;
    publishingEnhancedHoles_ = false;

    // Advertise enhanced message to victim node
    victimThermalPublisher_ = nh_.advertise
      <pandora_vision_msgs::EnhancedImage>(victimThermalTopic_, 1);

    // Advertise empty message to synchronizer node so the thermal process
    // circle will start again
    unlockThermalProcedurePublisher_ = nh_.advertise
      <std_msgs::Empty>(unlockThermalProcedureTopic_, 1, true);

    // Advertise the topic where any external node(e.g. a functional test node)
    // will be subscribed to know that the hole node has finished processing
    // the current candidate holes as well as the result of the procedure.
    processEndPublisher_ = nh_.advertise
      <sensor_processor::ProcessorLogInfo>(processEndTopic_, 1, true);

    clientInitialize();

    NODELET_INFO("[%s] Initiated", nodeName_.c_str());
  }

  /**
    @brief Completes the transition to a new state
    @param void
    @return void
   **/
  void ThermalCropper::completeTransition(void)
  {
    NODELET_INFO("[%s] Transition Complete", nodeName_.c_str());
  }

  /**
    @brief Callback for the thermal point of interest received
    by the thermal node.

    The thermal poi message received by the thermal node is unpacked.
    A counter is set. When this counter reach 2 it means both rgb Depth and
    thermal poi message have been subscribed and are ready to be sent to victim.
    @param msg [const ::pandora_vision_hole::CandidateHolesVectorMsg&]
    The thermal image message
    @return void
   **/
  void
  ThermalCropper::
  inputThermalRoiCallback(const pandora_vision_msgs::EnhancedImageConstPtr& msg)
  {
    isThermalAvailable_ = true;
    thermalEnhancedImageConstPtr_ = msg;

    if (isEnhancedImageAvailable_ && isThermalAvailable_)
    {
      process();
    }
  }

  void
  ThermalCropper::
  inputEnhancedImageCallback(const pandora_vision_msgs::EnhancedImageConstPtr& msg)
  {
    isEnhancedImageAvailable_ = true;
    enhancedImageConstPtr_ = msg;

    if (isEnhancedImageAvailable_ && isThermalAvailable_)
    {
      process();
    }
  }

  void
  ThermalCropper::
  process()
  {
    if (!isEnhancedImageAvailable_ || !isThermalAvailable_)
    {
      NODELET_ERROR("[%s] Incorrect callback: process has not received enough info",
          nodeName_.c_str());
      return;
    }
    isEnhancedImageAvailable_ = false;
    isThermalAvailable_ = false;

    unlockThermalProcedure();

    NODELET_INFO("[%s] Processing thermal images", nodeName_.c_str());

    if (!publishingEnhancedHoles_)
      return;

    sensor_processor::ProcessorLogInfoPtr processorLogPtr( new sensor_processor::ProcessorLogInfo );

    if (thermalEnhancedImageConstPtr_->regionsOfInterest.size() == 0)
    {
      processorLogPtr->success = false;
      processorLogPtr->logInfo = "No ROIs";
      processEndPublisher_.publish(processorLogPtr);
      return;
    }

    pandora_vision_msgs::EnhancedImagePtr enhancedImagePtr(
        new pandora_vision_msgs::EnhancedImage );

    enhancedImagePtr->header = thermalEnhancedImageConstPtr_->header;
    enhancedImagePtr->thermalImage = thermalEnhancedImageConstPtr_->thermalImage;
    enhancedImagePtr->rgbImage = enhancedImageConstPtr_->rgbImage;
    enhancedImagePtr->depthImage = enhancedImageConstPtr_->depthImage;
    enhancedImagePtr->isDepth = enhancedImageConstPtr_->isDepth;
    enhancedImagePtr->regionsOfInterest = thermalEnhancedImageConstPtr_->regionsOfInterest;

    processorLogPtr->success = true;
    processorLogPtr->logInfo = "Finished";
    processEndPublisher_.publish(processorLogPtr);
    victimThermalPublisher_.publish(enhancedImagePtr);
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the thermal_cropper node
    @param void
    @return void
   **/
  void ThermalCropper::getTopicNames()
  {
    if (!private_nh_.getParam("subscribed_topics/thermal_roi_topic", thermalRoiTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal roi topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (!private_nh_.getParam("subscribed_topics/enhanced_image_topic", enhancedImageTopic_))
    {
      NODELET_FATAL("[%s] Could not find enhanced image topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (!private_nh_.getParam("published_topics/thermal_victim_node_topic", victimThermalTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal roi topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (!private_nh_.getParam("published_topics/thermal_unlock_synchronizer_topic", unlockThermalProcedureTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal roi topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (!private_nh_.getParam("published_topics/processor_log_topic", processEndTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal roi topic", nodeName_.c_str());
      ROS_BREAK();
    }
  }

  /**
    @brief The node's state manager.
    @param[in] newState [const int&] The robot's new state
    @return void
   **/
  void
  ThermalCropper::
  startTransition(int newState)
  {
    // The new on/off state of the Hole Detector package
    bool toBeOn = ((newState ==
          state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE)
      || (newState ==
          state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION)
      || (newState ==
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD)
      || (newState ==
        state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST));

    // off -> on
    if (!isOn_ && toBeOn)
    {
      // The on/off state of the Hole Detector Package is off, so the
      // synchronizer must be unlocked to start the thermal procedure
      isOn_ = toBeOn;
      unlockThermalProcedure();
    }
    // on -> off
    else if (isOn_ && !toBeOn)
    {
      isOn_ = toBeOn;
    }

    // Shutdown or open publisher of enhanced images
    if (newState == state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION
      || newState == state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD
      || newState == state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST)
    {
      if (!publishingEnhancedHoles_)
      {
        NODELET_INFO("[%s] Publishes now enhanced images from holes", nodeName_.c_str());
        publishingEnhancedHoles_ = true;
        victimThermalPublisher_ = nh_.advertise
          <pandora_vision_msgs::EnhancedImage>(victimThermalTopic_, 1);
      }
    }
    else
    {
      if (publishingEnhancedHoles_)
      {
        NODELET_INFO("[%s] Stop publishing now enhanced images from holes", nodeName_.c_str());
        publishingEnhancedHoles_ = false;
        victimThermalPublisher_.shutdown();
      }
    }

    transitionComplete(newState);
  }
  /**
    @brief Sends an empty message to dictate synchronizer node to unlock
    the thermal procedure.
    @param void
    @return void
   **/
  void
  ThermalCropper::
  unlockThermalProcedure()
  {
    // Send message to synchronizer in order for thermal procedure to start.
    if (isOn_)
    {
      std_msgs::EmptyPtr unlockThermalProcedure( new std_msgs::Empty );
      unlockThermalProcedurePublisher_.publish(unlockThermalProcedure);
    }
  }

}  // namespace thermal
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
