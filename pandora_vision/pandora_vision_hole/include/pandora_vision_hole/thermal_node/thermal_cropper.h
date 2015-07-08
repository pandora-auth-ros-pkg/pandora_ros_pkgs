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

#ifndef PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_CROPPER_H
#define PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_CROPPER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Empty.h>
#include "state_manager/state_client_nodelet.h"

#include "sensor_msgs/Image.h"
#include "pandora_vision_msgs/EnhancedImage.h"
#include "sensor_processor/ProcessorLogInfo.h"
#include "pandora_vision_msgs/RegionOfInterest.h"

#include "thermal_node/utils/noise_elimination.h"
#include "thermal_node/utils/message_conversions.h"

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
    @class ThermalCropper
    @brief Is responsible to sent the point of interest detected from thermal
    analysis of image to victim node if it exists. Because victim needs enchanced
    message, it also aquires rgb and depth images. When a message is sent
    to victim node, an empty message is sent to synchronizer node to restart
    the whole thermal process.
   **/
  class ThermalCropper : public state_manager::StateClientNodelet
  {
   public:
    /**
      @brief Default constructor. Initiates communications, loads parameters.
      @return void
      **/
    ThermalCropper();

    /**
      @brief Default destructor
      @return void
      **/
    virtual
    ~ThermalCropper();

    virtual void
    onInit();

    /**
      @brief Callback for the thermal point of interest received
      by the thermal node.

      The thermal poi message received by the thermal node is unpacked.
      A counter is set. When this counter reaches 2 it means both rgb Depth and
      thermal poi message have been subscribed and are ready to be sent to victim.
      @param msg [const ::pandora_vision_hole::CandidateHolesVectorMsg&]
      The thermal image message
      @return void
      **/
    void
    inputThermalRoiCallback(const pandora_vision_msgs::EnhancedImageConstPtr& msg);

    /**
      @brief Callback for the synchronized rgb and depth images message
      received by synchronizer node.

      The message received by the synchronizer node is stored in private variable.
      A counter is set. When this counter reaches 2 it means both rgb Depth and
      thermal poi messages have been subscribed and are ready to be sent to victim.
      @param msg [const pandora_vision_msgs::EnhancedImageConstPtr&]
      The input synchronized rgb and depth images message
      @return void
      **/
    void
    inputEnhancedImageCallback(const pandora_vision_msgs::EnhancedImageConstPtr& msg);

   private:
    void
    process();

    /**
      @brief Acquires topics' names needed to be subscribed by the
      the thermal_cropper node.
      @param void
      @return void
      **/
    void
    getTopicNames();

    /**
      @brief Sends an empty message to dictate synchronizer node to unlock
      the thermal procedure.
      @param void
      @return void
      **/
    void
    unlockThermalProcedure();
    /**
      @brief The node's state manager.
      @param[in] newState [const int&] The robot's new state
      @return void
    **/
    void startTransition(int newState);

    /**
      @brief Completes the transition to a new state
      @param void
      @return void
    **/
    void completeTransition(void);


   private:
    //!< Node's distinct name
    std::string nodeName_;
    //!< The ROS node handle in general namespace
    ros::NodeHandle nh_;
    //!< The ROS node handle in private namespace
    ros::NodeHandle private_nh_;

    // Subscriber of thermal node result ( the exctracted point of interest)
    // and the thermal image
    ros::Subscriber thermalRoiSubscriber_;
    // The name of the topic where the thermal poi is acquired from
    std::string thermalRoiTopic_;
    pandora_vision_msgs::EnhancedImageConstPtr thermalEnhancedImageConstPtr_;
    bool isThermalAvailable_;

    // Subscriber of synchronizer node from where we acquire the synchronized
    // rgb and depth image
    ros::Subscriber enhancedImageSubscriber_;
    // The name of the topic where rgb and depth images are acquired from
    std::string enhancedImageTopic_;
    pandora_vision_msgs::EnhancedImageConstPtr enhancedImageConstPtr_;
    bool isEnhancedImageAvailable_;

    // Ros publisher for enchanced message directly to victim node.
    ros::Publisher victimThermalPublisher_;
    // The name of the topic where the thermal_cropper node publishes
    // directly to victim node.
    std::string victimThermalTopic_;

    // Ros Publisher to synchronizer.
    // Synchronizer dictates thermal standalone process to start.
    ros::Publisher unlockThermalProcedurePublisher_;
    // The name of the topic where the thermal cropper node publishes
    // to synchronizer node.
    std::string unlockThermalProcedureTopic_;

    // The publisher used to infor that thermal node has finished
    // processing the input data.
    ros::Publisher processEndPublisher_;
    // The name of the topic where the process end will be advertised
    std::string processEndTopic_;

    // The on/off state of the Hole Detector package
    bool isOn_;
    bool publishingEnhancedHoles_;
  };

}  // namespace thermal
}  // namespace pandora_vision_hole
}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_CROPPER_H
