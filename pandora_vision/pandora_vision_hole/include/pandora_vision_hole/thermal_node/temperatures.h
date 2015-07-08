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

#ifndef PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_H
#define PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_H

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Float32MultiArray.h>
#include "thermal_node/temperatures_parameters.h"
#include "utils/blob_detection.h"
#include "distrib_msgs/flirLeptonMsg.h"
#include "utils/message_conversions.h"


/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class Temperature
    @brief Provides functionalities for locating holes via
    analysis of a thermal image
   **/
  class Temperatures
  {
    private:
      // The Ros nodehandle
      ros::NodeHandle nodeHandle_;

      // Subscriber of thermal camera(flir-lepton) message
      ros::Subscriber thermalMsgSubscriber_;

      // The name of the topic where the thermal message is acquired from
      std::string thermalMsgTopic_;

      // The publisher for thermal points of interest
      ros::Publisher thermalPoiPublisher_;

      // The name of the topic where the thermal Pois are pulished to
      std::string thermalPoiTopic_;

      // Variables that help measure the processing time of the node
      double processTime_;
      double timeBefore_;
      double timeNow_;

      // The dynamic reconfigure (Temperatures) parameters server
      dynamic_reconfigure::Server<pandora_vision_hole::temperatures_cfgConfig>
        server_;

      // The dynamic reconfigure (Temperatures) parameters callback
      dynamic_reconfigure::Server<pandora_vision_hole::temperatures_cfgConfig>
        ::CallbackType f_;

      // The variables that define the temperature range that we search
      float lowTemperature;
      float highTemperature;

      /**
       @brief Acquires topic's names for subscription
       @param void
       @return void
       **/
      void getTopicNames();

      /**
       @brief Callback for the thermal message acquired from
       raspberry(flir-lepton). The message is unpacked in a cv::Mat image.
       Thermal points of interest regarding on a specific temperature range
       are located and information about them is later sent to other nodes.
       @param[in] msg [const std_msgs::Float32MultiArray&]
       The thermal message
       @return void
       **/
      void inputThermalMsgCallback(
        const distrib_msgs::flirLeptonMsg& msg);

      /**
       @brief The function called when a parameter is changed
       @param[in] config [const pandora_vision_hole::temperatures_cfgConfig&]
       @param[in] level [const uint32_t]
       @return void
       **/
      void parametersCallback(
        const pandora_vision_hole::temperatures_cfgConfig& config,
        const uint32_t& level);

    public:
      /**
        @brief Default constructor. Initiates communications, loads parameters.
        @return void
       **/
      Temperatures(void);

      /**
        @brief Default destructor
        @return void
       **/
      ~Temperatures(void);
  };


}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_H
