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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,Angelos Triantafyllidis
 *********************************************************************/

#ifndef PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_H
#define PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_H

#include "thermal_node/hole_detector.h"
#include "utils/parameters.h"
#include "utils/message_conversions.h"
#include "utils/image_matching.h"
#include "pandora_vision_hole/CandidateHolesVectorMsg.h"
#include "pandora_vision_msgs/IndexedThermal.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_vision_common/pandora_vision_interface/general_alert_converter.h"
#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_vision_msgs/ThermalAlert.h"
#include "pandora_vision_msgs/ThermalAlertVector.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_processor/ProcessorLogInfo.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class Thermal
    @brief Provides functionalities for locating holes via
    analysis of a thermal image
   **/
  class Thermal
  {
    private:
      // The ROS node handle
      ros::NodeHandle nodeHandle_;

      // Subscriber of thermal camera info
      ros::Subscriber thermalImageSubscriber_;

      // The name of the topic where the thermal image is acquired from
      std::string thermalImageTopic_;

      // Ros publisher for the candidate holes to hole-fusion.
      ros::Publisher candidateHolesPublisher_;

      // The name of the topic where the candidate holes that the thermal node
      // locates are published to. Publishes to hole-fusion and its independent
      // from the state.
      std::string candidateHolesTopic_;

      // Ros publisher for candidate holes POI directly to Data fusion.
      ros::Publisher dataFusionThermalPublisher_;

      // The name of the topic where the thermal node publishes
      // directly to Data fusion.
      std::string dataFusionThermalTopic_;

      // Ros publisher for candidate holes POI to thermal cropper node.
      ros::Publisher thermalToCropperPublisher_;

      // The name of the topic where the thermal node publishes
      // directly to thermal cropper node
      std::string thermalToCropperTopic_;

      // The publisher used to infor that thermal node has finished
      // processing the input data.
      ros::Publisher processEndPublisher_;

      // The name of the topic where the process end will be advertised
      std::string processEndTopic_;

      // The variables used to match the holeConveyor information to the
      // Rgb and Depth images.
      double xThermal_;
      double yThermal_;
      double cX_;
      double cY_;
      double angle_;

      // The dynamic reconfigure (thermal) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole::thermal_cfgConfig>
        server;

      // The dynamic reconfigure (thermal) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole::thermal_cfgConfig>
       ::CallbackType f;

      /**
        @brief Callback for the thermal image received by the camera.

        The thermal image message received by the camera is unpacked
        in a cv::Mat image.
        Holes are then located inside this image and information about them,
        along with the denoised image, is then sent to the hole fusion node
        @param msg [const pandora_vision_msgs::IndexedThermal&]
        The thermal image message
        @return void
       **/
      void inputThermalImageCallback(
        const pandora_vision_msgs::IndexedThermal& msg);

      /**
        @brief Acquires topics' names needed to be subscribed by the thermal node.
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole::thermal_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallback(
        const pandora_vision_hole::thermal_cfgConfig& config,
        const uint32_t& level);

      /**
        @brief This function finds for each point of interest found it's
        probability based on the keypoint's average temperature.
        @param[out] holes [const HolesConveyor&] The points of interest found
        @param[in] temperatures [const Float32MultiArray&] The multiArray with
        the temperatures of the image.
        @param[in] method [const int&] Denotes the probabilities extraction
        method.
        @return void
       **/
      void findHolesProbability(HolesConveyor* holes,
        const std_msgs::Float32MultiArray& temperatures, const int& method);

    public:
      /**
        @brief Default constructor. Initiates communications, loads parameters.
        @return void
       **/
      Thermal(void);

      /**
        @brief Default destructor
        @return void
       **/
      ~Thermal(void);
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_THERMAL_NODE_THERMAL_H
