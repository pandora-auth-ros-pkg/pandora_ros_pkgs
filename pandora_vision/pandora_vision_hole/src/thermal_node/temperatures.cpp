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

#include "thermal_node/temperatures.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  Temperatures::Temperatures(void)
    : timeNow_(0.0), timeBefore_(0.0), processTime_(0.0)
  {
    // Acquire the names of topics which the temperature node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the thermal message published by raspberry (flir-lepton)
    thermalMsgSubscriber_ = nodeHandle_.subscribe(thermalMsgTopic_, 1,
      &Temperatures::inputThermalMsgCallback, this);

    // Advertise the thermal pois found by temperature node
    //thermalPoiPublisher_ = nodeHandle_.advertise
      //<pandora_vision_msgs::CandidateHolesVectorMsg>(
      //thermalPoiTopic_, 1000);

    // The dynamic reconfigure (Temperatures) parameter's callback
    f_ = boost::bind(&Temperatures::parametersCallback, this, _1, _2);
    server_.setCallback(f_);

    ROS_INFO("[Temperatures node] Initiated");
  }



  /**
    @brief Default destructor
    @return void
   **/
  Temperatures::~Temperatures(void)
  {
    ROS_INFO("[Temperatures node] Terminated");
  }

  /**
    @brief Acquires topic's names for subscription
    @param void
    @return void
  **/
  void Temperatures::getTopicNames()
  {
    // The namespace of the node
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the temperature node acquires
    // the thermal message
    if(nodeHandle_.getParam(
      ns + "/thermal_temperatures_node/subscribed_topics/thermal_msg_topic",
      thermalMsgTopic_))
    {
      ROS_INFO("[Temperatures node] Subscribed to input thermal message");
    }
    else
    {
      ROS_ERROR(
        "[Temperatures node] Could not subscribe to thermal message topic");
    }

    // Read the name of the topic to which the temperatures node will be
    // publishing.
    if(nodeHandle_.getParam(
      ns + "/thermal_temperatures_node/published_topics/thermal_poi_topic",
      thermalPoiTopic_))
    {
      ROS_INFO("[Temperatures node] Advertising to the thermal poi topic");
    }
    else
    {
      ROS_ERROR("[Temperatures node] Could not find topic to advertise");
    }
  }


  /**
    @brief Callback for the thermal message acquired from
    raspberry(flir-lepton). The message is unpacked in a cv::Mat image.
    Thermal points of interest regarding on a specific temperature range
    are located and information about them is later sent to other nodes.
    @param msg [const std_msgs::Float32MultiArray&]
    The thermal message
    @return void
   **/
  void Temperatures::inputThermalMsgCallback(
    const distrib_msgs::flirLeptonMsg& msg)
  {
    // Debugging information
    ROS_INFO("=========================================================");

    ROS_INFO("Temperatures Callback called");
    timeNow_ = ros::Time::now().toSec();
    
    processTime_ = timeNow_ - timeBefore_;
    
    ROS_INFO_STREAM("[Temperatures node] Process time:" << processTime_);

    ROS_INFO("=========================================================");

    // The time when the new process starts
    timeBefore_ = ros::Time::now().toSec();

    // Obtain the thermal message and extract the temperature information.
    // Convert this information to cv::Mat in order to be processed.
    // It's format will be CV_8UC1
    cv::Mat temperatureImage = MessageConversions::convertFloat32MultiArrayToMat
      (msg.temperatures);

    // Apply double threshold(up and down) in the temperature image.
    // The threshold is set by configuration
    ///////////////////////////////////////////////////////////////////////////////////////////
    //cv::Mat thresholdImage;
    //cv::threshold(temperatureImage, thresholdImage, highTemperature, 255, cv::THRESH_TOZERO_INV);
    //cv::threshold(thresholdImage, thresholdImage, lowTemperature, 255, cv::THRESH_BINARY);

    //cv::resize(thresholdImage, thresholdImage, cvSize(640, 480));
    //cv::imshow("threshold",thresholdImage);
    //cv::waitKey(1);
    ///////////////////////////////////////////////////////////////////////////////////////////
    cv::inRange(
      temperatureImage, cv::Scalar(lowTemperature), cv::Scalar(highTemperature), temperatureImage); 

    cv::resize(temperatureImage, temperatureImage, cvSize(640, 480));
    cv::imshow("temperature",temperatureImage);
    cv::waitKey(1);

    // Find blobs in the thresholded image. Each blob is represented as
    // a keypoint which is the center of the blob found
    ////////////////////// change the parameters make thermal /////////////////
    //std::vector<cv::KeyPoint> keyPoints;
    //BlobDetection::detectBlobs(thresholdImage, &keyPoints);


  }

  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole::temperatures_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void Temperatures::parametersCallback(
    const pandora_vision_hole::temperatures_cfgConfig& config,
    const uint32_t& level)
  {
    ROS_INFO("[Temperatures node] Parameters callback called");

    // ----------------- Temperature threshold parameters ------------------- //
    lowTemperature = config.lowTemperature;
    highTemperature = config.highTemperature;
  }


} // namespace pandora_vision
