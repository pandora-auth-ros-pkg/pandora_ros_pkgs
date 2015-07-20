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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Angelos Triantafyllidis
 *********************************************************************/

#include <string>
#include <limits>
#include <boost/algorithm/string.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include "distrib_msgs/FlirLeptonMsg.h"
#include "pandora_vision_common/pois_stamped.h"
#include "pandora_common_msgs/GeneralAlertVector.h"
#include "pandora_vision_msgs/ThermalAlert.h"
#include "pandora_vision_msgs/ThermalAlertVector.h"
#include "pandora_vision_msgs/EnhancedImage.h"
#include "sensor_processor/ProcessorLogInfo.h"
#include "pandora_vision_common/pandora_vision_interface/vision_exceptions.h"

#include "pandora_vision_hole/CandidateHolesVectorMsg.h"
#include "pandora_vision_hole/CandidateHoleMsg.h"
#include "thermal_node/utils/parameters.h"
#include "thermal_node/utils/message_conversions.h"
#include "thermal_node/utils/image_matching.h"
#include "thermal_node/thermal.h"

PLUGINLIB_EXPORT_CLASS(pandora_vision::pandora_vision_hole::thermal::Thermal, nodelet::Nodelet)

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
  Thermal::
  Thermal() {}

  /**
    @brief Default destructor
    @return void
   **/
  Thermal::
  ~Thermal()
  {
    NODELET_INFO("[%s] Terminated", nodeName_.c_str());
  }

  void
  Thermal::
  onInit()
  {
    // Take NodeHandlers from nodelet manager
    nh_ = this->getNodeHandle();
    private_nh_ = this->getPrivateNodeHandle();
    nodeName_ = boost::to_upper_copy<std::string>(this->getName());

    nh_.param("thermal_mode", thermalMode_, true);
    nh_.param("thermal_alone_mode", thermal_alone_, true);
    nh_.param("rgbdt_mode", rgbdtMode_, false);
    private_nh_.param<std::string>("converted_frame", converted_frame_, "/kinect_rgb_optical_frame");

    if (thermal_alone_)
      thermalMode_ = true;

    // Acquire the names of topics which the thermal node will be having
    // transactionary affairs with
    getTopicNames();

    // Get the values of the variables used to match the final holeConveyors
    /* ImageMatching::variableSetUp(private_nh_, &xThermal_, &yThermal_, */
      /* &cX_, &cY_, &angle_); */

    isImageAvailable_ = false;
    // Subscribe to the thermal image published by raspberry
    thermalImageSubscriber_ = nh_.subscribe(thermalImageTopic_, 1,
      &Thermal::inputThermalImageCallback, this);

    if (!thermal_alone_)
    {
      isReceiverInfoAvailable_ = false;
      // Subscribe to the thermal image published by raspberry
      thermalOutputReceiverSubscriber_ = nh_.subscribe(thermalOutputReceiverTopic_, 1,
        &Thermal::inputThermalOutputReceiverCallback, this);

      if (rgbdtMode_)
      {
        // Advertise the candidate holes found by the thermal node to hole fusion
        candidateHolesPublisher_ = nh_.advertise
          < ::pandora_vision_hole::CandidateHolesVectorMsg >(candidateHolesTopic_, 1);
      }

      if (thermalMode_)
      {
        // Advertise the candidate holes found by the thermal
        // node to thermal cropper
        thermalToCropperPublisher_ = nh_.advertise
          <pandora_vision_msgs::EnhancedImage>(thermalToCropperTopic_, 1);
      }
    }

    // Advertise the candidate holes found by the thermal node to hole fusion
    dataFusionThermalPublisher_ = nh_.advertise
      <pandora_vision_msgs::ThermalAlertVector>(dataFusionThermalTopic_, 1);

    // Advertise the topic where any external node(e.g. a functional test node)
    // will be subscribed to know that the hole node has finished processing
    // the current candidate holes as well as the result of the procedure.
    processEndPublisher_ = nh_.advertise<sensor_processor::ProcessorLogInfo>(
        processEndTopic_, 1, true);

    // The dynamic reconfigure (thermal) parameter's callback
    thermalReconfServerPtr_.reset( new dynamic_reconfigure::Server<
        ::pandora_vision_hole::thermal_cfgConfig >(private_nh_) );
    thermalReconfServerPtr_->setCallback(boost::bind(&Thermal::parametersCallback, this, _1, _2));

    std::string modes;
    if (rgbdtMode_)
      modes += "rgbdt ";
    if (thermalMode_)
      modes += "thermal ";
    if (thermal_alone_)
      modes += "thermal_alone!";
    NODELET_INFO("[%s] Initiated %s", nodeName_.c_str(), modes.c_str());
  }

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
  void
  Thermal::
  inputThermalImageCallback(const distrib_msgs::FlirLeptonMsgConstPtr& msg)
  {
    isImageAvailable_ = true;
    imageConstPtr_ = msg;

    if (thermal_alone_)
    {
      isReceiverInfoAvailable_ = true;
      std_msgs::StringPtr infoPtr( new std_msgs::String );
      infoPtr->data = "thermal";
      receiverInfoConstPtr_ = infoPtr;
    }

    if (isImageAvailable_ && isReceiverInfoAvailable_)
      process();
  }

  void
  Thermal::
  inputThermalOutputReceiverCallback(const std_msgs::StringConstPtr& msg)
  {
    isReceiverInfoAvailable_ = true;
    receiverInfoConstPtr_ = msg;

    if (isImageAvailable_ && isReceiverInfoAvailable_)
      process();
  }

  void
  Thermal::
  process()
  {
    if (!isImageAvailable_ || !isReceiverInfoAvailable_)
    {
      NODELET_ERROR("[%s] Incorrect callback: process has not received enough info",
          nodeName_.c_str());
      return;
    }
    isImageAvailable_ = false;
    isReceiverInfoAvailable_ = false;

    NODELET_INFO("[%s] callback", nodeName_.c_str());

#ifdef DEBUG_TIME
    Timer::start("inputThermalImageCallback", "", true);
#endif

    //  Obtain the thermal message and extract the temperature information.
    //  Convert this
    //  information to cv::Mat in order to be processed.
    //  It's format will be CV_8UC1
    cv::Mat thermalImage = MessageConversions::convertFloat32MultiArrayToMat(
        imageConstPtr_->temperatures);
    // Apply double threshold(up and down) in the temperature image.
    // The threshold is set by configuration
    cv::inRange(
      thermalImage, cv::Scalar(Parameters::Thermal::low_temperature),
      cv::Scalar(Parameters::Thermal::high_temperature), thermalImage);
    // Obtain the thermal image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be CV_8UC1.
    cv::Mat thermalSensorImage;
    MessageConversions::extractImageFromMessage(imageConstPtr_->thermalImage,
      &thermalSensorImage, sensor_msgs::image_encodings::TYPE_8UC1);

#ifdef DEBUG_SHOW
    if (Parameters::Debug::show_thermal_image)
    {
      Visualization::showScaled("Thermal image", thermalImage, 1);
    }
#endif
    HolesConveyor holes;
    // Detection method = 0 --> process the binary image acquired from temperatures
    // Detection method = 1 --> process the sensor image acuired from sensor
    switch (Parameters::Thermal::detection_method)
    {
      case 0:
        // Locate potential holes in the thermal image
        holes = ThermalHoleDetector::findHoles(thermalImage);
        break;
      case 1:
        // Locate potential holes in the thermal image
        holes = ThermalHoleDetector::findHoles(thermalSensorImage);
        break;
      default:
        NODELET_ERROR("[%s] Incorrect callback: detection method param is %d",
            nodeName_.c_str(), Parameters::Thermal::detection_method);
        return;
    }

    // Find the average temperature of each point of interest that we found
    // and their probability.
    findHolesProbability(
      &holes, imageConstPtr_->temperatures, Parameters::Thermal::probability_method);

    if (holes.size() != 0)
      NODELET_WARN("[%s] Thermal Alert found!!!", nodeName_.c_str());


    // TODO(@anyone): Delegate to Frame Matcher
    // Convert the conveyors information so it can match with the
    //  Rgb and Depth images. If its outside of limits discart that conveyor.
    // ImageMatching::conveyorMatching(&holes, Parameters::Thermal::xThermal,
    //                                 Parameters::Thermal::yThermal,
    //                                 Parameters::Thermal::c_x,
    //                                 Parameters::Thermal::c_y,
    //                                 Parameters::Thermal::angle * CV_PI / 180);
    // Resize the thermal image to match the rgb-d images, of course the thermal
    // image will not be further processed
    // cv::resize(thermalSensorImage,
    //     thermalSensorImage, cvSize(Parameters::Image::WIDTH, Parameters::Image::HEIGHT));

    // Create the candidate holes message
    ::pandora_vision_hole::CandidateHolesVectorMsgPtr thermalCandidateHolesMsg(
        new ::pandora_vision_hole::CandidateHolesVectorMsg );

    // Pack information about holes found and the thermal image
    // inside a message.
    // This message will be published to and received by the hole fusion
    // or thermal cropper node based on the index of thermal message acquired
    // from the synchronizer node
    MessageConversions::createCandidateHolesVectorMessage(holes,
      thermalSensorImage,
      thermalCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_8UC1,
      imageConstPtr_->thermalImage);

    if (!thermal_alone_)
    {
      // Check the index and send the message to its proper receiver
      // The index takes only three values from synchronized node
      // "thermal", "hole" or "thermalhole".
      if (receiverInfoConstPtr_->data == "thermal")
      {
        if (thermalMode_)
          // Publish the candidate holes message to thermal cropper node
          publishToThermalCropper(thermalCandidateHolesMsg);
        else
          NODELET_ERROR("[%s] ReceiverInfo is 'thermal' while 'thermal' mode is not on!",
              nodeName_.c_str());
      }
      else if (receiverInfoConstPtr_->data == "hole")
      {
        if (rgbdtMode_)
          // Publish the candidate holes message to hole fusion node
          candidateHolesPublisher_.publish(thermalCandidateHolesMsg);
        else
          NODELET_ERROR("[%s] ReceiverInfo is 'hole' while 'rgbdt' mode is not on!",
              nodeName_.c_str());
      }
      else if (receiverInfoConstPtr_->data == "thermalhole")
      {
        if (thermalMode_)
          // Publish to both thermal cropper and hole fusion nodes
          publishToThermalCropper(thermalCandidateHolesMsg);
        else
          NODELET_ERROR("[%s] ReceiverInfo is 'thermalhole' while 'thermal' mode is not on!",
              nodeName_.c_str());
        if (rgbdtMode_)
          candidateHolesPublisher_.publish(thermalCandidateHolesMsg);
        else
          NODELET_ERROR("[%s] ReceiverInfo is 'thermalhole' while 'rgbdt' mode is not on!",
              nodeName_.c_str());
      }
      else
      {
        NODELET_ERROR("[%s] Incorrect callback: receiverInfo is %s", nodeName_.c_str(),
            receiverInfoConstPtr_->data.c_str());
        return;
      }
    }

    // Used for functional test
    sensor_processor::ProcessorLogInfoPtr resultMsg( new sensor_processor::ProcessorLogInfo );
    resultMsg->success = (holes.size() > 0);
    processEndPublisher_.publish(resultMsg);

    if (!resultMsg->success)
    {
      NODELET_WARN("[%s] Did not find any thermal alerts", nodeName_.c_str());
      return;
    }

    // Finally find the yaw and pitch of each candidate hole found and
    // send it to data fusion if a hole exists. The message to be sent is
    // ThermalAlertsVectorMsg type.
    // Fill the thermal message to be sent
    POIsStamped poisStamped;

    poisStamped.header = imageConstPtr_->header;
    poisStamped.frameWidth = thermalSensorImage.cols;
    poisStamped.frameHeight = thermalSensorImage.rows;

    // For each hole found by thermal node
    std::vector<HoleConveyor>::iterator iter = holes.holes.begin();

    while (iter != holes.holes.end())
    {
      // dynamic parameter of minimum probability
      if (iter->holeProbability < Parameters::Thermal::min_thermal_probability)
      {
        iter = holes.holes.erase(iter);
        continue;
      }

      POIPtr poi(new POI);

      // Set the keypoint
      poi->point.x = iter->keypoint.pt.x;
      poi->point.y = iter->keypoint.pt.y;

      // Fill the probabilities
      poi->probability = iter->holeProbability;

      poisStamped.pois.push_back(poi);

      ++iter;
    }

    pandora_vision_msgs::ThermalAlertVectorPtr thermalAlertVector( new pandora_vision_msgs::ThermalAlertVector );
    if (poisStamped.pois.size() > 0)
    {
      pandora_common_msgs::GeneralAlertVector alertVector;
      try
      {
        alertVector = alertConverter_.getGeneralAlertVector(nh_, poisStamped);
      }
      catch (const vision_config_error& ex)
      {
        NODELET_ERROR("[%s] Error in converting POIs to alerts: %s",
            nodeName_.c_str(), ex.what());
        return;
      }

      // Fill the thermal message header to be sent
      thermalAlertVector->header = alertVector.header;

      for (unsigned int ii = 0; ii < alertVector.alerts.size(); ++ii)
      {
        pandora_vision_msgs::ThermalAlert thermalAlert;
        // Fill the temperature of the thermal message for each hole
        thermalAlert.temperature = holes.holes[ii].holeTemperature;
        thermalAlert.info = alertVector.alerts.at(ii);

        // Push back into vector of messages
        thermalAlertVector->alerts.push_back(thermalAlert);
      }

      // Publish the thermal message
      dataFusionThermalPublisher_.publish(thermalAlertVector);
    }
    else
    {
      NODELET_WARN("[%s] Did not find thermal with high enough probability",
          nodeName_.c_str());
    }

#ifdef DEBUG_TIME
    Timer::tick("inputThermalImageCallback");
    Timer::printAllMeansTree();
#endif
  }

  void
  Thermal::
  publishToThermalCropper(const ::pandora_vision_hole::
      CandidateHolesVectorMsgConstPtr& candidateHolesConstPtr)
  {
    pandora_vision_msgs::EnhancedImagePtr enhancedThermalImagePtr(
        new pandora_vision_msgs::EnhancedImage );
    convertCandidateHolesToEnhancedImage(candidateHolesConstPtr, enhancedThermalImagePtr);
    thermalToCropperPublisher_.publish(enhancedThermalImagePtr);
  }

  void
  Thermal::
  convertCandidateHolesToEnhancedImage(
      const ::pandora_vision_hole::CandidateHolesVectorMsgConstPtr& candidateHolesConstPtr,
      const pandora_vision_msgs::EnhancedImagePtr& enhancedThermalImagePtr)
  {
    enhancedThermalImagePtr->header = candidateHolesConstPtr->header;
    enhancedThermalImagePtr->thermalImage = candidateHolesConstPtr->image;
    enhancedThermalImagePtr->isDepth = false;
    for (int ii = 0; ii < candidateHolesConstPtr->candidateHoles.size(); ++ii) {
      ::pandora_vision_hole::CandidateHoleMsg hole;
      hole = candidateHolesConstPtr->candidateHoles[ii];
      pandora_vision_msgs::RegionOfInterest roi;

      roi.center.x = hole.keypointX;
      roi.center.y = hole.keypointY;

      int minx, maxx, miny, maxy;
      minx = miny = std::numeric_limits<int>::max();
      maxx = maxy = std::numeric_limits<int>::min();
      for (int jj = 0; jj < hole.verticesX.size(); ++jj) {
        minx = hole.verticesX[jj] < minx ? hole.verticesX[jj] : minx;
        maxx = hole.verticesX[jj] > maxx ? hole.verticesX[jj] : maxx;
      }
      for (int jj = 0; jj < hole.verticesY.size(); ++jj) {
        miny = hole.verticesY[jj] < miny ? hole.verticesY[jj] : miny;
        maxy = hole.verticesY[jj] > maxy ? hole.verticesY[jj] : maxy;
      }
      roi.width = maxx - minx;
      roi.height = maxy - miny;

      enhancedThermalImagePtr->regionsOfInterest.push_back(roi);
    }
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the thermal node
    @param void
    @return void
   **/
  void
  Thermal::
  getTopicNames()
  {
    if (!private_nh_.getParam("subscribed_topics/thermal_image_topic", thermalImageTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal image topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (!private_nh_.getParam("subscribed_topics/thermal_output_receiver_topic", thermalOutputReceiverTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal output receiver topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (rgbdtMode_)
    {
      if (!private_nh_.getParam("published_topics/candidate_holes_topic", candidateHolesTopic_))
      {
        NODELET_FATAL("[%s] Could not find candidate holes topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }
    if (!private_nh_.getParam("published_topics/thermal_data_fusion_topic", dataFusionThermalTopic_))
    {
      NODELET_FATAL("[%s] Could not find thermal data fusion topic", nodeName_.c_str());
      ROS_BREAK();
    }
    if (thermalMode_)
    {
      if (!private_nh_.getParam("published_topics/thermal_to_cropper_topic", thermalToCropperTopic_))
      {
        NODELET_FATAL("[%s] Could not find thermal to cropper topic", nodeName_.c_str());
        ROS_BREAK();
      }
    }
    if (!private_nh_.getParam("published_topics/processor_log_topic", processEndTopic_))
    {
      NODELET_FATAL("[%s] Could not find processor log topic", nodeName_.c_str());
      ROS_BREAK();
    }
  }

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
  void
  Thermal::
  findHolesProbability(HolesConveyor* holes,
      const std_msgs::Float32MultiArray& temperatures, int method)
  {
    // The width and height of the input temperature multiarray
    int width = temperatures.layout.dim[1].size;
    int height = temperatures.layout.dim[0].size;

    // For each hole find its probability
    for (unsigned int i = 0; i < holes->size(); i++)
    {
      float average = 0;

      // Find the keypoint coordinates
      float temperatureX = holes->holes[i].keypoint.pt.x;
      float temperatureY = holes->holes[i].keypoint.pt.y;

      // Convert them to int, in order to access the point in MultiArray
      int tempX = static_cast <int> (std::floor(temperatureX));
      int tempY = static_cast <int> (std::floor(temperatureY));

      // Find the average temperature around the keypoint

      // Check if the keypoint is on the edges of the image
      if (tempX > 0 && tempX < 60 && tempY > 0 && tempY < 80)
      {
        int counter = 0;

        for (unsigned int k = (tempY - 1); k < (tempY + 2); k++)
        {
          for (unsigned int o = (tempX - 1); o < (tempX + 2); o++)
          {
            average += temperatures.data[k * width + o];
            counter++;
          }
        }
        average = average / counter;
      }
      else
      {
        // If it is on the edges it take the temperature of the keypoint it self
        average = temperatures.data[tempY * width + tempX];
      }

      // Fill the average temperature vector
      holes->holes[i].holeTemperature = average;

      // Apply gaussian function on the average temperature found
      if (method == 0)
      {
        float probability = exp(
          - pow((average - Parameters::Thermal::optimal_temperature), 2)
          / (2 * pow(Parameters::Thermal::tolerance , 2)));

        // Push the probability in the vector
        holes->holes[i].holeProbability = probability;
      }
      // Apply logistic function on the average temperature found
      else if (method == 1)
      {
        float probability = 1/(1 + exp(-Parameters::Thermal::left_tolerance *
            (average - Parameters::Thermal::low_acceptable_temperature)))
            - 1/(1 + exp(- Parameters::Thermal::right_tolerance *
            (average - Parameters::Thermal::high_acceptable_temperature)));

        // Push the probability in the vector
        holes->holes[i].holeProbability = probability;
      }
    }
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole::thermal_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void
  Thermal::
  parametersCallback(
      const ::pandora_vision_hole::thermal_cfgConfig& config,
      uint32_t level)
  {
    NODELET_INFO("[%s] Parameters callback called", nodeName_.c_str());

    ///////////////////// The thermal detection method ////////////////////////
    // If set to 0 process the binary image acquired from temperatures MultiArray
    // If set to 1 process the sensor/Image from thermal sensor
    Parameters::Thermal::detection_method = config.detection_method;

    //////////////////// Blob detection - specific parameters //////////////////

    Parameters::Blob::min_threshold =
      config.min_threshold;
    Parameters::Blob::max_threshold =
      config.max_threshold;
    Parameters::Blob::threshold_step =
      config.threshold_step;

    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Blob::min_area =
        config.min_area;
      Parameters::Blob::max_area =
        config.max_area;
    }
    // In wavelet mode, the image shrinks by a factor of 4
    else if (Parameters::Image::image_representation_method == 1)
    {
      Parameters::Blob::min_area =
        static_cast<int>(config.min_area / 4);
      Parameters::Blob::max_area =
        static_cast<int>(config.max_area / 4);
    }

    Parameters::Blob::min_convexity =
      config.min_convexity;
    Parameters::Blob::max_convexity =
      config.max_convexity;
    Parameters::Blob::min_inertia_ratio =
      config.min_inertia_ratio;
    Parameters::Blob::max_circularity =
      config.max_circularity;
    Parameters::Blob::min_circularity =
      config.min_circularity;
    Parameters::Blob::filter_by_color =
      config.filter_by_color;
    Parameters::Blob::filter_by_circularity =
      config.filter_by_circularity;


    ////////////////////////////// Debug parameters ////////////////////////////

    // Show the thermal image that arrives in the thermal node
    Parameters::Debug::show_thermal_image =
     config.show_thermal_image;

    Parameters::Debug::show_find_holes =
      config.show_find_holes;
    Parameters::Debug::show_find_holes_size =
      config.show_find_holes_size;

    Parameters::Debug::show_denoise_edges =
      config.show_denoise_edges;
    Parameters::Debug::show_denoise_edges_size =
      config.show_denoise_edges_size;

    Parameters::Debug::show_connect_pairs =
      config.show_connect_pairs;
    Parameters::Debug::show_connect_pairs_size =
      config.show_connect_pairs_size;

    Parameters::Debug::show_get_shapes_clear_border  =
      config.show_get_shapes_clear_border;
    Parameters::Debug::show_get_shapes_clear_border_size =
      config.show_get_shapes_clear_border_size;


    //------------------- Edge detection specific parameters -------------------

    // Canny parameters
    Parameters::Edge::canny_ratio =
      config.canny_ratio;
    Parameters::Edge::canny_kernel_size =
      config.canny_kernel_size;
    Parameters::Edge::canny_low_threshold =
      config.canny_low_threshold;
    Parameters::Edge::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;

    Parameters::Edge::edge_detection_method =
      config.edge_detection_method;

    // Threshold parameters
    Parameters::Edge::denoised_edges_threshold =
      config.denoised_edges_threshold;


    // Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::Image::scale_method = config.scale_method;


    //----------------- Outline discovery specific parameters ------------------

    // The detection method used to obtain the outline of a blob
    // 0 for detecting by means of brushfire
    // 1 for detecting by means of raycasting
    Parameters::Outline::outline_detection_method =
      config.outline_detection_method;

    // When using raycast instead of brushfire to find the (approximate here)
    // outline of blobs, raycast_keypoint_partitions dictates the number of
    // rays, or equivalently, the number of partitions in which the blob is
    // partitioned in search of the blob's borders
    Parameters::Outline::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //-------------------- Loose ends connection parameters --------------------

    Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    // In wavelet mode, the image shrinks by a factor of 4
    if (Parameters::Image::image_representation_method == 0)
    {
      Parameters::Outline::minimum_curve_points =
        config.minimum_curve_points;
    }
    else if (Parameters::Image::image_representation_method == 1)
    {
      Parameters::Outline::minimum_curve_points =
        static_cast<int>(config.minimum_curve_points / 4);
    }

    //-------------------- Probability extraction Parameters -------------------

    // The interpolation method for noise removal
    // 0 for averaging the pixel's neighbor values
    // 1 for brushfire near
    // 2 for brushfire far
    Parameters::Thermal::probability_method = config.probability_method;
    Parameters::Thermal::min_thermal_probability = config.min_thermal_probability;

    //-------------------- Gausian variables ---------------------
    Parameters::Thermal::optimal_temperature = config.optimal_temperature;

    // As the standard deviation(tolerance) is higher we consider
    // more temperatures as valid. We handle the tolerance.
    Parameters::Thermal::tolerance = config.tolerance;

    //-------------------- Logistic variables ---------------------

    // These temperatures have 50% probability.
    // Inside their range the probability grows.
    Parameters::Thermal::low_acceptable_temperature =
      config.low_acceptable_temperature;
    Parameters::Thermal::high_acceptable_temperature =
      config.high_acceptable_temperature;

    // These variables handle the tolerance of the temperatures.
    // left is for the left side of the destributions curve
    // right is for the right side of the destributions curve
    Parameters::Thermal::left_tolerance = config.left_tolerance;
    Parameters::Thermal::right_tolerance = config.right_tolerance;

    //-------------- Low and High accepted Temperatures ---------------------

    Parameters::Thermal::low_temperature = config.low_temperature;
    Parameters::Thermal::high_temperature = config.high_temperature;

    //-------------- Transformation coords ---------------------

    Parameters::Thermal::xThermal = config.xThermal;
    Parameters::Thermal::yThermal = config.yThermal;
    Parameters::Thermal::c_x = config.c_x;
    Parameters::Thermal::c_y = config.c_y;
    Parameters::Thermal::angle = config.angle;
  }

}  // namespace thermal
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
