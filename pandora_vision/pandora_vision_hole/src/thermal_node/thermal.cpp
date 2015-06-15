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

#include "thermal_node/thermal.h"

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
  Thermal::Thermal(void)
  {
    // Acquire the names of topics which the thermal node will be having
    // transactionary affairs with
    getTopicNames();

    // Get the values of the variables used to match the final holeConveyors
    ImageMatching::variableSetUp(nodeHandle_, &xThermal_, &yThermal_,
      &cX_, &cY_, &angle_);

    // Subscribe to the thermal image published by raspberry
    thermalImageSubscriber_ = nodeHandle_.subscribe(thermalImageTopic_, 1,
      &Thermal::inputThermalImageCallback, this);

    // Advertise the candidate holes found by the thermal node to hole fusion
    candidateHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_hole::CandidateHolesVectorMsg>(
      candidateHolesTopic_, 1000);

    // Advertise the candidate holes found by the thermal
    // node to thermal cropper
    thermalToCropperPublisher_ = nodeHandle_.advertise
      <pandora_vision_hole::CandidateHolesVectorMsg>(
      thermalToCropperTopic_, 1000);

    // Advertise the candidate holes found by the thermal node to hole fusion
    dataFusionThermalPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::ThermalAlertVector>
      (dataFusionThermalTopic_, 1000);

    // The dynamic reconfigure (thermal) parameter's callback
    server.setCallback(boost::bind(&Thermal::parametersCallback, this, _1, _2));

    ROS_INFO_NAMED(PKG_NAME, "[Thermal node] Initiated");
  }



  /**
    @brief Default destructor
    @return void
   **/
  Thermal::~Thermal(void)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Thermal node] Terminated");
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
  void Thermal::inputThermalImageCallback(
    const pandora_vision_msgs::IndexedThermal&  msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputThermalImageCallback", "", true);
    #endif

    ROS_INFO_NAMED(PKG_NAME, "Thermal node callback");

    // Obtain the thermal image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be CV_8UC1.

    cv::Mat thermalImage;
    MessageConversions::extractImageFromMessage(msg.thermalImage, &thermalImage,
      sensor_msgs::image_encodings::TYPE_8UC1);

    // Obtain the thermal message and extract the temperature information.
    // Convert this information to cv::Mat in order to be processed.
    // It's format will be CV_8UC1
    //cv::Mat thermalImage = MessageConversions::convertFloat32MultiArrayToMat
      //(msg.temperatures);

    // Apply double threshold(up and down) in the temperature image.
    // The threshold is set by configuration
    //cv::inRange(
      //thermalImage, cv::Scalar(30), cv::Scalar(40), thermalImage);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_thermal_image)
    {
      Visualization::showScaled("Thermal image", thermalImage, 1);
    }
    #endif

    // Locate potential holes in the thermal image
    HolesConveyor holes = HoleDetector::findHoles(thermalImage);

    // Find the average temperature of each point of interest that we found
    // and their probability.
    findHolesProbability(
      &holes, msg.temperatures, Parameters::Thermal::probability_method);

    // Convert the conveyors information so it can match with the
    //  Rgb and Depth images. If its outside of limits discart that conveyor.
    ImageMatching::conveyorMatching(&holes, xThermal_, yThermal_, cX_,
      cY_, angle_);

    // Resize the thermal image to match the rgb-d images, of course the thermal
    // image will not be further processed.
    cv::resize(thermalImage, thermalImage, cvSize(
        Parameters::Image::WIDTH, Parameters::Image::HEIGHT));

    // Create the candidate holes message
    pandora_vision_hole::CandidateHolesVectorMsg thermalCandidateHolesMsg;

    // Pack information about holes found and the thermal image
    // inside a message.
    // This message will be published to and received by the hole fusion
    // or thermal cropper node based on the index of thermal message acquired
    // from the synchronizer node
    MessageConversions::createCandidateHolesVectorMessage(holes,
      thermalImage,
      &thermalCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_8UC1,
      msg.thermalImage);

    // Check the index and send the message to its proper receiver
    // The index takes only three values from synchronized node
    // "thermal", "hole" or "thermalhole".
    if(msg.thermalIndex == "thermal")
    {
      // Publish the candidate holes message to thermal cropper node
      thermalToCropperPublisher_.publish(thermalCandidateHolesMsg);
    }
    else if (msg.thermalIndex == "hole")
    {
      // Publish the candidate holes message to hole fusion node
      candidateHolesPublisher_.publish(thermalCandidateHolesMsg);
    }
    else
    {
      // Publish to both thermal cropper and hole fusion nodes
      thermalToCropperPublisher_.publish(thermalCandidateHolesMsg);
      candidateHolesPublisher_.publish(thermalCandidateHolesMsg);
    }

    // Finally find the yaw and pitch of each candidate hole found and
    // send it to data fusion if a hole exists. The message to be sent is
    // ThermalAlertsVectorMsg type.
    if(holes.size() > 0)
    {
      // Fill the thermal message to be sent
      pandora_vision_msgs::ThermalAlert thermalMsg;
      pandora_vision_msgs::ThermalAlertVector thermalMsgVector;

      POIsStampedPtr poisStamped(new POIsStamped);

      poisStamped->header.stamp = msg.header.stamp;
      poisStamped->header.frame_id = "/kinect_rgb_optical_frame";
      poisStamped->frameWidth = thermalImage.cols;
      poisStamped->frameHeight = thermalImage.rows;

      // For each hole found by thermal node
      for(unsigned int i = 0; i < holes.size(); i++)
      {

        POIPtr poi(new POI);

        // Set the keypoint
        poi->point.x = holes.holes[i].keypoint.pt.x;
        poi->point.y = holes.holes[i].keypoint.pt.y;

        // Fill the probabilities
        poi->probability = holes.holes[i].holeProbability;

        poisStamped->pois.push_back(poi);
      }

      GeneralAlertConverter converter;

      pandora_common_msgs::GeneralAlertVector alert =
        converter.getGeneralAlertInfo(ros::this_node::getName(),
          nodeHandle_, poisStamped);

      for(unsigned int i = 0; i < alert.alerts.size(); i++)
      {
        // Fill the temperature of the thermal message for each hole
        thermalMsg.temperature = holes.holes[i].holeTemperature;

        thermalMsg.info = alert.alerts.at(i);

        // Push back into vector of messages
        thermalMsgVector.alerts.push_back(thermalMsg);
      }
      // Fill the thermal message header to be sent
      thermalMsgVector.header = alert.header;

      // Publish the thermal message
      dataFusionThermalPublisher_.publish(thermalMsgVector);
    }

    #ifdef DEBUG_TIME
    Timer::tick("inputThermalImageCallback");
    Timer::printAllMeansTree();
    #endif

    return;
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the thermal node
    @param void
    @return void
   **/
  void Thermal::getTopicNames()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the thermal node acquires the
    // unadulterated thermal image and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/subscribed_topics/thermal_image_topic",
        thermalImageTopic_ ))
    {

      // Make topic's name absolute
      thermalImageTopic_ = ns + "/" + thermalImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[Thermal Node] Subscribed to the input thermal image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Thermal Node] Could not find topic thermal_image_topic");
    }

    // Read the name of the topic to which the thermal node will be publishing
    // information to hole-fusion about the candidate holes found
    // and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/candidate_holes_topic",
        candidateHolesTopic_))
    {
      // Make the topic's name absolute
      candidateHolesTopic_ = ns + "/" + candidateHolesTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[Thermal Node] Advertising to the candidate holes topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Thermal Node] Could not find topic candidate_holes_topic");
    }

    // Read the name of the topic to which the thermal node will be publishing
    // information to thermal cropper about the candidate holes found
    // and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/thermal_to_cropper_topic",
        thermalToCropperTopic_))
    {
      // Make the topic's name absolute
      thermalToCropperTopic_ = ns + "/" + thermalToCropperTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[Thermal Node] Advertising to the Thermal-Therma Cropper topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Thermal Node] Could not find topic thermal_to_cropper_topic");
    }

    // Read the name of the topic to which the thermal node will be publishing
    // information directly to Data fusion about the candidate holes found
    // and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/thermal_data_fusion_topic",
        dataFusionThermalTopic_))
    {
      ROS_INFO_NAMED(PKG_NAME,
        "[Thermal Node] Advertising to the Thermal-Data fusion topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[Thermal Node] Could not find topic thermal_data_fusion_topic");
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
  void Thermal::findHolesProbability(HolesConveyor* holes,
    const std_msgs::Float32MultiArray& temperatures, const int& method)
  {
    // The width and height of the input temperature multiarray
    int width = temperatures.layout.dim[1].size;
    int height = temperatures.layout.dim[0].size;

    // For each hole find its probability
    for(unsigned int i = 0; i < holes->size(); i++)
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
      if(tempX > 0 && tempX < 60 && tempY > 0 && tempY < 80)
      {
        int counter = 0;

        for(unsigned int k = (tempY - 1); k < (tempY + 2); k++)
        {
          for(unsigned int o = (tempX - 1); o < (tempX + 2); o++)
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
      if(method == 0)
      {
        float probability = exp(
          - pow((average - Parameters::Thermal::optimal_temperature), 2)
          / (2 * pow(Parameters::Thermal::tolerance , 2)));

        // Push the probability in the vector
        holes->holes[i].holeProbability = probability;
      }
      // Apply logistic function on the average temperature found
      else if(method == 1)
      {
        float probability = 1/(1 + exp( -Parameters::Thermal::left_tolerance *
            (average - Parameters::Thermal::low_acceptable_temperature ) ))
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
  void Thermal::parametersCallback(
    const pandora_vision_hole::thermal_cfgConfig& config,
    const uint32_t& level)
  {
    ROS_INFO_NAMED(PKG_NAME, "[Thermal node] Parameters callback called");

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
  }

} // namespace pandora_vision
