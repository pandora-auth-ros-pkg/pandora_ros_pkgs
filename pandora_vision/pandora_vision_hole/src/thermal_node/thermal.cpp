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
 * Authors: Alexandros Philotheou, Manos Tsardoulias,Angelos Triantafyllidis
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
    ImageMatching::variableSetUp(nodeHandle_, &xThermal_, &yThermal_, &cX_, &cY_);

    // Subscribe to the thermal image published by raspberry
    thermalImageSubscriber_ = nodeHandle_.subscribe(thermalImageTopic_, 1,
      &Thermal::inputThermalImageCallback, this);

    // Advertise the candidate holes found by the thermal node
    candidateHolesPublisher_ = nodeHandle_.advertise
      <pandora_vision_hole::CandidateHolesVectorMsg>(
      candidateHolesTopic_, 1000);

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
    @param msg [const sensor_msgs::Image&]
    The thermal image message
    @return void
   **/
  void Thermal::inputThermalImageCallback(const sensor_msgs::Image&  msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputThermalImageCallback", "", true);
    #endif

    ROS_INFO_NAMED(PKG_NAME, "Thermal node callback");

    // Obtain the thermal image. Since the image is in a format of
    // sensor_msgs::Image, it has to be transformed into a cv format in order
    // to be processed. Its cv format will be CV_8UC1.

    cv::Mat thermalImage;
    MessageConversions::extractImageFromMessage(msg, &thermalImage, 
      sensor_msgs::image_encodings::TYPE_8UC1);

    #ifdef DEBUG_SHOW
    if (Parameters::Debug::show_thermal_image)
    {
      Visualization::showScaled("Thermal image", thermalImage, 1);
    }
    #endif

    // Locate potential holes in the thermal image
    HolesConveyor holes = HoleDetector::findHoles(thermalImage);

    // Convert the conveyors information so it can match with the
    //  Rgb and Depth images
    ImageMatching::conveyorMatching(&holes, xThermal_, yThermal_, cX_, cY_);

    // Create the candidate holes message
    pandora_vision_hole::CandidateHolesVectorMsg thermalCandidateHolesMsg;

    // Pack information about holes found and the interpolated depth image
    // inside a message.
    // This message will be published to and received by the hole fusion node
    MessageConversions::createCandidateHolesVectorMessage(holes,
      thermalImage,
      &thermalCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_8UC1,
      msg);

    // Publish the candidate holes message
    candidateHolesPublisher_.publish(thermalCandidateHolesMsg);

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
  void Thermal::getTopicNames ()
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
      //thermalImageTopic_ = ns + "/" + thermalImageTopic_;

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
    // information directly to Data fusion about the candidate holes found 
    // and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_camera_node/published_topics/thermal_data_fusion_topic",
        holeFusionThermalTopic_))
    {
      // Make the topic's name absolute
      holeFusionThermalTopic_ = ns + "/" + holeFusionThermalTopic_;

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
  }

} // namespace pandora_vision
