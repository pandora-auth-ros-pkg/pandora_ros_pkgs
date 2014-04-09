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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#include "depth_node/depth.h"

namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  PandoraKinect::PandoraKinect(void)
  {
    #ifdef DEBUG_TIME
    Timer::start("PandoraKinect");
    #endif

    ros::Duration(0.5).sleep();

/*
 *    //!< Subscribe to the point cloud published by the
 *    //!< rgb_depth_synchronizer node
 *    inputCloudSubscriber_ = nodeHandle_.subscribe(
 *      "/synchronized/camera/depth/points", 1,
 *      &PandoraKinect::inputCloudCallback, this);
 *
 */
    //!< Subscribe to the point cloud published by the
    //!< rgb_depth_synchronizer node
    depthImageSubscriber_ = nodeHandle_.subscribe(
      "/synchronized/camera/depth/image_raw", 1,
      &PandoraKinect::inputDepthImageCallback, this);

    //!< Advertise the candidate holes found by the depth node
    candidateHolesPublisher_ = nodeHandle_.advertise
      <vision_communications::CandidateHolesVectorMsg>(
      "/synchronized/camera/depth/candidate_holes", 1000);

    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&PandoraKinect::parametersCallback,
        this, _1, _2));

    ROS_INFO("Depth node initiated");

    #ifdef DEBUG_TIME
    Timer::tick("PandoraKinect");
    #endif
  }



  /**
    @brief Default destructor
    @return void
   **/
  PandoraKinect::~PandoraKinect(void)
  {
    ROS_INFO("Depth node terminated");
  }



  /**
    @brief Callback for depth image
    @param msg [const sensor_msgs::Image&] The depth image message
    @return void
   **/
  void PandoraKinect::inputDepthImageCallback(
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputDepthImageCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Depth node callback");
    #endif

    cv::Mat depthImage;
    MessageConversions::extractImageFromMessage(msg, &depthImage,
      sensor_msgs::image_encodings::TYPE_32FC1);

    //!< Finds possible holes
    cv::Mat interpolatedDepthImage;
    HolesConveyor holes = HoleDetector::findHoles(depthImage,
      &interpolatedDepthImage);

    //!< Create the candidate holes message
    vision_communications::CandidateHolesVectorMsg depthCandidateHolesMsg;

    MessageConversions::createCandidateHolesVectorMessage(holes,
      interpolatedDepthImage,
      &depthCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_32FC1,
      msg);

    //!< Publish the candidate holes message
    candidateHolesPublisher_.publish(depthCandidateHolesMsg);

    #ifdef DEBUG_TIME
    Timer::tick("inputDepthImageCallback");
    Timer::printAllMeansTree();
    #endif

    return;
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole_detector::depth_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void PandoraKinect::parametersCallback(
    const pandora_vision_hole_detector::depth_cfgConfig& config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Parameters callback called");
    #endif

    //!< Kanny parameters
    Parameters::kanny_ratio = config.kanny_ratio;
    Parameters::kanny_kernel_size = config.kanny_kernel_size;
    Parameters::kanny_low_threshold = config.kanny_low_threshold;
    Parameters::kanny_blur_noise_kernel_size =
      config.kanny_blur_noise_kernel_size;

    Parameters::contrast_enhance_alpha = config.contrast_enhance_alpha;
    Parameters::contrast_enhance_beta = config.contrast_enhance_beta;

    //!< Threshold parameters
    Parameters::threshold_lower_value = config.threshold_lower_value;

    //!< Blob detection parameters
    Parameters::blob_min_threshold = config.blob_min_threshold;
    Parameters::blob_max_threshold = config.blob_max_threshold;
    Parameters::blob_threshold_step = config.blob_threshold_step;
    Parameters::blob_min_area = config.blob_min_area;
    Parameters::blob_max_area = config.blob_max_area;
    Parameters::blob_min_convexity = config.blob_min_convexity;
    Parameters::blob_max_convexity = config.blob_max_convexity;
    Parameters::blob_min_inertia_ratio = config.blob_min_inertia_ratio;
    Parameters::blob_max_circularity = config.blob_max_circularity;
    Parameters::blob_min_circularity = config.blob_min_circularity;
    Parameters::blob_filter_by_color = config.blob_filter_by_color;
    Parameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;

    //!< Bounding boxes parameters
    Parameters::bounding_box_min_area_threshold =
      config.bounding_box_min_area_threshold;

    //!< The bounding box detection method
    //!< 0 for detecting by means of brushfire starting
    //!< from the keypoint of the blob
    //!< 1 for detecting by means of contours around the edges of the blob
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;

    //!< When using raycast instead of brushfire to find the (approximate here)
    //!< outline of blobs, raycast_keypoint_partitions dictates the number of
    //!< rays, or equivalently, the number of partitions in which the blob is
    //!< partitioned in search of the blob's borders
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //<! Loose ends connection parameters
    Parameters::AB_to_MO_ratio = config.AB_to_MO_ratio;
    Parameters::minimum_curve_points = config.minimum_curve_points;

    //!< The interpolation method for noise removal
    //!< 0 for averaging the pixel's neighbor values
    //!< 1 for brushfire near
    //!< 2 for brushfire far
    Parameters::interpolation_method = config.interpolation_method;

    //!< Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::scale_method = config.scale_method;

    //!< Debug
    Parameters::debug_show_find_holes = config.debug_show_find_holes;
    Parameters::debug_show_find_holes_size =
      config.debug_show_find_holes_size;

    Parameters::debug_show_denoise_edges = config.debug_show_denoise_edges;
    Parameters::debug_show_denoise_edges_size =
      config.debug_show_denoise_edges_size;

    Parameters::debug_show_connect_pairs = config.debug_show_connect_pairs;
    Parameters::debug_show_connect_pairs_size =
      config.debug_show_connect_pairs_size;

    Parameters::debug_show_get_shapes_clear_border  =
      config.debug_show_get_shapes_clear_border;
    Parameters::debug_show_get_shapes_clear_border_size =
      config.debug_show_get_shapes_clear_border_size;
  }

} // namespace pandora_vision
