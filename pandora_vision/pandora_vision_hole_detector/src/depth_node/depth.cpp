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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "depth_node/depth.h"

namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  Depth::Depth(void)
  {
    #ifdef DEBUG_TIME
    Timer::start("Depth");
    #endif

    ros::Duration(0.5).sleep();

    // Subscribe to the depth image published by the
    // rgb_depth_synchronizer node
    depthImageSubscriber_ = nodeHandle_.subscribe(
      Parameters::Topics::depth_image_topic, 1,
      &Depth::inputDepthImageCallback, this);

    // Advertise the candidate holes found by the depth node
    candidateHolesPublisher_ = nodeHandle_.advertise
      <vision_communications::CandidateHolesVectorMsg>(
      Parameters::Topics::depth_candidate_holes_topic, 1000);

    // The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&Depth::parametersCallback,
        this, _1, _2));

    ROS_INFO("Depth node initiated");

    #ifdef DEBUG_TIME
    Timer::tick("Depth");
    #endif
  }



  /**
    @brief Default destructor
    @return void
   **/
  Depth::~Depth(void)
  {
    ROS_INFO("Depth node terminated");
  }



  /**
    @brief Callback for depth image
    @param msg [const sensor_msgs::Image&] The depth image message
    @return void
   **/
  void Depth::inputDepthImageCallback(
    const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputDepthImageCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Depth node callback");
    #endif

    // Obtain the depth image
    cv::Mat depthImage;
    MessageConversions::extractImageFromMessage(msg, &depthImage,
      sensor_msgs::image_encodings::TYPE_32FC1);

    #ifdef DEBUG_SHOW
    if (Parameters::Depth::show_depth_image)
    {
      Visualization::showScaled("Depth image", depthImage, 1);
    }
    #endif

    // Perform noise elimination on the depth image
    cv::Mat interpolatedDepthImage;
    NoiseElimination::performNoiseElimination(depthImage,
      &interpolatedDepthImage);

    // Regardless of the image representation method, the depth node
    // will publish the interpolated depth image of original size
    // to the Hole Fusion node
    cv::Mat interpolatedDepthImageSent;
    interpolatedDepthImage.copyTo(interpolatedDepthImageSent);

    // A value of 1 means that the depth image is subtituted by its
    // low-low, wavelet analysis driven, part
    if (Parameters::Image::image_representation_method == 1)
    {
      double min;
      double max;
      cv::minMaxIdx(depthImage, &min, &max);

      Wavelets::getLowLow(interpolatedDepthImage, min, max,
        &interpolatedDepthImage);
    }

    // Finds possible holes
    HolesConveyor holes = HoleDetector::findHoles(interpolatedDepthImage);

    // Create the candidate holes message
    vision_communications::CandidateHolesVectorMsg depthCandidateHolesMsg;

    MessageConversions::createCandidateHolesVectorMessage(holes,
      interpolatedDepthImageSent,
      &depthCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_32FC1,
      msg);

    // Publish the candidate holes message
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
  void Depth::parametersCallback(
    const pandora_vision_hole_detector::depth_cfgConfig& config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("[Depth node] Parameters callback called");
    #endif

    // Blob detection - specific parameters
    Parameters::Blob::blob_min_threshold =
      config.blob_min_threshold;
    Parameters::Blob::blob_max_threshold =
      config.blob_max_threshold;
    Parameters::Blob::blob_threshold_step =
      config.blob_threshold_step;

    //!< In wavelet mode, the image shrinks by a factor of 4
    if (config.image_representation_method == 0)
    {
      Parameters::Blob::blob_min_area =
        config.blob_min_area;
      Parameters::Blob::blob_max_area =
        config.blob_max_area;
    }
    else if (config.image_representation_method == 1)
    {
      Parameters::Blob::blob_min_area =
        static_cast<int>(config.blob_min_area / 4);
      Parameters::Blob::blob_max_area =
        static_cast<int>(config.blob_max_area / 4);
    }

    Parameters::Blob::blob_min_convexity =
      config.blob_min_convexity;
    Parameters::Blob::blob_max_convexity =
      config.blob_max_convexity;
    Parameters::Blob::blob_min_inertia_ratio =
      config.blob_min_inertia_ratio;
    Parameters::Blob::blob_max_circularity =
      config.blob_max_circularity;
    Parameters::Blob::blob_min_circularity =
      config.blob_min_circularity;
    Parameters::Blob::blob_filter_by_color =
      config.blob_filter_by_color;
    Parameters::Blob::blob_filter_by_circularity =
      config.blob_filter_by_circularity;


    // Debug
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


    // Parameters specific to the Depth node

    // Show the depth image that arrives in the depth node
    Parameters::Depth::show_depth_image =
     config.show_depth_image;

    // The interpolation method for noise removal
    // 0 for averaging the pixel's neighbor values
    // 1 for brushfire near
    // 2 for brushfire far
    Parameters::Depth::interpolation_method = config.interpolation_method;


    // Edge detection specific parameters

    // canny parameters
    Parameters::Edge::canny_ratio =
      config.canny_ratio;
    Parameters::Edge::canny_kernel_size =
      config.canny_kernel_size;
    Parameters::Edge::canny_low_threshold =
      config.canny_low_threshold;
    Parameters::Edge::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;

    Parameters::Edge::contrast_enhance_alpha =
      config.contrast_enhance_alpha;
    Parameters::Edge::contrast_enhance_beta =
      config.contrast_enhance_beta;

    Parameters::Edge::edge_detection_method =
      config.edge_detection_method;

    // Threshold parameters
    Parameters::Edge::denoised_edges_threshold =
      config.denoised_edges_threshold;


    // Image representation specific parameters

    // Depth image representation method.
    // 0 if the depth image used is the one obtained from the depth sensor,
    // unadulterated
    // 1 through wavelet representation
    Parameters::Image::image_representation_method =
      config.image_representation_method;

    // Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::Image::scale_method = config.scale_method;


    // Outline discovery specific parameters

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

    //<! Loose ends connection parameters
    Parameters::Outline::AB_to_MO_ratio = config.AB_to_MO_ratio;

    //!< In wavelet mode, the image shrinks by a factor of 4
    if (config.image_representation_method == 0)
    {
      Parameters::Outline::minimum_curve_points =
        config.minimum_curve_points;
    }
    else if (config.image_representation_method == 1)
    {
      Parameters::Outline::minimum_curve_points =
        static_cast<int>(config.minimum_curve_points / 4);
    }

  }

} // namespace pandora_vision
