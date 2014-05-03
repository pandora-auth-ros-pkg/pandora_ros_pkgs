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
* Author: Despoina Paschalidou
*********************************************************************/

#include "rgb_node/rgb.h"

namespace pandora_vision
{
  /**
    @brief Constructor
  **/
  Rgb::Rgb(): _nh(), holeNowON(false)
  {
    // Subscribe to the RGB image published by the
    // rgb_depth_synchronizer node
    _frameSubscriber = _nh.subscribe(
      "/synchronized/camera/rgb/image_raw", 1,
      &Rgb::inputRgbImageCallback, this);

    // Advertise the candidate holes found by the depth node
    rgbCandidateHolesPublisher_ = _nh.advertise
      <vision_communications::CandidateHolesVectorMsg>(
      "/synchronized/camera/rgb/candidate_holes", 1000);

    // The dynamic reconfigure (RGB) parameter's callback
    server.setCallback(boost::bind(&Rgb::parametersCallback,
        this, _1, _2));

    ROS_INFO("[rgb_node] : Created Rgb instance");
  }



  /**
    @brief Destructor
   **/
  Rgb::~Rgb()
  {
    ROS_DEBUG("[rgb_node] : Destroying Hole Detection instance");
  }



  /**
    @brief Function called when new ROS message appears, for camera
    @param msg [const sensor_msgs::ImageConstPtr&] The message
    @return void
  */
  void Rgb::inputRgbImageCallback(const sensor_msgs::Image& msg)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("RGB node callback");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("inputRgbImageCallback", "", true);
    #endif

    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _holeFrame = in_msg->image.clone();
    _holeFrameTimestamp = msg.header.stamp;

    if (_holeFrame.empty() )
    {
      ROS_ERROR("[rgb_node] : No more Frames");
      return;
    }

    #ifdef DEBUG_SHOW
    if (Parameters::show_rgb_image)
    {
      Visualization::show("RGB image", _holeFrame, 1);
    }
    #endif

    // Regardless of the image representation method, the RGB node
    // will publish the RGB image of original size to the Hole Fusion node
    cv::Mat holeFrameSent;
    _holeFrame.copyTo(holeFrameSent);

    // A value of 1 means that the rgb image is subtituted by its
    // low-low, wavelet analysis driven, part
    if (Parameters::image_representation_method == 1)
    {
      Wavelets::getLowLow(_holeFrame, &_holeFrame);
    }

    // Find candidate holes in the current frame
    HolesConveyor conveyor = _holeDetector.findHoles(_holeFrame);

    // With candidate holes found, create the message that includes them
    // and the original RGB image and send them over to the HoleFusion node
    vision_communications::CandidateHolesVectorMsg rgbCandidateHolesMsg;

    MessageConversions::createCandidateHolesVectorMessage(conveyor,
      holeFrameSent,
      &rgbCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_8UC3, msg);

    rgbCandidateHolesPublisher_.publish(rgbCandidateHolesMsg);

    #ifdef DEBUG_TIME
    Timer::tick("inputRgbImageCallback");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_hole_detector::rgb_cfgConfig&]
    @param[in] level [const uint32_t] The level (?)
    @return void
   **/
  void Rgb::parametersCallback(
    const pandora_vision_hole_detector::rgb_cfgConfig& config,
    const uint32_t& level)
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Parameters callback called");
    #endif

    // Show the rgb image that arrives in the rgb node
    Parameters::show_rgb_image =
     config.show_rgb_image;

    // RGB image representation method.
    // 0 if the depth image used is the one obtained from the depth sensor,
    // unadulterated
    // 1 through wavelet representation
    Parameters::image_representation_method =
      config.image_representation_method;

    // canny parameters
    Parameters::canny_ratio = config.canny_ratio;
    Parameters::canny_kernel_size = config.canny_kernel_size;
    Parameters::canny_low_threshold = config.canny_low_threshold;
    Parameters::canny_blur_noise_kernel_size =
      config.canny_blur_noise_kernel_size;

    Parameters::contrast_enhance_beta = config.contrast_enhance_beta;
    Parameters::contrast_enhance_alpha = config.contrast_enhance_alpha;

    // Threshold parameters
    Parameters::threshold_lower_value = config.threshold_lower_value;

    // Blob detection parameters
    Parameters::blob_min_threshold = config.blob_min_threshold;
    Parameters::blob_max_threshold = config.blob_max_threshold;
    Parameters::blob_threshold_step = config.blob_threshold_step;

    //!< In wavelet mode, the image shrinks by a factor of 4
    if (config.image_representation_method == 0)
    {
      Parameters::blob_min_area = config.blob_min_area;
      Parameters::blob_max_area = config.blob_max_area;
    }
    else if (config.image_representation_method == 1)
    {
      Parameters::blob_min_area = static_cast<int>(config.blob_min_area / 4);
      Parameters::blob_max_area = static_cast<int>(config.blob_max_area / 4);
    }

    Parameters::blob_min_convexity = config.blob_min_convexity;
    Parameters::blob_max_convexity = config.blob_max_convexity;
    Parameters::blob_min_inertia_ratio = config.blob_min_inertia_ratio;
    Parameters::blob_max_circularity = config.blob_max_circularity;
    Parameters::blob_min_circularity = config.blob_min_circularity;
    Parameters::blob_filter_by_color = config.blob_filter_by_color;
    Parameters::blob_filter_by_circularity =
      config.blob_filter_by_circularity;

    // Bounding boxes parameters

    // The bounding box detection method
    // 0 for detecting by means of brushfire starting
    // from the keypoint of the blob
    // 1 for detecting by means of contours around the edges of the blob
    Parameters::bounding_box_detection_method =
      config.bounding_box_detection_method;

    // When using raycast instead of brushfire to find the (approximate here)
    // outline of blobs, raycast_keypoint_partitions dictates the number of
    // rays, or equivalently, the number of partitions in which the blob is
    // partitioned in search of the blob's borders
    Parameters::raycast_keypoint_partitions =
      config.raycast_keypoint_partitions;

    //<! Loose ends connection parameters
    Parameters::AB_to_MO_ratio = config.AB_to_MO_ratio;
    Parameters::minimum_curve_points = config.minimum_curve_points;

    // Interpolation parameters

    // The interpolation method for noise removal
    // 0 for averaging the pixel's neighbor values
    // 1 for brushfire near
    // 2 for brushfire far
    Parameters::interpolation_method = config.interpolation_method;

    // Method to scale the CV_32FC1 image to CV_8UC1
    Parameters::scale_method = config.scale_method;

    // Parameters needed for histogram calculation
    Parameters::number_of_hue_bins = config.number_of_hue_bins;
    Parameters::number_of_saturation_bins = config.number_of_saturation_bins;
    Parameters::number_of_value_bins = config.number_of_value_bins;
    Parameters::secondary_channel = config.secondary_channel;


    // Debug
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
