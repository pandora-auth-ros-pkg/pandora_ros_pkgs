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

#include "utils/parameters.h"

namespace pandora_vision
{
  // Topics' names

  // The input, depth registered, point cloud topic
  std::string Parameters::hole_detector_input_topic =
    "/camera/depth_registered/points";

  // The topic where the synchronizer node pubishes the
  // synchronized depth image, extracted from the input point cloud
  std::string Parameters::depth_image_topic =
    "synchronized/camera/depth/image_raw";

  // The topic where the synchronizer node pubishes the
  // synchronized rgb image, extracted from the input point cloud
  std::string Parameters::rgb_image_topic =
    "synchronized/camera/rgb/image_raw";

  // The topic where the hole fusion publishes messages that unlock the
  // synchronizer node
  std::string Parameters::synchronizer_unlock_topic =
    "unlock_rgb_depth_synchronizer";

  // The topic where the synchronized node publishes the input
  // point cloud to the hole fusion node
  std::string Parameters::point_cloud_internal_topic =
    "synchronized/camera/depth/points";

  // The topic where the depth node publishes the candidate holes found
  std::string Parameters::depth_candidate_holes_topic =
    "synchronized/camera/depth/candidate_holes";

  // The topic where the rgb node publishes the candidate holes found
  std::string Parameters::rgb_candidate_holes_topic =
    "synchronized/camera/rgb/candidate_holes";

  // The topic where the hole detector package publishes information
  // about the holes that considered valid
  std::string Parameters::hole_detector_output_topic =
    "holes_direction";

  // The topic where the hole detector package publishes enhancement
  // information about the holes considered valid
  std::string Parameters::enhanced_holes_topic =
    "enhanced_holes";


  // Show the depth image that arrives in the depth node
  bool Parameters::show_depth_image = false;

  // Show the rgb image that arrives in the rgb node
  bool Parameters::show_rgb_image = false;

  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool Parameters::show_respective_holes = false;

  // The product of this package: valid holes
  bool Parameters::show_final_holes = false;

  // Depth and RGB images' representation method.
  // 0 if image used is used as obtained from the image sensor
  // 1 through wavelet analysis
  int Parameters::image_representation_method = 0;

  // Edge detection parameters
  int Parameters::edge_detection_method = 2;

  // When mixed edge detection is selected, this toggle switch
  // is needed in order to shift execution from one edge detector
  // to the other.
  // 1 for the Scharr edge detector,
  // 2 for the Sobel edge detector
  int Parameters::mixed_edges_toggle_switch = 1;

  // canny parameters
  int Parameters::canny_ratio = 3;
  int Parameters::canny_kernel_size = 3;
  int Parameters::canny_low_threshold = 50;
  int Parameters::canny_blur_noise_kernel_size = 3;

  float Parameters::contrast_enhance_alpha = 2;
  float Parameters::contrast_enhance_beta = 2;


  // Threshold parameters
  int Parameters::threshold_lower_value = 10;

  // Blob detection parameters
  int Parameters::blob_min_threshold = 0;
  int Parameters::blob_max_threshold = 1255;
  int Parameters::blob_threshold_step = 5;
  int Parameters::blob_min_area = 550;
  int Parameters::blob_max_area = 300000;
  double Parameters::blob_min_convexity = 0;
  double Parameters::blob_max_convexity = 100;
  double Parameters::blob_min_inertia_ratio = 0;
  double Parameters::blob_max_circularity = 1.0;
  double Parameters::blob_min_circularity = 0.3;
  bool Parameters::blob_filter_by_color = 0;
  bool Parameters::blob_filter_by_circularity = 1;

  // Bounding boxes parameters

  // The bounding box detection method
  // 0 for detecting by means of brushfire starting
  // from the keypoint of the blob
  // 1 for detecting by means of contours around the edges of the blob
  int Parameters::bounding_box_detection_method = 0;

  // When using raycast instead of brushfire to find the (approximate here)
  // outline of blobs, raycast_keypoint_partitions dictates the number of
  // rays, or equivalently, the number of partitions in which the blob is
  // partitioned in search of the blob's borders
  int Parameters::raycast_keypoint_partitions = 8;

  //<! Loose ends connection parameters
  int Parameters::AB_to_MO_ratio = 4;
  int Parameters::minimum_curve_points = 100;


  //// Interpolation parameters

  // The interpolation method for noise removal
  // 0 for averaging the pixel's neighbor values
  // 1 for brushfire near
  // 2 for brushfire far
  int Parameters::interpolation_method = 0;

  // Hole checkers and their thresholds`
  int Parameters::run_checker_depth_diff = 1;
  float Parameters::checker_depth_diff_threshold = 0.4;

  int Parameters::run_checker_depth_area = 3;
  float Parameters::checker_depth_area_threshold = 0.4;

  int Parameters::run_checker_brushfire_outline_to_rectangle = 4;
  float Parameters::checker_brushfire_outline_to_rectangle_threshold = 0.4;

  int Parameters::run_checker_outline_of_rectangle = 2;
  float Parameters::checker_outline_of_rectangle_threshold = 0.75;

  int Parameters::run_checker_depth_homogeneity = 5;
  float Parameters::checker_depth_homogeneity_threshold = 0.2;

  int Parameters::rectangle_inflation_size = 20;
  float Parameters::holes_gaussian_mean = 0.3;
  float Parameters::holes_gaussian_stddev = 0.3;


  int Parameters::run_checker_color_homogeneity = 1;
  float Parameters::checker_color_homogeneity_threshold = 0.4;

  int Parameters::run_checker_luminosity_diff = 2;
  float Parameters::checker_luminosity_diff_threshold = 0.4;

  int Parameters::run_checker_texture_diff = 3;
  float Parameters::checker_texture_diff_threshold = 0.4;

  int Parameters::run_checker_texture_backproject = 4;
  float Parameters::checker_texture_backproject_threshold = 0.4;

  // Plane detection
  int Parameters::segmentation_method = 0;
  int Parameters::max_iterations = 1000;
  double Parameters::num_points_to_exclude = 0.1;
  double Parameters::point_to_plane_distance_threshold = 0.01;

  // Method to scale the CV_32FC1 image to CV_8UC1
  int Parameters::scale_method = 0;

  // Debug
  bool Parameters::debug_show_find_holes = false;
  int Parameters::debug_show_find_holes_size = 1000;

  bool Parameters::debug_show_produce_edges = false;
  int Parameters::debug_show_produce_edges_size = 900;

  bool Parameters::debug_show_denoise_edges = false;
  int Parameters::debug_show_denoise_edges_size = 900;

  bool Parameters::debug_show_connect_pairs = false;
  int Parameters::debug_show_connect_pairs_size = 1200;
  bool Parameters::debug_print_connect_pairs = false;

  bool Parameters::debug_show_get_shapes_clear_border = false;
  int Parameters::debug_show_get_shapes_clear_border_size  = 1200;

  bool Parameters::debug_show_check_holes = false;
  int Parameters::debug_show_check_holes_size = 1200;

  bool Parameters::debug_show_merge_holes = false;
  int Parameters::debug_show_merge_holes_size = 1200;


  // Texture parameters
  // The threshold for texture matching
  float Parameters::match_texture_threshold = 0.5;

  //Color homogeneity parameters
  int Parameters::num_bins_threshold = 10;
  int Parameters::non_zero_points_in_box_blob_histogram = 0;

  // Histogram parameters
  int Parameters::number_of_hue_bins = 30;
  int Parameters::number_of_saturation_bins = 32;
  int Parameters::number_of_value_bins = 32;
  int Parameters::secondary_channel = 2;

  // RGB image segmentation parameters
  int Parameters::rgb_edges_extraction_method = 1;
  int Parameters::compute_edges_backprojection_threshold = 128;
  int Parameters::spatial_window_radius = 4;
  int Parameters::color_window_radius = 40;
  int Parameters::maximum_level_pyramid_segmentation = 2;
  int Parameters::segmentation_blur_method = 0;
  int Parameters::floodfill_lower_colour_difference = 2;
  int Parameters::floodfill_upper_colour_difference = 3;
  int Parameters::watershed_foreground_dilation_factor = 1;
  int Parameters::watershed_foreground_erosion_factor = 1;
  int Parameters::watershed_background_dilation_factor = 1;
  int Parameters::watershed_background_erosion_factor = 1;
  bool Parameters::posterize_after_segmentation = false;

  // Holes connection - merger
  float Parameters::connect_holes_min_distance = 0.1;
  float Parameters::connect_holes_max_distance = 0.2;


  // Holes validity thresholds
  // Normal : when depth analysis is applicable
  float Parameters::holes_validity_threshold_normal = 0.84;

  // Urgent : when depth analysis is not applicable, we can only rely
  // on RGB analysis
  float Parameters::holes_validity_threshold_urgent = 0.6;

  // The depth sensor's horizontal field of view
  float Parameters::horizontal_field_of_view = 57;

  // The depth sensor's vertical field of view
  float Parameters::vertical_field_of_view = 43;

} // namespace pandora_vision
