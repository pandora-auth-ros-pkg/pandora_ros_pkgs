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

#include "utils/parameters.h"

namespace pandora_vision
{
  //!< Kanny parameters
  int Parameters::kanny_ratio = 3;
  int Parameters::kanny_kernel_size = 3;
  int Parameters::kanny_low_threshold = 50;
  int Parameters::kanny_blur_noise_kernel_size = 3;

  float Parameters::contrast_enhance_alpha = 2;
  float Parameters::contrast_enhance_beta = 2;


  //!< Threshold parameters
  int Parameters::threshold_lower_value = 10;

  //!< Blob detection parameters
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

  //!< Bounding boxes parameters
  int Parameters::bounding_box_min_area_threshold = 550;

  //!< The bounding box detection method
  //!< 0 for detecting by means of brushfire starting
  //!< from the keypoint of the blob
  //!< 1 for detecting by means of contours around the edges of the blob
  int Parameters::bounding_box_detection_method = 0;

  //!< When using raycast instead of brushfire to find the (approximate here)
  //!< outline of blobs, raycast_keypoint_partitions dictates the number of
  //!< rays, or equivalently, the number of partitions in which the blob is
  //!< partitioned in search of the blob's borders
  int Parameters::raycast_keypoint_partitions = 8;

  //<! Loose ends connection parameters
  int Parameters::AB_to_MO_ratio = 2;
  int Parameters::minimum_curve_points = 1200;


  ////!< Interpolation parameters

  //!< The interpolation method for noise removal
  //!< 0 for averaging the pixel's neighbor values
  //!< 1 for brushfire near
  //!< 2 for brushfire far
  int Parameters::interpolation_method = 0;

  //!< Hole checkers
  int Parameters::run_checker_depth_diff = 1;
  int Parameters::run_checker_depth_area = 3;
  int Parameters::run_checker_brushfire_outline_to_rectangle = 4;
  int Parameters::run_checker_outline_of_rectangle = 2;
  int Parameters::run_checker_depth_homogenity = 5;
  int Parameters::rectangle_inflation_size = 20;
  float Parameters::depth_difference = 0.4;

  int Parameters::run_checker_color_homogenity = 1;
  int Parameters::run_checker_luminosity_diff = 2;
  int Parameters::run_checker_texture_diff = 3;
  int Parameters::run_checker_texture_backproject = 4;

  //!< Plane detection
  int Parameters::segmentation_method = 0;
  int Parameters::max_iterations = 1000;
  double Parameters::num_points_to_exclude = 0.1;
  double Parameters::point_to_plane_distance_threshold = 0.01;

  //!< Method to scale the CV_32FC1 image to CV_8UC1
  int Parameters::scale_method = 0;

  //!< Debug
  bool Parameters::debug_show_find_holes = false;
  int Parameters::debug_show_find_holes_size = 1000;

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


  //!< Texture parameters
  //!< The threshold for texture matching
  float Parameters::match_texture_threshold = 0.5;

  //!<Color homogenity parameters
  int Parameters::num_bins_threshold = 10;
  int Parameters::non_zero_points_in_box_blob_histogram = 0;

  //!< Histogram parameters
  int Parameters::number_of_hue_bins = 30;
  int Parameters::number_of_saturation_bins = 32;
  int Parameters::number_of_value_bins = 30;

  //!< Holes connection - merger
  float Parameters::connect_holes_min_distance = 0.1;

} // namespace pandora_vision
