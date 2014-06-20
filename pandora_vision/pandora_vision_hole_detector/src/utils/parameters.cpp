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

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  // Blob detection - specific parameters
  int Parameters::Blob::blob_min_threshold = 0;
  int Parameters::Blob::blob_max_threshold = 1255;
  int Parameters::Blob::blob_threshold_step = 5;
  int Parameters::Blob::blob_min_area = 550;
  int Parameters::Blob::blob_max_area = 300000;
  double Parameters::Blob::blob_min_convexity = 0;
  double Parameters::Blob::blob_max_convexity = 100;
  double Parameters::Blob::blob_min_inertia_ratio = 0;
  double Parameters::Blob::blob_max_circularity = 1.0;
  double Parameters::Blob::blob_min_circularity = 0.3;
  bool Parameters::Blob::blob_filter_by_color = 0;
  bool Parameters::Blob::blob_filter_by_circularity = 1;



  // Debug-specific parameters
  bool Parameters::Debug::show_find_holes = false;
  int Parameters::Debug::show_find_holes_size = 1000;

  bool Parameters::Debug::show_produce_edges = false;
  int Parameters::Debug::show_produce_edges_size = 900;

  bool Parameters::Debug::show_denoise_edges = false;
  int Parameters::Debug::show_denoise_edges_size = 900;

  bool Parameters::Debug::show_connect_pairs = false;
  int Parameters::Debug::show_connect_pairs_size = 1200;
  bool Parameters::Debug::print_connect_pairs = false;

  bool Parameters::Debug::show_get_shapes_clear_border = false;
  int Parameters::Debug::show_get_shapes_clear_border_size  = 1200;

  bool Parameters::Debug::show_check_holes = false;
  int Parameters::Debug::show_check_holes_size = 1200;

  bool Parameters::Debug::show_merge_holes = false;
  int Parameters::Debug::show_merge_holes_size = 1200;



  // Parameters specific to the Depth node

  // Show the depth image that arrives in the depth node
  bool Parameters::Depth::show_depth_image = false;

  // The interpolation method for noise removal
  // 0 for averaging the pixel's neighbor values
  // 1 for brushfire near
  // 2 for brushfire far
  int Parameters::Depth::interpolation_method = 0;



  // Edge detection specific parameters

  // canny parameters
  int Parameters::Edge::canny_ratio = 3;
  int Parameters::Edge::canny_kernel_size = 3;
  int Parameters::Edge::canny_low_threshold = 50;
  int Parameters::Edge::canny_blur_noise_kernel_size = 3;

  float Parameters::Edge::contrast_enhance_alpha = 2;
  float Parameters::Edge::contrast_enhance_beta = 2;

  // The opencv edge detection method:
  // 0 for the Canny edge detector
  // 1 for the Scharr edge detector
  // 2 for the Sobel edge detector
  // 3 for the Laplacian edge detector
  // 4 for mixed Scharr / Sobel edge detection
  int Parameters::Edge::edge_detection_method = 2;

  // Threshold parameters
  int Parameters::Edge::denoised_edges_threshold = 10;

  // When mixed edge detection is selected, this toggle switch
  // is needed in order to shift execution from one edge detector
  // to the other.
  // 1 for the Scharr edge detector,
  // 2 for the Sobel edge detector
  int Parameters::Edge::mixed_edges_toggle_switch = 1;



  // Histogram related parameters
  int Parameters::Histogram::number_of_hue_bins = 30;
  int Parameters::Histogram::number_of_saturation_bins = 32;
  int Parameters::Histogram::number_of_value_bins = 32;
  int Parameters::Histogram::secondary_channel = 2;



  // HoleFusion-specific parameters

  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool Parameters::HoleFusion::show_respective_holes = false;

  // The product of this package: valid holes
  bool Parameters::HoleFusion::show_final_holes = false;

  // Hole checkers and their thresholds`
  int Parameters::HoleFusion::run_checker_depth_diff = 1;
  float Parameters::HoleFusion::checker_depth_diff_threshold = 0.7;

  int Parameters::HoleFusion::run_checker_depth_area = 3;
  float Parameters::HoleFusion::checker_depth_area_threshold = 0.8;

  int Parameters::HoleFusion::run_checker_brushfire_outline_to_rectangle = 4;
  float Parameters::HoleFusion::checker_brushfire_outline_to_rectangle_threshold = 0.75;

  int Parameters::HoleFusion::run_checker_outline_of_rectangle = 2;
  float Parameters::HoleFusion::checker_outline_of_rectangle_threshold = 0.75;

  int Parameters::HoleFusion::run_checker_depth_homogeneity = 5;
  float Parameters::HoleFusion::checker_depth_homogeneity_threshold = 0.2;

  int Parameters::HoleFusion::rectangle_inflation_size = 20;

  float Parameters::HoleFusion::holes_gaussian_mean = 0.3;
  float Parameters::HoleFusion::holes_gaussian_stddev = 0.2;

  int Parameters::HoleFusion::run_checker_color_homogeneity = 1;
  int Parameters::HoleFusion::run_checker_color_homogeneity_urgent = 1;
  float Parameters::HoleFusion::checker_color_homogeneity_threshold = 0.4;

  int Parameters::HoleFusion::run_checker_luminosity_diff = 2;
  int Parameters::HoleFusion::run_checker_luminosity_diff_urgent = 2;
  float Parameters::HoleFusion::checker_luminosity_diff_threshold = 0.4;

  int Parameters::HoleFusion::run_checker_texture_diff = 3;
  int Parameters::HoleFusion::run_checker_texture_diff_urgent = 3;
  float Parameters::HoleFusion::checker_texture_diff_threshold = 0.4;

  int Parameters::HoleFusion::run_checker_texture_backproject = 4;
  int Parameters::HoleFusion::run_checker_texture_backproject_urgent = 4;
  float Parameters::HoleFusion::checker_texture_backproject_threshold = 0.4;

  // 0 for binary probability assignment on positive depth difference
  // 1 for gaussian probability assignment on positive depth difference
  int Parameters::HoleFusion::depth_difference_probability_assignment_method = 1;

  // Plane detection
  float Parameters::HoleFusion::filter_leaf_size = 0.1;
  int Parameters::HoleFusion::max_iterations = 1000;
  double Parameters::HoleFusion::num_points_to_exclude = 0.2;
  double Parameters::HoleFusion::point_to_plane_distance_threshold = 0.08;

  // Holes connection - merger
  float Parameters::HoleFusion::connect_holes_min_distance = 0.1;
  float Parameters::HoleFusion::connect_holes_max_distance = 0.2;

  // The threshold for texture matching
  float Parameters::HoleFusion::match_texture_threshold = 0.8;

  // Color homogeneity parameters
  int Parameters::HoleFusion::num_bins_threshold = 10;
  int Parameters::HoleFusion::non_zero_points_in_box_blob_histogram = 0;

  // Merger parameters
  float Parameters::HoleFusion::merger_depth_diff_threshold = 0.6;
  float Parameters::HoleFusion::merger_depth_area_threshold = 1.0;

  // Holes validity thresholds
  // Normal : when depth analysis is applicable
  float Parameters::HoleFusion::holes_validity_threshold_normal = 0.94;

  // Urgent : when depth analysis is not applicable, we can only rely
  // on RGB analysis
  float Parameters::HoleFusion::holes_validity_threshold_urgent = 0.70;

  // The depth sensor's horizontal field of view in rads
  float Parameters::HoleFusion::horizontal_field_of_view =
    static_cast<float>(58) / 180 * M_PI;

  // The depth sensor's vertical field of view in rads
  float Parameters::HoleFusion::vertical_field_of_view =
    static_cast<float>(45) / 180 * M_PI;



  // Image representation specific parameters

  // Fallback values. See the input point cloud callback of the
  // synchronizer node
  int Parameters::Image::HEIGHT = 480;
  int Parameters::Image::WIDTH = 640;

  // Depth and RGB images' representation method.
  // 0 if image used is used as obtained from the image sensor
  // 1 through wavelet analysis
  int Parameters::Image::image_representation_method = 1;

  // Method to scale the CV_32F image to CV_8UC1
  int Parameters::Image::scale_method = 0;

  // Term criteria for segmentation purposes
  int Parameters::Image::term_criteria_max_iterations = 5;
  double Parameters::Image::term_criteria_max_epsilon = 1;


  // Outline discovery specific parameters

  // The detection method used to obtain the outline of a blob
  // 0 for detecting by means of brushfire
  // 1 for detecting by means of raycasting
  int Parameters::Outline::outline_detection_method = 0;

  // When using raycast instead of brushfire to find the (approximate here)
  // outline of blobs, raycast_keypoint_partitions dictates the number of
  // rays, or equivalently, the number of partitions in which the blob is
  // partitioned in search of the blob's borders
  int Parameters::Outline::raycast_keypoint_partitions = 32;

  // Loose ends connection parameters
  int Parameters::Outline::AB_to_MO_ratio = 4;
  int Parameters::Outline::minimum_curve_points = 50;


  // Parameters specific to the RGB node

  // Show the rgb image that arrives in the rgb node
  bool Parameters::Rgb::show_rgb_image = false;


  // RGB image segmentation parameters

  // Selects the method for extracting a RGB image's edges.
  // Choices are via segmentation and via backprojection
  int Parameters::Rgb::edges_extraction_method = 0;

  // The threshold applied to the backprojection of the RGB image
  // captured by the image sensor
  int Parameters::Rgb::backprojection_threshold = 128;

  // Parameters specific to the pyrMeanShiftFiltering method
  int Parameters::Rgb::spatial_window_radius = 13;
  int Parameters::Rgb::color_window_radius = 40;
  int Parameters::Rgb::maximum_level_pyramid_segmentation = 2;

  // True to posterize the product of the segmentation
  bool Parameters::Rgb::posterize_after_segmentation = false;

  // FloodFill options regarding minimum and maximum colour difference
  int Parameters::Rgb::floodfill_lower_colour_difference = 2;
  int Parameters::Rgb::floodfill_upper_colour_difference = 3;

  // Watershed-specific parameters
  int Parameters::Rgb::watershed_foreground_dilation_factor = 1;
  int Parameters::Rgb::watershed_foreground_erosion_factor = 1;
  int Parameters::Rgb::watershed_background_dilation_factor = 1;
  int Parameters::Rgb::watershed_background_erosion_factor = 1;

} // namespace pandora_vision
