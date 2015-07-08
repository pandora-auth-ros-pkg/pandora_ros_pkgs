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

#include "thermal_node/utils/parameters.h"

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
  //////////////////// Blob detection - specific parameters ////////////////////

  int Parameters::Blob::min_threshold = 0;
  int Parameters::Blob::max_threshold = 200;
  int Parameters::Blob::threshold_step = 100;
  int Parameters::Blob::min_area = 550;
  int Parameters::Blob::max_area = 300000;
  double Parameters::Blob::min_convexity = 0;
  double Parameters::Blob::max_convexity = 100;
  double Parameters::Blob::min_inertia_ratio = 0;
  double Parameters::Blob::max_circularity = 1.0;
  double Parameters::Blob::min_circularity = 0.3;
  bool Parameters::Blob::filter_by_color = 0;
  bool Parameters::Blob::filter_by_circularity = 1;


  ///////////////////////// Debug-specific parameters //////////////////////////

  // Publish the enhanced Images
  bool Parameters::Debug::publish_enhanced_Images = false;

  // Show the depth image that arrives in the depth node
  bool Parameters::Debug::show_depth_image = false;

  // Show the thermal image that arrives in the thermal node
  bool Parameters::Debug::show_thermal_image = false;

  // Show the rgb image that arrives in the rgb node
  bool Parameters::Debug::show_rgb_image = false;

  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool Parameters::Debug::show_respective_holes = false;

  // Show all valid holes, from either the Depth or RGB source, or
  // the merges between them
  bool Parameters::Debug::show_valid_holes = false;

  // The product of this package: unique, valid holes
  bool Parameters::Debug::show_final_holes = false;

  // In the terminal's window, show the probabilities of candidate holes
  bool Parameters::Debug::show_probabilities = false;

  // Show the texture's watersheded backprojection
  bool Parameters::Debug::show_texture = false;

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


  /////////////////// Parameters specific to the Depth node ////////////////////

  // The interpolation method for noise removal
  // 0 for averaging the pixel's neighbor values
  // 1 for brushfire near
  // 2 for brushfire far
  int Parameters::Depth::interpolation_method = 0;

  ////////////////// Parameters pecific to the Thermal node ////////////////////

  // The thermal detection method
  // If set to 0 process the binary image acquired from temperatures MultiArray
  // If set to 1 process the sensor/Image from thermal sensor
  int Parameters::Thermal::detection_method = 0;

  // The probability extraction method
  // 0 for Gaussian function
  // 1 for Logistic function

  int Parameters::Thermal::probability_method = 1;
  float Parameters::Thermal::min_thermal_probability = 0.3;

  // Gausian variables
  float Parameters::Thermal::optimal_temperature = 35;
  float Parameters::Thermal::tolerance = 10;

  // Logistic variables
  float Parameters::Thermal::low_acceptable_temperature = 32;
  float Parameters::Thermal::high_acceptable_temperature = 38;

  float Parameters::Thermal::left_tolerance = 4;
  float Parameters::Thermal::right_tolerance = 8;

  ///////////// Parameters of acceptable temperature for threshold /////////////
  float Parameters::Thermal::low_temperature = 28;
  float Parameters::Thermal::high_temperature = 40;

  ////////////////////// Parameters of the thermal image ///////////////////////
  int Parameters::ThermalImage::WIDTH = 80;
  int Parameters::ThermalImage::HEIGHT =60;

  ///////////////////// Edge detection specific parameters /////////////////////

  // canny parameters
  int Parameters::Edge::canny_ratio = 3;
  int Parameters::Edge::canny_kernel_size = 3;
  int Parameters::Edge::canny_low_threshold = 50;
  int Parameters::Edge::canny_blur_noise_kernel_size = 3;

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


  ///////////////////////  Histogram related parameters ////////////////////////

  int Parameters::Histogram::number_of_hue_bins = 30;
  int Parameters::Histogram::number_of_saturation_bins = 32;
  int Parameters::Histogram::number_of_value_bins = 32;
  int Parameters::Histogram::secondary_channel = 2;



  ////////////////////////// Filters-related parameters ////////////////////////

  // DepthDiff
  int Parameters::Filters::DepthDiff::priority = 5;
  float Parameters::Filters::DepthDiff::threshold = 0.7;

  // 0 for binary probability assignment on positive depth difference
  // 1 for gaussian probability assignment on positive depth difference
  int Parameters::Filters::DepthDiff::probability_assignment_method = 1;

  // The mean stardard deviation for the normal distribution
  // incorporated in the depth diff filter.
  float Parameters::Filters::DepthDiff::gaussian_mean = 0.15;

  float Parameters::Filters::DepthDiff::gaussian_stddev = 0.2;

  // Min difference in depth between the inside and the outside of a hole
  float Parameters::Filters::DepthDiff::min_depth_cutoff = 0.02;

  // Max difference in depth between the inside and the outside of a hole
  float Parameters::Filters::DepthDiff::max_depth_cutoff = 0.5;


  // DepthArea
  int Parameters::Filters::DepthArea::priority = 5;
  float Parameters::Filters::DepthArea::threshold = 0.7;

  // DepthHomogeneity
  int Parameters::Filters::DepthHomogeneity::priority = 5;
  float Parameters::Filters::DepthHomogeneity::threshold = 0.7;

  // RectanglePlaneConstitution
  int Parameters::Filters::RectanglePlaneConstitution::priority = 5;
  float Parameters::Filters::RectanglePlaneConstitution::threshold = 0.7;

  // IntermediatePointsPlaneConstitution
  int Parameters::Filters::IntermediatePointsPlaneConstitution::priority = 5;
  float Parameters::Filters::IntermediatePointsPlaneConstitution::threshold = 0.7;

  // ColourHomogeneity
  int Parameters::Filters::ColourHomogeneity::rgbd_priority = 5;
  int Parameters::Filters::ColourHomogeneity::rgb_priority = 5;

  float Parameters::Filters::ColourHomogeneity::rgbd_threshold = 0.7;
  float Parameters::Filters::ColourHomogeneity::rgb_threshold = 0.7;

  // LuminosityDiff
  int Parameters::Filters::LuminosityDiff::rgbd_priority = 5;
  int Parameters::Filters::LuminosityDiff::rgb_priority = 5;

  float Parameters::Filters::LuminosityDiff::rgbd_threshold = 0.7;
  float Parameters::Filters::LuminosityDiff::rgb_threshold = 0.7;

  // TextureDiff
  int Parameters::Filters::TextureDiff::rgbd_priority = 5;
  int Parameters::Filters::TextureDiff::rgb_priority = 5;

  float Parameters::Filters::TextureDiff::rgbd_threshold = 0.7;
  float Parameters::Filters::TextureDiff::rgb_threshold = 0.7;

  // The threshold for texture matching regarding the intermediate points
  float Parameters::Filters::TextureDiff::match_texture_threshold = 0.85;

  // The threshold for texture matching reagrding the points inside the hole
  float Parameters::Filters::TextureDiff::mismatch_texture_threshold = 0.8;

  // TextureBackprojection
  int Parameters::Filters::TextureBackprojection::rgbd_priority = 5;
  int Parameters::Filters::TextureBackprojection::rgb_priority = 5;

  float Parameters::Filters::TextureBackprojection::rgbd_threshold = 0.7;
  float Parameters::Filters::TextureBackprojection::rgb_threshold = 0.7;


  /////////////////////// HoleFusion-specific parameters ///////////////////////

  //-------------------------------- Validation --------------------------------

  // The holes' validation process identifier
  int Parameters::HoleFusion::Validation::validation_process = 0;

  // When depth analysis is applicable
  float Parameters::HoleFusion::Validation::rgbd_validity_threshold= 0.54;

  // When depth analysis is not applicable, we can only rely
  // on RGB analysis
  float Parameters::HoleFusion::Validation::rgb_validity_threshold = 0.40;


  // Plane detection parameters
  float Parameters::HoleFusion::Planes::filter_leaf_size = 0.1;
  int Parameters::HoleFusion::Planes::max_iterations = 1000;
  double Parameters::HoleFusion::Planes::num_points_to_exclude = 0.2;
  double Parameters::HoleFusion::Planes::point_to_plane_distance_threshold = 0.08;

  // Option to enable or disable the merging of holes
  bool Parameters::HoleFusion::Merger::merge_holes = true;

  // Holes connection - merger
  float Parameters::HoleFusion::Merger::connect_holes_min_distance = 0.1;
  float Parameters::HoleFusion::Merger::connect_holes_max_distance = 0.2;

  // Merger parameters
  float Parameters::HoleFusion::Merger::depth_diff_threshold = 0.3;
  float Parameters::HoleFusion::Merger::depth_area_threshold = 1.0;

  // The inflation size of holes' bounding rectangles
  int Parameters::HoleFusion::rectangle_inflation_size = 10;



  ////////////////// Image representation specific parameters //////////////////

  // The depth sensor's horizontal field of view in rads
  float Parameters::Image::horizontal_field_of_view =
    static_cast<float>(58) / 180 * M_PI;

  // The depth sensor's vertical field of view in rads
  float Parameters::Image::vertical_field_of_view =
    static_cast<float>(45) / 180 * M_PI;

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


  /////////////////// Outline discovery specific parameters ////////////////////

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


  //////////////////// Parameters specific to the RGB node /////////////////////

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

}  // namespace thermal
}  // namespace pandora_vision_hole
}  // namespace pandora_vision
