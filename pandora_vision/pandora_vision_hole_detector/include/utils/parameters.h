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

#ifndef UTILS_PARAMETERS_H
#define UTILS_PARAMETERS_H

#include "utils/defines.h"
#include "utils/parameters.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_vision_hole_detector/depth_cfgConfig.h>
#include <pandora_vision_hole_detector/hole_fusion_cfgConfig.h>
#include <pandora_vision_hole_detector/rgb_cfgConfig.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @struct Parameters
    @brief Provides flexibility by parameterizing variables needed by the
    hole detector package
   **/
  struct Parameters
  {
    //! Blob detection - specific parameters
    struct Blob
    {
      static int blob_min_threshold;
      static int blob_max_threshold;
      static int blob_threshold_step;
      static int blob_min_area;
      static int blob_max_area;
      static double blob_min_convexity;
      static double blob_max_convexity;
      static double blob_min_inertia_ratio;
      static double blob_max_circularity;
      static double blob_min_circularity;
      static bool blob_filter_by_color;
      static bool blob_filter_by_circularity;
    };



    //! Debug-specific parameters
    struct Debug
    {
      static bool show_find_holes;
      static int show_find_holes_size;

      static bool show_produce_edges;
      static int show_produce_edges_size;

      static bool show_denoise_edges;
      static int show_denoise_edges_size;

      static bool print_connect_pairs;
      static bool show_connect_pairs;
      static int show_connect_pairs_size;

      static bool show_get_shapes_clear_border;
      static int show_get_shapes_clear_border_size;

      static bool show_check_holes;
      static int show_check_holes_size;

      static bool show_merge_holes;
      static int show_merge_holes_size;
    };



    //! Parameters specific to the Depth node
    struct Depth
    {
      // Show the depth image that arrives in the depth node
      static bool show_depth_image;

      // The interpolation method for noise removal
      // 0 for averaging the pixel's neighbor values
      // 1 for brushfire near
      // 2 for brushfire far
      static int interpolation_method;
    };



    //! Edge detection specific parameters
    struct Edge
    {
      // canny parameters
      static int canny_ratio;
      static int canny_kernel_size;
      static int canny_low_threshold;
      static int canny_blur_noise_kernel_size;

      static float contrast_enhance_alpha;
      static float contrast_enhance_beta;

      // The opencv edge detection method:
      // 0 for the Canny edge detector
      // 1 for the Scharr edge detector
      // 2 for the Sobel edge detector
      // 3 for the Laplacian edge detector
      // 4 for mixed Scharr / Sobel edge detection
      static int edge_detection_method;

      // Threshold parameters
      static int denoised_edges_threshold;

      // When mixed edge detection is selected, this toggle switch
      // is needed in order to shift execution from one edge detector
      // to the other.
      // 1 for the Scharr edge detector,
      // 2 for the Sobel edge detector
      static int mixed_edges_toggle_switch;

    };



    //! Histogram related parameters
    struct Histogram
    {
      static int number_of_hue_bins;
      static int number_of_saturation_bins;
      static int number_of_value_bins;
      static int secondary_channel;
    };



    //! Parameters specific to the Hole Fusion node
    struct HoleFusion
    {
      // Show the holes that each of the depth and RGB nodes transmit to the
      // hole fusion node, on top of their respective origin images
      static bool show_respective_holes;

      // The product of this package: valid holes
      static bool show_final_holes;

      // Hole checkers and their thresholds`
      static int run_checker_depth_diff;
      static float checker_depth_diff_threshold;

      static int run_checker_depth_area;
      static float checker_depth_area_threshold;

      static int run_checker_brushfire_outline_to_rectangle;
      static float checker_brushfire_outline_to_rectangle_threshold;

      static int run_checker_outline_of_rectangle;
      static float checker_outline_of_rectangle_threshold;

      static int run_checker_depth_homogeneity;
      static float checker_depth_homogeneity_threshold;

      static int rectangle_inflation_size;
      static float holes_gaussian_mean;
      static float holes_gaussian_stddev;

      static int run_checker_color_homogeneity;
      static int run_checker_color_homogeneity_urgent;
      static float checker_color_homogeneity_threshold;

      static int run_checker_luminosity_diff;
      static int run_checker_luminosity_diff_urgent;
      static float checker_luminosity_diff_threshold;

      static int run_checker_texture_diff;
      static int run_checker_texture_diff_urgent;
      static float checker_texture_diff_threshold;

      static int run_checker_texture_backproject;
      static int run_checker_texture_backproject_urgent;
      static float checker_texture_backproject_threshold;

      // 0 for binary probability assignment on positive depth difference
      // 1 for gaussian probability assignment on positive depth difference
      static int depth_difference_probability_assignment_method;

      // Plane detection
      static float filter_leaf_size;
      static int max_iterations;
      static double num_points_to_exclude;
      static double point_to_plane_distance_threshold;

      // Holes connection - merger
      static float connect_holes_min_distance;
      static float connect_holes_max_distance;

      // Texture parameters
      // The threshold for texture matching
      static float match_texture_threshold;

      // Color homogeneity parameters
      static int num_bins_threshold;
      static int non_zero_points_in_box_blob_histogram;

      // Merger parameters
      static float merger_depth_diff_threshold;
      static float merger_depth_area_threshold;

      // Holes validity thresholds

      // Normal : when depth analysis is applicable
      static float holes_validity_threshold_normal;

      // Urgent : when depth analysis is not applicable, we can only rely
      // on RGB analysis
      static float holes_validity_threshold_urgent;

      // The depth sensor's horizontal field of view
      static float horizontal_field_of_view;

      // The depth sensor's vertical field of view
      static float vertical_field_of_view;

    };



    //! Image representation specific parameters
    struct Image
    {
      // Fallback values. See the input point cloud callback of the
      // synchronizer node
      static int HEIGHT;
      static int WIDTH;

      // Depth and RGB images' representation method.
      // 0 if image used is used as obtained from the image sensor
      // 1 through wavelet analysis
      static int image_representation_method;

      // Method to scale the CV_32F images to CV_8UC1
      static int scale_method;

      // Term criteria for segmentation purposes
      static int term_criteria_max_iterations;
      static double term_criteria_max_epsilon;
    };



    //! Outline discovery specific parameters
    struct Outline
    {
      // The detection method used to obtain the outline of a blob
      // 0 for detecting by means of brushfire
      // 1 for detecting by means of raycasting
      static int outline_detection_method;

      // When using raycast instead of brushfire to find the (approximate here)
      // outline of blobs, raycast_keypoint_partitions dictates the number of
      // rays, or equivalently, the number of partitions in which the blob is
      // partitioned in search of the blob's borders
      static int raycast_keypoint_partitions;

      // Loose ends connection parameters
      static int AB_to_MO_ratio;
      static int minimum_curve_points;

    };



    //! Parameters specific to the RGB node
    struct Rgb
    {
      // Show the rgb image that arrives in the rgb node
      static bool show_rgb_image;

      // Selects the method for extracting a RGB image's edges.
      // Choices are via segmentation and via backprojection
      static int edges_extraction_method;

      // The threshold applied to the backprojection of the RGB image
      // captured by the image sensor
      static int backprojection_threshold;

      // Parameters specific to the pyrMeanShiftFiltering method
      static int spatial_window_radius;
      static int color_window_radius;
      static int maximum_level_pyramid_segmentation;

      // True to posterize the product of the segmentation
      static bool posterize_after_segmentation;

      // FloodFill options regarding minimum and maximum colour difference
      static int floodfill_lower_colour_difference;
      static int floodfill_upper_colour_difference;

      // Watershed-specific parameters
      static int watershed_foreground_dilation_factor;
      static int watershed_foreground_erosion_factor;
      static int watershed_background_dilation_factor;
      static int watershed_background_erosion_factor;

    };

  };

} // namespace pandora_vision

#endif  // UTILS_PARAMETERS_H
