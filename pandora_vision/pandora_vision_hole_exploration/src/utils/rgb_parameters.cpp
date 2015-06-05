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

#include "utils/rgb_parameters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  RgbParametersHandler::RgbParametersHandler()
  {
    serverRgb.setCallback(boost::bind(&RgbParametersHandler::parametersCallbackRgb, this, _1, _2)); }

  /**
    @brief The function called when a parameter is changed
    @param[in] configRgb [const pandora_vision_hole::rgb_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void RgbParametersHandler::parametersCallbackRgb(
      const pandora_vision_hole_exploration::rgb_cfgConfig& configRgb,
      const uint32_t& level)
  {

    //////////////////////////////// Debug parameters ////////////////////////////

    //// Show the rgb image that arrives in the rgb node
    Rgb::show_rgb_image =
      configRgb.show_rgb_image;

    Rgb::show_std_variance_image =
      configRgb.show_std_variance_image;

    Rgb::show_find_holes =
      configRgb.show_find_holes;
    Rgb::show_find_holes_size =
      configRgb.show_find_holes_size;

    ////////////////////// Parameters specific to the RGB node ///////////////////

    // Std variance, morphology extraction, holes validation thresholds, holes merging thresholds.
    Rgb::original_image_gaussian_blur =
      configRgb.original_image_gaussian_blur;
    Rgb::std_variance_kernel_size =
      configRgb.std_variance_kernel_size;
    Rgb::std_variance_threshold =
      configRgb.std_variance_threshold;
    Rgb::std_variance_morphology_close_size =
      configRgb.std_variance_morphology_close_size;
    Rgb::std_variance_morphology_open_size =
      configRgb.std_variance_morphology_open_size;
    Rgb::contour_erode_kernel_size =
      configRgb.contour_erode_kernel_size;
    Rgb::lower_contour_number_to_test_huge =
      configRgb.lower_contour_number_to_test_huge;
    Rgb::huge_contour_thresh =
      configRgb.huge_contour_thresh;
    Rgb::tiny_contour_thresh =
      configRgb.tiny_contour_thresh;
    Rgb::border_thresh =
      configRgb.border_thresh;
    Rgb::small_contour_thresh =
      configRgb.small_contour_thresh;
    Rgb::neighbor_thresh =
      configRgb.neighbor_thresh;
    Rgb::homog_rect_dims_thresh =
      configRgb.homog_rect_dims_thresh;
    Rgb::neighbor_value_thresh =
      configRgb.neighbor_value_thresh;
    Rgb::homogenity_thresh =
      configRgb.homogenity_thresh;
    Rgb::neighbor_tiny_distance_thresh =
      configRgb.neighbor_tiny_distance_thresh;
    Rgb::shape_validation =
      configRgb.shape_validation;
    Rgb::one_direction_rectangle_contour_overlap_thresh =
      configRgb.one_direction_rectangle_contour_overlap_thresh;
    //Rgb::max_intersections_thresh =
    //  configRgb.max_intersections_thresh;
    Rgb::intersections_mean_cost =
      configRgb.intersections_mean_cost;
    Rgb::unclosed_contour_punishment =
      configRgb.unclosed_contour_punishment;
    Rgb::intersections_stddev_cost =
      configRgb.intersections_stddev_cost;
    Rgb::internal_pixels_2d_mean_cost =
      configRgb.internal_pixels_2d_mean_cost;
    Rgb::internal_pixels_2d_stddev_cost =
      configRgb.internal_pixels_2d_mean_cost;
    Rgb::shape_validity_thresh =
      configRgb.shape_validity_thresh;
  }


  // Show the rgb image that arrives in the rgb node
  bool Rgb::show_rgb_image = false;
  // Show the std variance image after processing rgb image
  bool Rgb::show_std_variance_image = false;
  //
  //  // Show the texture's watersheded backprojection
  //  bool Rgb::show_texture = false;
  //
  bool Rgb::show_find_holes = false;
  int Rgb::show_find_holes_size = 1000;



  int Rgb::original_image_gaussian_blur = 4;
  int Rgb::std_variance_kernel_size = 5;
  int Rgb::std_variance_threshold = 32;
  int Rgb::std_variance_morphology_close_size = 4; 
  int Rgb::std_variance_morphology_open_size = 8;
  int Rgb::contour_erode_kernel_size = 8;
  int Rgb::lower_contour_number_to_test_huge = 2;
  int Rgb::huge_contour_thresh = 40000;
  int Rgb::tiny_contour_thresh = 500;
  int Rgb::border_thresh = 10;
  int Rgb::small_contour_thresh = 100; 
  int Rgb::neighbor_thresh = 50;
  int Rgb::homog_rect_dims_thresh = 50;
  int Rgb::neighbor_value_thresh = 50;
  float Rgb::homogenity_thresh = 0.5;
  int Rgb::neighbor_tiny_distance_thresh = 50;
  int Rgb::rect_diff_thresh = 3;
  int Rgb::shape_validation = 1;
  float Rgb::one_direction_rectangle_contour_overlap_thresh = 40.0;
  int Rgb::max_intersections_thresh = 4;
  float Rgb::intersections_mean_cost = 0.3;
  float Rgb::unclosed_contour_punishment = 1.0;
  float Rgb::intersections_stddev_cost = 0.3;
  float Rgb::internal_pixels_2d_mean_cost = 0.2;
  float Rgb::internal_pixels_2d_stddev_cost = 0.2;
  float Rgb::shape_validity_thresh = 1.2;

}  // namespace pandora_vision
