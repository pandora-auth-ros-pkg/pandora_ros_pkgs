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

#include "utils/depth_parameters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  DepthParametersHandler::DepthParametersHandler()
  {
    serverDepth.setCallback(boost::bind(&DepthParametersHandler::parametersCallbackDepth, this, _1, _2));
  }


  /**
    @brief The function called when a parameter is changed
    @param[in] configDepth [const pandora_vision_hole::depth_cfgConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void DepthParametersHandler::parametersCallbackDepth(
      const pandora_vision_hole_exploration::depth_cfgConfig& configDepth,
      const uint32_t& level)
  {
    //  //////////////////////////////// Debug parameters ////////////////////////////

    // Show the depth image that arrives in the depth node
    Depth::show_depth_image =
      configDepth.show_depth_image;

    Depth::show_find_holes =
      configDepth.show_find_holes;
    Depth::show_find_holes_size =
      configDepth.show_find_holes_size;

    Depth::intensity_threshold =
      configDepth.intensity_threshold;
    Depth::morphology_open_kernel_size =
      configDepth.morphology_open_kernel_size;
    Depth::morphology_close_kernel_size =
      configDepth.morphology_close_kernel_size;
    Depth::border_thresh =
      configDepth.border_thresh;
    Depth::dilation_kernel_size =
      configDepth.dilation_kernel_size;
    Depth::rect_diff_thresh =
      configDepth.rect_diff_thresh;
    Depth::huge_contour_thresh =
      configDepth.huge_contour_thresh;
    Depth::tiny_contour_thresh =
      configDepth.tiny_contour_thresh;
    Depth::small_contour_thresh =
      configDepth.small_contour_thresh;
    Depth::neighbor_thresh =
      configDepth.neighbor_thresh;
    Depth::neighbor_value_thresh =
      configDepth.neighbor_value_thresh;
    Depth::depth_similarity_rect_dims_thresh =
      configDepth.depth_similarity_rect_dims_thresh;
    Depth::merge_thresh =
      configDepth.merge_thresh;
    Depth::canny_low_threshold =
      configDepth.canny_low_threshold;
    Depth::canny_ratio =
      configDepth.canny_ratio;
    Depth::canny_kernel_size =
      configDepth.canny_kernel_size;
    Depth::filtering_type =
      configDepth.filtering_type;
    Depth::min_valid_depth =
      configDepth.min_valid_depth;
    Depth::shape_validation =
      configDepth.shape_validation;
    Depth::one_direction_rectangle_contour_overlap_thresh =
      configDepth.one_direction_rectangle_contour_overlap_thresh;
    //Depth::max_intersections_thresh =
    //  configDepth.max_intersections_thresh;
    Depth::intersections_mean_cost =
      configDepth.intersections_mean_cost;
    Depth::unclosed_contour_punishment =
      configDepth.unclosed_contour_punishment;
    Depth::intersections_stddev_cost =
      configDepth.intersections_stddev_cost;
    Depth::internal_pixels_2d_mean_cost =
      configDepth.internal_pixels_2d_mean_cost;
    Depth::internal_pixels_2d_stddev_cost =
      configDepth.internal_pixels_2d_mean_cost;
    Depth::shape_validity_thresh =
      configDepth.shape_validity_thresh;
    Depth::noise_percent_thresh =
      configDepth.noise_percent_thresh;
  }


  // Show the depth image that arrives in the depth node
  bool Depth::show_depth_image = false;
  //
  //  // Show the texture's watersheded backprojection
  //  bool Depth::show_texture = false;
  //
  bool Depth::show_find_holes = false;
  int Depth::show_find_holes_size = 1000;


  float Depth::intensity_threshold = 0.1;
  int Depth::morphology_open_kernel_size = 2;
  int Depth::morphology_close_kernel_size = 12;
  int Depth::border_thresh = 20;
  int Depth::dilation_kernel_size = 12;
  int Depth::rect_diff_thresh = 3;
  int Depth::huge_contour_thresh = 40000;
  int Depth::tiny_contour_thresh = 800;
  int Depth::small_contour_thresh = 2500;
  int Depth::neighbor_thresh = 50;
  int Depth::neighbor_value_thresh = 30;
  int Depth::depth_similarity_rect_dims_thresh = 50;
  float Depth::merge_thresh = 50.0;
  int Depth::canny_low_threshold = 50;
  int Depth::canny_ratio = 3;
  int Depth::canny_kernel_size = 3;
  int Depth::filtering_type = 1;
  float Depth::min_valid_depth = 0.5;
  int Depth::shape_validation = 1;
  float Depth::one_direction_rectangle_contour_overlap_thresh = 40.0;
  int Depth::max_intersections_thresh = 4;
  float Depth::intersections_mean_cost = 0.3;
  float Depth::unclosed_contour_punishment = 1.0;
  float Depth::intersections_stddev_cost = 0.3;
  float Depth::internal_pixels_2d_mean_cost = 0.2;
  float Depth::internal_pixels_2d_stddev_cost = 0.2;
  float Depth::shape_validity_thresh = 1.2;
  float Depth::noise_percent_thresh = 0.1;

}  // namespace pandora_vision
