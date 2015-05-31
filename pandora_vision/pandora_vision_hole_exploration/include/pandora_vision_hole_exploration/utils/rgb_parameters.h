/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Vasilis Bosdelekidis
 *********************************************************************/

#ifndef UTILS_RGB_PARAMETERS_H
#define UTILS_RGB_PARAMETERS_H

//#include "utils/defines.h"
#include "utils/parameters.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_vision_hole_exploration/rgb_cfgConfig.h>

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  class RgbParametersHandler
  {
    public:
      //the constructor
      RgbParametersHandler();

    private:
      // The dynamic reconfigure (RGB) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::rgb_cfgConfig> serverRgb;

      // The dynamic reconfigure (RGB) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::rgb_cfgConfig>::CallbackType fRgb;
      
      /**
        @brief The function called when a parameter is changed
        @param[in] configRgb [const pandora_vision_hole::rgb_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallbackRgb(
          const pandora_vision_hole_exploration::rgb_cfgConfig& configRgb,
          const uint32_t& level);
  };
  
  //! Parameters specific to the RGB node
  struct Rgb
  {
    static int original_image_gaussian_blur;
    static int std_variance_kernel_size;
    static int std_variance_threshold;
    static int std_variance_morphology_close_size;
    static int std_variance_morphology_open_size;
    static int contour_erode_kernel_size;
    static int lower_contour_number_to_test_huge;
    static int huge_contour_thresh;
    static int tiny_contour_thresh;
    static int border_thresh;
    static int small_contour_thresh;
    static int neighbor_thresh;
    static int homog_rect_dims_thresh;
    static int neighbor_value_thresh;
    static float homogenity_thresh;
    static int neighbor_tiny_distance_thresh;
    static int rect_diff_thresh;
    static int shape_validation;
    static float one_direction_rectangle_contour_overlap_thresh;
    static int max_intersections_thresh;
    static float intersections_mean_cost;
    static float unclosed_contour_punishment;
    static float intersections_stddev_cost;
    static float internal_pixels_2d_mean_cost;
    static float internal_pixels_2d_stddev_cost;
    static float shape_validity_thresh;
    
    //Show the rgb image that arrives in the rgb node
    static bool show_rgb_image;
    //Show the std variance image after rgb image processing
    static bool show_std_variance_image;
    static bool show_find_holes;
    static int show_find_holes_size;
  };

}  // namespace pandora_vision

#endif  // UTILS_RGB_PARAMETERS_H

