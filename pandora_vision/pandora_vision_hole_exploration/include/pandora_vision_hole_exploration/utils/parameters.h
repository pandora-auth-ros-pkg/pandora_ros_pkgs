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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Vasilis Bosdelekidis
 *********************************************************************/

#ifndef UTILS_PARAMETERS_H
#define UTILS_PARAMETERS_H

//#include "utils/defines.h"
#include "utils/parameters.h"
#include <dynamic_reconfigure/server.h>
#include <pandora_vision_hole_exploration/hole_fusion_cfgConfig.h>
#include <pandora_vision_hole_exploration/debug_cfgConfig.h>
//#include <pandora_vision_hole/filters_priority_cfgConfig.h>
//#include <pandora_vision_hole/filters_thresholds_cfgConfig.h>
#include <pandora_vision_hole_exploration/general_cfgConfig.h>
#include <pandora_vision_hole_exploration/validity_cfgConfig.h>

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
    struct HoleFusion
    {
      static int bin_to_find_mergable_size;
      static float valid_strong_probability;
      static float valid_medium_probability;
      static float valid_light_probability;
      static float max_depth_to_test_small_thresh;
      static float min_depth_to_test_big_thresh;
      static int small_rect_thresh;
      static int big_rect_thresh;
      static float rgb_distance_variance_thresh;
      static float rgb_small_distance_variance_thresh;
      static int hole_border_thresh;
      static float depth_difference_thresh;
      static int remove_unstuffed_holes;
      static int unstuffed_removal_method;
      static float difference_scanline_thresh;
      
    };


    struct Debug
    {
      // Show the holes that each of the depth and RGB nodes transmit to the
      // hole fusion node, on top of their respective origin images
      static bool show_respective_holes;
      //
      // Show all valid holes, from either the Depth or RGB source, or
      // the merges between them
      static bool show_valid_holes;
      // In the terminal's window, show the probabilities of candidate holes
      static bool show_probabilities;
    };


    //    //! Image representation specific parameters
    struct Image
    {
      // The depth sensor's horizontal field of view
      static float horizontal_field_of_view;
      //
      // The depth sensor's vertical field of view
      static float vertical_field_of_view;
      //
      //      // Fallback values. See the input point cloud callback of the
      //      // synchronizer node
      static int HEIGHT;
      static int WIDTH;

      // Method to scale the CV_32F images to CV_8UC1
      static int scale_method;
      //
    };
  };

}  // namespace pandora_vision

#endif  // UTILS_PARAMETERS_H
