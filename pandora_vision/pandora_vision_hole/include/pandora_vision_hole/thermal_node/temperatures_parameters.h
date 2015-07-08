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
 * Authors: Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/


#ifndef PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_PARAMETERS_H
#define PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_PARAMETERS_H

#include <ros/ros.h>
#include "pandora_vision_hole/temperatures_cfgConfig.h"
#include <dynamic_reconfigure/server.h>

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
  struct TemperatureParameters
  {
    /// The variables that define the temperature range that we search
    struct TemperatureRange
    {
      float lowTemperature;
      float highTemperature;
    };

    /// Blob detection - specific parameters
    struct Blob
    {
      int min_threshold;
      int max_threshold;
      int threshold_step;
      int min_area;
      int max_area;
      double min_convexity;
      double max_convexity;
      double min_inertia_ratio;
      double max_circularity;
      double min_circularity;
      bool filter_by_color;
      bool filter_by_circularity;
    };



    /// Debug-specific parameters
    struct Debug
    {
      // Show the thermal image that arrives in the thermal node
      bool show_temperature_image;

      // Show the holes that each of the depth and RGB nodes transmit to the
      // hole fusion node, on top of their respective origin images
      bool show_respective_holes;

      // Show all valid holes, from either the Depth or RGB source, or
      // the merges between them
      bool show_valid_holes;

      // The product of this package: unique, valid holes
      bool show_final_holes;

      bool show_find_holes;
      int show_find_holes_size;
    };

    /// Image representation specific parameters
    struct Image
    {
      // The depth sensor's horizontal field of view
      float horizontal_field_of_view;

      // The depth sensor's vertical field of view
      float vertical_field_of_view;

      // HEIGHT, WIDTH of thermal camera image
      int HEIGHT;
      int WIDTH;
    };

    /// Outline discovery specific parameters
    struct Outline
    {
      // The detection method used to obtain the outline of a blob
      // 0 for detecting by means of brushfire
      // 1 for detecting by means of raycasting
      int outline_detection_method;

      // When using raycast instead of brushfire to find the (approximate here)
      // outline of blobs, raycast_keypoint_partitions dictates the number of
      // rays, or equivalently, the number of partitions in which the blob is
      // partitioned in search of the blob's borders
      int raycast_keypoint_partitions;

      // Loose ends connection parameters
      int AB_to_MO_ratio;
      int minimum_curve_points;
    };
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_HOLE_THERMAL_NODE_TEMPERATURES_PARAMETERS_H
