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
  // Show the holes that each of the depth and RGB nodes transmit to the
  // hole fusion node, on top of their respective origin images
  bool Parameters::Debug::show_respective_holes = false;
  //
  // Show all valid holes, from either the Depth or RGB source, or
  // the merges between them
  bool Parameters::Debug::show_valid_holes = false;
  //
  //  // The product of this package: unique, valid holes
  //  bool HoleFusion::show_final_holes = false;
  //
  // In the terminal's window, show the probabilities of candidate holes
  bool Parameters::Debug::show_probabilities = false;


  int Parameters::HoleFusion::bin_to_find_mergable_size = 60;
  float Parameters::HoleFusion::valid_strong_probability = 1.0;
  float Parameters::HoleFusion::valid_medium_probability = 0.6;
  float Parameters::HoleFusion::valid_light_probability = 0.4;
  float Parameters::HoleFusion::max_depth_to_test_small_thresh = 2.5;
  float Parameters::HoleFusion::min_depth_to_test_big_thresh = 2.6;
  int Parameters::HoleFusion::small_rect_thresh = 900;
  int Parameters::HoleFusion::big_rect_thresh = 6400;
  float Parameters::HoleFusion::rgb_distance_variance_thresh = 0.5;
  float Parameters::HoleFusion::rgb_small_distance_variance_thresh = 0.2;
  int Parameters::HoleFusion::hole_border_thresh = 10;
  float Parameters::HoleFusion::depth_difference_thresh = 2.0;
  int Parameters::HoleFusion::remove_unstuffed_holes = 1;
  int Parameters::HoleFusion::unstuffed_removal_method = 1;
  float Parameters::HoleFusion::difference_scanline_thresh = 1.0;


  //  ////////////////// Image representation specific parameters //////////////////
  // The depth sensor's horizontal field of view in rads
  float Parameters::Image::horizontal_field_of_view =
    static_cast<float>(58) / 180 * M_PI;
  //
  // The depth sensor's vertical field of view in rads
  float Parameters::Image::vertical_field_of_view =
    static_cast<float>(45) / 180 * M_PI;
  //
  //  // Fallback values. See the input point cloud callback of the
  //  // synchronizer node
  int Parameters::Image::HEIGHT = 480;
  int Parameters::Image::WIDTH = 640;
  //
  //  // Depth and RGB images' representation method.
  //  // 0 if image used is used as obtained from the image sensor
  //  // 1 through wavelet analysis
  //  int Parameters::Image::image_representation_method = 1;
  //
  // Method to scale the CV_32F image to CV_8UC1
  int Parameters::Image::scale_method = 0;
  //

} // namespace pandora_vision
