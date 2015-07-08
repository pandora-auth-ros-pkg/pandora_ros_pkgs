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
 * Authors: Vasilis Bosdelekidis
 *********************************************************************/

#include "pandora_vision_obstacle/barrel_detection/parameters.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
namespace pandora_vision_obstacle
{

  BarrelParametersHandler::BarrelParametersHandler(const std::string& name,
                              const ros::NodeHandle& nh) : serverBarrel(nh)
  {
    serverBarrel.setCallback(boost::bind(&BarrelParametersHandler::parametersCallbackBarrel, this, _1, _2));
  }

  /**
    @brief The function called when a parameter is changed
    @param[in] configBarrel [const pandora_vision_obstacle::barrel_nodeConfig&]
    @param[in] level [const uint32_t]
    @return void
   **/
  void BarrelParametersHandler::parametersCallbackBarrel(
      const ::pandora_vision_obstacle::barrel_nodeConfig& configBarrel,
      const uint32_t& level)
  {
    //////////////////////////////// Debug parameters ////////////////////////////

    //// Show the image with the segmented roi from fsd
    BarrelDetection::show_respective_barrel =
      configBarrel.show_respective_barrel;

    BarrelDetection::show_valid_barrel =
      configBarrel.show_valid_barrel;

    ////////////////////// Parameters specific to the FSD ///////////////////

    BarrelDetection::fsd_canny_thresh_1 =
      configBarrel.fsd_canny_thresh_1;
    BarrelDetection::fsd_canny_thresh_2 =
      configBarrel.fsd_canny_thresh_2;
    BarrelDetection::fsd_min_pair_dist =
      configBarrel.fsd_min_pair_dist;
    BarrelDetection::fsd_max_pair_dist =
      configBarrel.fsd_max_pair_dist;
    BarrelDetection::fsd_no_of_peaks =
      configBarrel.fsd_no_of_peaks;
    BarrelDetection::roi_variance_thresh =
      configBarrel.roi_variance_thresh;
    BarrelDetection::differential_depth_unsymmetry_thresh =
      configBarrel.differential_depth_unsymmetry_thresh;
    BarrelDetection::symmetry_line_depth_difference_thresh =
      configBarrel.symmetry_line_depth_difference_thresh;
    BarrelDetection::curve_approximation_max_epsilon =
      configBarrel.curve_approximation_max_epsilon;
    BarrelDetection::min_circle_overlapping =
      configBarrel.min_circle_overlapping;
    BarrelDetection::max_corner_thresh =
      configBarrel.max_corner_thresh;
  }

  // Show the barrel that depth node segments through FSD
  bool BarrelDetection::show_respective_barrel = false;
  //
  // Show validated barrel, which is the alert transmitted to
  // data fusion
  bool BarrelDetection::show_valid_barrel = false;
  //
  // Parameters for Fast Symmetry Detection Algorithm
  int BarrelDetection::fsd_canny_thresh_1 = 0;
  int BarrelDetection::fsd_canny_thresh_2 = 27;
  int BarrelDetection::fsd_min_pair_dist = 100;
  int BarrelDetection::fsd_max_pair_dist = 640;
  int BarrelDetection::fsd_no_of_peaks = 1;
  float BarrelDetection::roi_variance_thresh = 60.0;
  float BarrelDetection::differential_depth_unsymmetry_thresh = 30.0;
  float BarrelDetection::symmetry_line_depth_difference_thresh = 11.0;
  float BarrelDetection::curve_approximation_max_epsilon = 1600.0;
  float BarrelDetection::min_circle_overlapping = 0.25;
  float BarrelDetection::max_corner_thresh = 35.0;

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
