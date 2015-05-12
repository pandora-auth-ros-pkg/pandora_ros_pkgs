/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_motion/motion_parameters.h"

namespace pandora_vision
{
  void MotionParameters::configMotion(const ros::NodeHandle& nh)
  {
    //!< Background segmentation parameters
    /// Length of the history according to which we calculate background image
    nh.param("/history", history, 10);
    
    /// Threshold of the sqaured Mahalanobis distance to decide whether
    /// it is well described by the background model
    nh.param("/varThreshold", varThreshold, 16);
    
    /// Parameter defining whether shadow detection should be enabled
    nh.param("/bShadowDetection", bShadowDetection, true);
    
    /// Maximum allowed number of mixture components
    nh.param("/nmixtures", nmixtures, 3);
    
    //!< Threshold parameters
    /// Threshold between pixel (grayscale) values to be considered "different"
    /// between 2 frames
    nh.param("/diff_threshold", diff_threshold, 45);
    
    /// Evaluation threshold: higher value means a lot of movement
    nh.param("/motion_high_thres", motion_high_thres, 7500.);
    
    /// Evaluation threshold: higher value means a little movement -
    /// less means no movement at all
    nh.param("/motion_low_thres", motion_low_thres, 200.);
    nh.param("/visualization", visualization, false);
    nh.param("/show_image", show_image, false);
    nh.param("/show_background", show_background, false);
    nh.param("/show_diff_image", show_diff_image, false);
    nh.param("/show_moving_objects_contours", show_moving_objects_contours, false);
  }
}  // namespace pandora_vision
