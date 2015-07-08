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
* Authors: Victor Daropoulos
*********************************************************************/

#ifndef PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_PARAMETERS_H
#define PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_PARAMETERS_H

#include <iostream>
#include <cstdlib>
#include <boost/shared_ptr.hpp>

#include "dynamic_reconfigure/server.h"
#include "pandora_vision_landoltc/landoltc_cfgConfig.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"

namespace pandora_vision
{
namespace pandora_vision_landoltc
{
  struct LandoltcParameters
  {
    void configLandoltC(const ros::NodeHandle& nh);

    /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_landoltc::landoltc_cfgConfig&]
    @param[in] level [const uint32_t] The level
    @return void
    **/
    void parametersCallback(
    const ::pandora_vision_landoltc::landoltc_cfgConfig& config,
    const uint32_t& level);

    dynamic_reconfigure::Server< ::pandora_vision_landoltc::landoltc_cfgConfig >::CallbackType f;

    /// The dynamic reconfigure (landoltc) parameters' server
    boost::shared_ptr< dynamic_reconfigure::Server< ::pandora_vision_landoltc::landoltc_cfgConfig > >
    server_;

    /// Threshold parameters
    double gradientThreshold;
    double centerThreshold;
    double huMomentsPrec;
    int adaptiveThresholdSubtractSize;
    bool visualization;
    double timerThreshold;
  };

}  // namespace pandora_vision_landoltc
}  // namespace pandora_vision
#endif  // PANDORA_VISION_LANDOLTC_LANDOLTC_2D_LANDOLTC_PARAMETERS_H
