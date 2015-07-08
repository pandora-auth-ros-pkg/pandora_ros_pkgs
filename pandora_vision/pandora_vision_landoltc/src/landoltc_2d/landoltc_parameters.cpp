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

#include "pandora_vision_landoltc/landoltc_2d/landoltc_parameters.h"

namespace pandora_vision
{
namespace pandora_vision_landoltc
{
  void LandoltcParameters::configLandoltC(const ros::NodeHandle& nh)
  {
    //!< The dynamic reconfigure parameter's callback
    server_.reset( new dynamic_reconfigure::Server< ::pandora_vision_landoltc::landoltc_cfgConfig >(nh) );
    server_->setCallback(boost::bind(&LandoltcParameters::parametersCallback, this, _1, _2));

    nh.param("/gradientThreshold", gradientThreshold, 60.);
    nh.param("/centerThreshold", centerThreshold, 90.);
    nh.param("/huMomentsPrec", huMomentsPrec, 0.3);
    nh.param("/adaptiveThresholdSubtractSize", adaptiveThresholdSubtractSize, 2);
    nh.param("/visualization", visualization, false);
    nh.param("/timerThreshold", timerThreshold, 0.16);
  }

  /**
  @brief The function called when a parameter is changed
  @param[in] config [const pandora_vision_landoltc::landoltc_cfgConfig&]
  @param[in] level [const uint32_t] The level
  @return void
  **/
  void LandoltcParameters::parametersCallback(
    const ::pandora_vision_landoltc::landoltc_cfgConfig& config,
    const uint32_t& level)
  {
    //!< Threshold parameters
    gradientThreshold = config.gradientThreshold;
    centerThreshold = config.centerThreshold;
    huMomentsPrec = config.huMomentsPrec;
    adaptiveThresholdSubtractSize = config.adaptiveThresholdSubtractSize;
    visualization = config.visualization;
    timerThreshold = config.timerThreshold;
  }
}  // namespace pandora_vision_landoltc
}  // namespace pandora_vision
