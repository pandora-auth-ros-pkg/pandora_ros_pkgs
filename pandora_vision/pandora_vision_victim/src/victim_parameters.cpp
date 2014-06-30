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

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
  double VictimParameters::rgb_vj_weight = 0.2;
  double VictimParameters::depth_vj_weight = 0;
  double VictimParameters::rgb_svm_weight = 0.9;
  double VictimParameters::depth_svm_weight = 0;
  
  bool VictimParameters::debug_img = false;
  
  VictimParameters::VictimParameters(void)
  {
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&VictimParameters::parametersCallback,
        this, _1, _2));
  }
  
    /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
    @param[in] level [const uint32_t] The level 
    @return void
  **/
  void VictimParameters::parametersCallback(
    const pandora_vision_victim::victim_dyn_reconfConfig& config,
    const uint32_t& level)
  {
    VictimParameters::rgb_vj_weight = config.rgb_vj_weight;
    VictimParameters::depth_vj_weight = config.depth_vj_weight;
    VictimParameters::rgb_svm_weight = config.rgb_svm_weight;
    VictimParameters::depth_svm_weight = config.depth_svm_weight;
    VictimParameters::debug_img = config.debug_img;
  }
}// namespace pandora_vision
