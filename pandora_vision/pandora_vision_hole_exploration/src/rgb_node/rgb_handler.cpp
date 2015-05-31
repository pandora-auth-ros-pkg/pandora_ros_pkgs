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
 * Authors: Vasilis Bosdelekidis, Despoina Paschalidou, Alexandros Philotheou 
 *********************************************************************/

#include "rgb_node/rgb_handler.h"

namespace pandora_vision
{
  RgbHandler::RgbHandler(const std::string& ns) : VisionHandler<RgbPreProcessor, RgbProcessor,
    RgbPostProcessor>(ns)
  {
    activeStates_.push_back(state_manager_msgs::RobotModeMsg::MODE_EXPLORATION_RESCUE);
    activeStates_.push_back(state_manager_msgs::RobotModeMsg::MODE_IDENTIFICATION);
    activeStates_.push_back(state_manager_msgs::RobotModeMsg::MODE_SENSOR_HOLD);
    activeStates_.push_back(state_manager_msgs::RobotModeMsg::MODE_SENSOR_TEST);
  }
  
  RgbHandler::~RgbHandler() {}
  
  void RgbHandler::completeTransition()
  {
    ROS_INFO_NAMED(PKG_NAME, "[Hole Handler] : Transition Complete");
  }
}  // namespace pandora_vision
