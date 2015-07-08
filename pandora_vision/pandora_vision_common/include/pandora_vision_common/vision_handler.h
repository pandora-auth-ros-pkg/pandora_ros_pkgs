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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_COMMON_VISION_HANDLER_H
#define PANDORA_VISION_COMMON_VISION_HANDLER_H

#include <string>
#include <vector>

#include <sensor_msgs/Image.h>

#include "sensor_processor/dynamic_handler.h"

#include "pandora_vision_common/cv_mat_stamped.h"
#include "pandora_vision_common/pois_stamped.h"

namespace pandora_vision
{
  template <class PreProc, class Proc, class PostProc>
  class VisionHandler : public sensor_processor::DynamicHandler
  {
   public:
    /**
      * @brief Constructor
      * @param ns [const std::string&] The namespace of this handler's nodeHandle
      **/
    VisionHandler() {}
    virtual ~VisionHandler() {}

   protected:
    /**
      * @brief Function that performs all the needed procedures when the robot's
      * state is changed
      * @param newState [int] Robot's new state
      **/
    virtual void
    startTransition(int newState)
    {
      this->previousState_ = this->currentState_;
      this->currentState_ = newState;

      bool previouslyOff = true;
      bool currentlyOn = false;

      for (int ii = 0; ii < activeStates_.size(); ii++) {
        previouslyOff = (previouslyOff && this->previousState_ != ROBOT_STATES(activeStates_[ii]));
        currentlyOn = (currentlyOn || this->currentState_ == ROBOT_STATES(activeStates_[ii]));
      }

      if (!previouslyOff && !currentlyOn)
      {
        unloadPreProcessor();
        unloadProcessor();
        unloadPostProcessor();
      }
      else if (previouslyOff && currentlyOn)
      {
        loadPreProcessor<PreProc>("~preprocessor");
        loadProcessor<Proc>("~detector");
        loadPostProcessor<PostProc>("~postprocessor");
      }

      if (this->currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
      {
        unloadPreProcessor();
        unloadProcessor();
        unloadPostProcessor();

        ROS_INFO("[%s] Terminating", name_.c_str());
        ros::shutdown();
        return;
      }

      transitionComplete(this->currentState_);
    }
  };
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_VISION_HANDLER_H
