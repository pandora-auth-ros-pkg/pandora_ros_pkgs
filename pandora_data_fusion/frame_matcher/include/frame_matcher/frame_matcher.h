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
 *********************************************************************/

#ifndef FRAME_MATCHER_FRAME_MATCHER_H
#define FRAME_MATCHER_FRAME_MATCHER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include "sensor_processor/dynamic_handler.h"
#include "sensor_processor/abstract_processor.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{
  /**
    * @class FrameMatcher class that implements Handler to organise frame
    * mather processors
    */
  class FrameMatcher : public sensor_processor::DynamicHandler
  {
   public:
    FrameMatcher();

    virtual void
    onInit();

   protected:
    /**
      * @brief Function that performs all the needed procedures when the robot's
      * state is changed
      * @param newState [int] Robot's new state
      */
    virtual void
    startTransition(int newState);

   private:
    ros::NodeHandle private_nh_;
    std::string name_;

    std::string preprocessor_type_;
    std::string postprocessor_type_;
  };
}  // namespace frame_matcher
}  // namespace pandora_data_fusion

#endif  // FRAME_MATCHER_FRAME_MATCHER_H
