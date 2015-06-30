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

#ifndef PANDORA_VISION_VICTIM_VICTIM_HANDLER_H
#define PANDORA_VISION_VICTIM_VICTIM_HANDLER_H

#include <string>
#include <vector>

#include "sensor_processor/handler.h"
#include "pandora_vision_victim/victim_image_preprocessor.h"
#include "pandora_vision_victim/victim_hole_preprocessor.h"
#include "pandora_vision_victim/victim_image_processor.h"
#include "pandora_vision_victim/victim_hole_processor.h"
#include "pandora_vision_victim/victim_postprocessor.h"

namespace pandora_vision
{
  class VictimHandler : public sensor_processor::Handler
  {
    public:
      explicit VictimHandler(const std::string& ns);
      virtual ~VictimHandler() {}

    private:
      virtual void startTransition(int newState);
      virtual void completeTransition();

    private:
      std::vector<int> holeActiveStates_;
      std::vector<int> imageActiveStates_;
  };
}  // namespace pandora_vision

#endif  // PANDORA_VISION_VICTIM_VICTIM_HANDLER_H
